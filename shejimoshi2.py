from servo_driver import ServoDriver
import time
import math
import numpy as np
import threading
import paho.mqtt.client as mqtt
import json
import cv2
import base64
import signal
import sys
import os
import socket
import serial
from collections import deque
from abc import ABC, abstractmethod
import logging
import datetime
import csv
import shutil
import glob
from pathlib import Path

# 配置日志系统，确保输出到控制台
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)  # 显式指定输出到标准输出
    ]
)
logger = logging.getLogger("SnakeRobot")

# 添加直接打印功能，确保终端显示
def print_terminal(message):
    """直接打印到终端，确保消息可见"""
    print(message, flush=True)

# 当前日期/时间和用户设置
CURRENT_DATE = "2025-05-17 03:17:23"
CURRENT_USER = "12ljf"
print_terminal(f"Starting as user: {CURRENT_USER}, Date: {CURRENT_DATE}")

# 数据存储配置
DATA_STORAGE_DIR = "/home/pi/snake_robot_data"  # 数据存储目录
DATA_BACKUP_DIR = "/home/pi/snake_robot_data/backup"  # 备份目录
SENSOR_DATA_FILE = "sensor_data.csv"  # 传感器数据文件名
LOG_DATA_FILE = "log_data.txt"  # 日志数据文件名
IMAGE_DATA_DIR = "images"  # 图像数据目录
MAX_STORAGE_SIZE_MB = 500  # 最大存储空间（MB）
DATA_SEND_INTERVAL = 60  # 数据发送间隔（秒）
MAX_FILES_PER_BATCH = 50  # 每批次最大文件数
DATA_RETENTION_DAYS = 7  # 数据保留天数

# 尝试导入SPL06压力/温度传感器
try:
    from SPL06 import SPL0601
    PRESSURE_SENSOR_AVAILABLE = True
    print_terminal("SPL06 pressure/temperature sensor available")
except ImportError:
    PRESSURE_SENSOR_AVAILABLE = False
    print_terminal("SPL06 pressure/temperature sensor not available")

# 尝试导入SGP40空气质量传感器
try:
    from SGP40 import SGP40
    AIR_QUALITY_SENSOR_AVAILABLE = True
    print_terminal("SGP40 air quality sensor available")
except ImportError:
    AIR_QUALITY_SENSOR_AVAILABLE = False
    print_terminal("SGP40 air quality sensor not available")

# 常量
SNAKE_LENGTH = 12
DEFAULT_TEMPERATURE = 25.0
DEFAULT_PRESSURE = 1013.0
DEFAULT_AIR_QUALITY = 80

# MQTT设置
MQTT_BROKER = "47.107.36.182"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "USER002"
MQTT_USERNAME = "USER002"
MQTT_PASSWORD = "UQU92K77cpxc2Tm"

# 主题
MQTT_TOPIC_PUBLISH = "USER002"    # 机器人发布传感器数据
MQTT_TOPIC_SUBSCRIBE = "USER001"  # 机器人订阅控制命令
MQTT_TOPIC_DATA_BATCH = "USER002/data_batch"  # 批量数据发送主题

# 摄像头设置
CAMERA_RESOLUTION = (320, 240)
JPEG_QUALITY = 80
FRAME_RATE = 10
ENABLE_DEBUG = True
TRY_ALTERNATE_BACKENDS = True

# 串口选项
SERIAL_PORTS = ['/dev/ttyS0', '/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyACM0']

# 单例模式元类
class Singleton(type):
    """单例模式元类，确保一个类只有一个实例"""
    _instances = {}
    
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

# 辅助函数：将NumPy类型转换为Python原生类型
def convert_to_serializable(obj):
    """将NumPy数据类型转换为Python原生类型，用于JSON序列化"""
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, np.bool_):
        return bool(obj)
    elif isinstance(obj, dict):
        return {k: convert_to_serializable(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_to_serializable(i) for i in obj]
    else:
        return obj

# 数据存储管理类
class DataStorageManager(metaclass=Singleton):
    """管理传感器数据本地存储和发送"""
    
    def __init__(self, base_dir=DATA_STORAGE_DIR):
        self.base_dir = base_dir
        self.backup_dir = os.path.join(base_dir, "backup")
        self.image_dir = os.path.join(base_dir, IMAGE_DATA_DIR)
        self.current_date = datetime.datetime.now().strftime("%Y-%m-%d")
        self.sensor_data_file = os.path.join(self.base_dir, f"{self.current_date}_{SENSOR_DATA_FILE}")
        self.log_data_file = os.path.join(self.base_dir, f"{self.current_date}_{LOG_DATA_FILE}")
        self.pending_files = []  # 待发送文件列表
        self.send_lock = threading.Lock()  # 发送操作的锁
        
        # 初始化存储目录
        self._init_directories()
        
        # 初始化传感器数据文件
        self._init_sensor_data_file()
        
        # 启动定时清理任务
        self._schedule_cleanup()
    
    def _init_directories(self):
        """初始化存储目录结构"""
        os.makedirs(self.base_dir, exist_ok=True)
        os.makedirs(self.backup_dir, exist_ok=True)
        os.makedirs(self.image_dir, exist_ok=True)
        
        # 按日期创建图像子目录
        self.today_image_dir = os.path.join(self.image_dir, self.current_date)
        os.makedirs(self.today_image_dir, exist_ok=True)
        
        print_terminal(f"数据存储目录初始化完成: {self.base_dir}")
    
    def _init_sensor_data_file(self):
        """初始化传感器数据CSV文件，添加表头"""
        # 如果文件不存在，创建并添加表头
        if not os.path.exists(self.sensor_data_file):
            with open(self.sensor_data_file, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'temperature', 'pressure', 'air_quality', 
                              'current_gait', 'image_filename']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                print_terminal(f"创建新的传感器数据文件: {self.sensor_data_file}")
    
    def _schedule_cleanup(self):
        """启动定时清理任务"""
        cleanup_thread = threading.Thread(target=self._periodic_cleanup, daemon=True)
        cleanup_thread.start()
    
    def _periodic_cleanup(self):
        """定期清理旧数据"""
        while True:
            try:
                # 检查存储空间使用情况
                self._check_storage_usage()
                
                # 清理过期数据
                self._cleanup_old_data()
                
                # 每天检查一次
                time.sleep(86400)  # 24小时
            except Exception as e:
                print_terminal(f"执行定期清理时出错: {e}")
                time.sleep(3600)  # 出错后1小时后重试
    
    def _check_storage_usage(self):
        """检查存储空间使用情况，如果超过限制则清理"""
        try:
            total_size = 0
            
            # 计算数据目录的总大小
            for dirpath, dirnames, filenames in os.walk(self.base_dir):
                for f in filenames:
                    fp = os.path.join(dirpath, f)
                    if os.path.exists(fp):
                        total_size += os.path.getsize(fp)
            
            # 转换为MB
            total_size_mb = total_size / (1024 * 1024)
            
            print_terminal(f"当前数据存储使用: {total_size_mb:.2f}MB / {MAX_STORAGE_SIZE_MB}MB")
            
            # 如果超过限制，清理旧数据
            if total_size_mb > MAX_STORAGE_SIZE_MB:
                print_terminal(f"存储空间超过限制 ({total_size_mb:.2f}MB)，开始清理旧数据...")
                self._cleanup_by_age()
        
        except Exception as e:
            print_terminal(f"检查存储空间时出错: {e}")
    
    def _cleanup_old_data(self):
        """清理超过保留期限的数据"""
        try:
            # 计算截止日期
            cutoff_date = datetime.datetime.now() - datetime.timedelta(days=DATA_RETENTION_DAYS)
            cutoff_str = cutoff_date.strftime("%Y-%m-%d")
            
            # 查找所有数据文件
            all_files = glob.glob(os.path.join(self.base_dir, "*.csv"))
            all_files.extend(glob.glob(os.path.join(self.base_dir, "*.txt")))
            
            # 添加图像目录
            image_dirs = glob.glob(os.path.join(self.image_dir, "*"))
            
            # 清理旧数据文件
            for file_path in all_files:
                filename = os.path.basename(file_path)
                # 提取文件名中的日期部分（假设格式为YYYY-MM-DD_filename）
                date_part = filename.split("_")[0]
                
                # 如果日期早于截止日期，则删除或归档
                if date_part < cutoff_str:
                    # 归档到备份目录，而不是直接删除
                    backup_path = os.path.join(self.backup_dir, filename)
                    shutil.move(file_path, backup_path)
                    print_terminal(f"已归档过期数据文件: {filename}")
            
            # 清理旧图像目录
            for dir_path in image_dirs:
                dir_name = os.path.basename(dir_path)
                # 如果目录名是日期格式，且早于截止日期，则归档
                if dir_name.count("-") == 2 and dir_name < cutoff_str:
                    backup_path = os.path.join(self.backup_dir, dir_name)
                    os.makedirs(backup_path, exist_ok=True)
                    
                    # 移动图像文件
                    image_files = glob.glob(os.path.join(dir_path, "*.jpg"))
                    for img_file in image_files:
                        img_name = os.path.basename(img_file)
                        shutil.move(img_file, os.path.join(backup_path, img_name))
                    
                    # 移除空目录
                    os.rmdir(dir_path)
                    print_terminal(f"已归档过期图像目录: {dir_name}")
        
        except Exception as e:
            print_terminal(f"清理旧数据时出错: {e}")
    
    def _cleanup_by_age(self):
        """按照年龄清理数据，从最旧的开始清理"""
        try:
            # 获取所有数据文件并按修改时间排序
            all_files = []
            
            # 收集CSV和日志文件
            for ext in ["*.csv", "*.txt"]:
                all_files.extend(glob.glob(os.path.join(self.base_dir, ext)))
            
            # 按修改时间排序
            all_files.sort(key=os.path.getmtime)
            
            # 从最旧的文件开始归档，直到空间使用量降至限制的80%
            target_size = MAX_STORAGE_SIZE_MB * 0.8 * 1024 * 1024  # 转为字节
            
            for file_path in all_files:
                # 检查当前存储使用量
                current_size = sum(os.path.getsize(f) for f in glob.glob(os.path.join(self.base_dir, "*.*")))
                
                if current_size <= target_size:
                    break
                
                # 归档而不是删除
                filename = os.path.basename(file_path)
                backup_path = os.path.join(self.backup_dir, filename)
                shutil.move(file_path, backup_path)
                print_terminal(f"已归档文件以释放空间: {filename}")
            
            # 如果仍然超过限制，清理图像文件
            current_size = sum(os.path.getsize(f) for f in glob.glob(os.path.join(self.base_dir, "*.*")))
            if current_size > target_size:
                # 获取所有图像目录
                image_dirs = glob.glob(os.path.join(self.image_dir, "*"))
                image_dirs.sort(key=os.path.getmtime)
                
                for dir_path in image_dirs:
                    # 再次检查存储使用量
                    current_size = sum(os.path.getsize(f) for f in glob.glob(os.path.join(self.base_dir, "*.*")))
                    if current_size <= target_size:
                        break
                    
                    # 归档整个目录
                    dir_name = os.path.basename(dir_path)
                    backup_path = os.path.join(self.backup_dir, dir_name)
                    os.makedirs(backup_path, exist_ok=True)
                    
                    # 移动图像文件
                    image_files = glob.glob(os.path.join(dir_path, "*.jpg"))
                    for img_file in image_files:
                        img_name = os.path.basename(img_file)
                        shutil.move(img_file, os.path.join(backup_path, img_name))
                    
                    # 移除空目录
                    os.rmdir(dir_path)
                    print_terminal(f"已归档图像目录以释放空间: {dir_name}")
        
        except Exception as e:
            print_terminal(f"按年龄清理数据时出错: {e}")
    
    def store_sensor_data(self, data):
        """存储传感器数据到CSV文件"""
        try:
            # 检查日期是否变化，需要创建新文件
            current_date = datetime.datetime.now().strftime("%Y-%m-%d")
            if current_date != self.current_date:
                self.current_date = current_date
                self.sensor_data_file = os.path.join(self.base_dir, f"{self.current_date}_{SENSOR_DATA_FILE}")
                self.log_data_file = os.path.join(self.base_dir, f"{self.current_date}_{LOG_DATA_FILE}")
                self.today_image_dir = os.path.join(self.image_dir, self.current_date)
                os.makedirs(self.today_image_dir, exist_ok=True)
                self._init_sensor_data_file()
            
            # 准备数据行
            timestamp = datetime.datetime.fromtimestamp(data.get("timestamp", time.time())).strftime("%Y-%m-%d %H:%M:%S")
            temperature = data.get("temperature", DEFAULT_TEMPERATURE)
            pressure = data.get("pressure", DEFAULT_PRESSURE)
            air_quality = data.get("air_quality", DEFAULT_AIR_QUALITY)
            current_gait = data.get("current_gait", "无")
            
            # 存储图像（如果有）
            image_filename = ""
            if "camera_frame" in data:
                image_filename = self._store_image(data["camera_frame"], timestamp)
            
            # 写入CSV
            with open(self.sensor_data_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([timestamp, temperature, pressure, air_quality, current_gait, image_filename])
            
            # 将文件添加到待发送列表
            with self.send_lock:
                if self.sensor_data_file not in self.pending_files:
                    self.pending_files.append(self.sensor_data_file)
            
            return True
        
        except Exception as e:
            print_terminal(f"存储传感器数据时出错: {e}")
            return False
    
    def _store_image(self, image_data, timestamp):
        """存储图像数据到文件"""
        try:
            # 构造文件名（使用时间戳和随机数以避免冲突）
            filename = f"img_{timestamp.replace(' ', '_').replace(':', '-')}_{hash(time.time()) % 1000}.jpg"
            filepath = os.path.join(self.today_image_dir, filename)
            
            # 将base64解码为二进制
            image_bytes = base64.b64decode(image_data)
            
            # 写入文件
            with open(filepath, 'wb') as f:
                f.write(image_bytes)
            
            # 记录日志
            if os.path.getsize(filepath) > 0:
                print_terminal(f"已保存图像: {filename}, 大小: {os.path.getsize(filepath)/1024:.1f}KB")
            else:
                print_terminal(f"保存图像失败: {filename}")
                return ""
            
            # 将文件添加到待发送列表
            with self.send_lock:
                if filepath not in self.pending_files:
                    self.pending_files.append(filepath)
            
            # 返回相对路径（从存储根目录开始）
            return os.path.join(IMAGE_DATA_DIR, self.current_date, filename)
        
        except Exception as e:
            print_terminal(f"存储图像数据时出错: {e}")
            return ""
    
    def log_event(self, event_type, message):
        """记录事件到日志文件"""
        try:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            with open(self.log_data_file, 'a') as f:
                f.write(f"{timestamp} [{event_type}] {message}\n")
            
            # 将文件添加到待发送列表
            with self.send_lock:
                if self.log_data_file not in self.pending_files:
                    self.pending_files.append(self.log_data_file)
            
            return True
        
        except Exception as e:
            print_terminal(f"记录事件时出错: {e}")
            return False
    
    def get_pending_files(self, max_files=MAX_FILES_PER_BATCH):
        """获取待发送的文件列表"""
        with self.send_lock:
            # 返回待发送文件的副本（最多max_files个）
            return self.pending_files[:max_files]
    
    def mark_files_as_sent(self, files):
        """标记文件为已发送"""
        with self.send_lock:
            for file in files:
                if file in self.pending_files:
                    self.pending_files.remove(file)
    
    def prepare_batch_data(self, files):
        """准备批量发送的数据"""
        batch_data = {
            "timestamp": time.time(),
            "files": []
        }
        
        for file_path in files:
            if not os.path.exists(file_path):
                continue
                
            file_info = {
                "path": os.path.relpath(file_path, self.base_dir),
                "size": os.path.getsize(file_path),
                "timestamp": os.path.getmtime(file_path)
            }
            
            # 根据文件类型添加内容
            _, ext = os.path.splitext(file_path)
            
            if ext.lower() == '.csv':
                # CSV文件，添加行数信息
                with open(file_path, 'r') as f:
                    lines = f.readlines()
                    file_info["lines"] = len(lines)
                    file_info["type"] = "csv"
                    # 不包含内容，因为可能太大
            
            elif ext.lower() == '.txt':
                # 文本文件，添加内容
                with open(file_path, 'r') as f:
                    content = f.read()
                    file_info["content"] = content
                    file_info["type"] = "text"
            
            elif ext.lower() in ['.jpg', '.jpeg', '.png']:
                # 图像文件，添加base64编码
                with open(file_path, 'rb') as f:
                    content = base64.b64encode(f.read()).decode('utf-8')
                    file_info["content"] = content
                    file_info["type"] = "image"
            
            batch_data["files"].append(file_info)
        
        return batch_data

# 传感器工厂类
class SensorFactory:
    @staticmethod
    def create_pressure_sensor():
        try:
            from SPL06 import SPL0601
            sensor = SPL0601(bus_num=1)
            print_terminal("SPL06 pressure/temperature sensor initialized")
            return sensor
        except ImportError:
            print_terminal("SPL06 pressure/temperature sensor not available")
            return None
        except Exception as e:
            print_terminal(f"Failed to initialize pressure sensor: {e}")
            return None
            
    @staticmethod
    def create_air_quality_sensor():
        try:
            from SGP40 import SGP40
            sensor = SGP40(bus=1)
            sensor.begin(duration=5)  # 短时预热
            print_terminal("SGP40 air quality sensor initialized")
            return sensor
        except ImportError:
            print_terminal("SGP40 air quality sensor not available")
            return None
        except Exception as e:
            print_terminal(f"Failed to initialize air quality sensor: {e}")
            return None

# 移动策略的抽象基类
class MovementStrategy(ABC):
    @abstractmethod
    def execute(self, robot, running_flag):
        pass

# 蠕动前进策略
class RudongStrategy(MovementStrategy):
    def execute(self, robot, running_flag):
        print_terminal("【执行动作】Starting rudong (forward movement)")
        # 记录事件
        robot.data_storage.log_event("MOVEMENT", "开始蠕动前进")
        
        # 先重置到中立位置
        robot.reset_to_neutral()
        
        t = 0  # 重置时间计数器
        # 继续移动，直到停止标志设置
        while running_flag.is_set(): 
            for i in range(0, SNAKE_LENGTH, 2):
                if not running_flag.is_set():
                    break
                if i == 0:
                    s = 150 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400 + 50
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)         
                if i == 2:
                    s = 150 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 4 or i == 6 or i == 8:
                    s = 150 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 10:
                    s = 150 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400 - 80
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
            if not running_flag.is_set():
                break
            t = t + 1
        
        # 记录结束事件
        robot.data_storage.log_event("MOVEMENT", "结束蠕动前进")

# 后退策略
class HoutuiStrategy(MovementStrategy):
    def execute(self, robot, running_flag):
        print_terminal("【执行动作】Starting houtui (backward movement)")
        # 记录事件
        robot.data_storage.log_event("MOVEMENT", "开始后退")
        
        t = 0  # 重置时间计数器
        while running_flag.is_set():
            for i in range(0, SNAKE_LENGTH, 2):
                if not running_flag.is_set():
                    break
                if i == 10:
                    s = 150 * (math.sin(math.pi * t / 25 + math.pi * 0 / 4)) + 400 + 50
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)         
                if i == 8:
                    s = 150 * (math.sin(math.pi * t / 25 + math.pi * 2 / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 6:
                    s = 150 * (math.sin(math.pi * t / 25 + math.pi * 4 / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 4:
                    s = 150 * (math.sin(math.pi * t / 25 + math.pi * 6 / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 2:
                    s = 120 * (math.sin(math.pi * t / 25 + math.pi * 8 / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 0:
                    s = 120 * (math.sin(math.pi * t / 25 + math.pi * 10 / 4)) + 400 - 80
                    robot.servo.move_time_write(i, 500, 4)
                    time.sleep(0.004)
            if not running_flag.is_set():
                break
            t = t + 1
        
        # 记录结束事件
        robot.data_storage.log_event("MOVEMENT", "结束后退")

# 左转策略
class ZuozhuanStrategy(MovementStrategy):
    def execute(self, robot, running_flag):
        print_terminal("【执行动作】Starting zuozhuan (left turn)")
        # 记录事件
        robot.data_storage.log_event("MOVEMENT", "开始左转")
        
        # 左转的初始位置设置
        for i in range(1, 4):
            if not running_flag.is_set():
                return
            robot.servo.move_time_write(1, 410 + 20 * i, 20)
            robot.servo.move_time_write(5, 400 + 20 * i, 20)
            robot.servo.move_time_write(9, 370 + 20 * i, 20)
            robot.servo.move_time_write(3, 410 + 20 * i, 20)
            robot.servo.move_time_write(7, 400 + 20 * i, 20)
            robot.servo.move_time_write(11, 370 + 20 * i, 20)
            time.sleep(0.060)

        t = 0
        # 继续移动，直到停止标志设置
        while running_flag.is_set():
            for i in range(0, SNAKE_LENGTH, 2):
                if not running_flag.is_set():
                    break
                if i == 0:
                    s = 130 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400 + 50
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)         
                if i == 2:
                    s = 130 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 4 or i == 6 or i == 8:
                    s = 130 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 10:
                    s = 130 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400 - 80
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
          
            t = t + 1
        
        # 记录结束事件
        robot.data_storage.log_event("MOVEMENT", "结束左转")

# 右转策略
class YouzhuanStrategy(MovementStrategy):
    def execute(self, robot, running_flag):
        print_terminal("【执行动作】Starting youzhuan (right turn)")
        # 记录事件
        robot.data_storage.log_event("MOVEMENT", "开始右转")
        
        # 右转的初始位置设置
        for i in range(1, 4):
            if not running_flag.is_set():
                return
            robot.servo.move_time_write(1, 410 - 20 * i, 20)
            robot.servo.move_time_write(5, 400 - 20 * i, 20)
            robot.servo.move_time_write(9, 370 - 20 * i, 20)
            robot.servo.move_time_write(3, 410 - 20 * i, 20)
            robot.servo.move_time_write(7, 400 - 20 * i, 20)
            robot.servo.move_time_write(11, 370 - 20 * i, 20)
            time.sleep(0.060)

        t = 0
        # 继续移动，直到停止标志设置
        while running_flag.is_set():
            for i in range(0, SNAKE_LENGTH, 2):
                if not running_flag.is_set():
                    break
                if i == 0:
                    s = 130 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400 + 50
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)         
                if i == 2:
                    s = 130 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 4 or i == 6 or i == 8:
                    s = 130 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 10:
                    s = 130 * (math.sin(math.pi * t / 25 + math.pi * i / 4)) + 400 - 80
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
            
            t = t + 1
        
        # 记录结束事件
        robot.data_storage.log_event("MOVEMENT", "结束右转")

# 蜿蜒策略
class WanyanStrategy(MovementStrategy):
    def execute(self, robot, running_flag):
        print_terminal("【执行动作】Starting wanyan (winding movement)")
        # 记录事件
        robot.data_storage.log_event("MOVEMENT", "开始蜿蜒运动")
        
        # 先重置到中立位置
        robot.reset_to_neutral()
        
        t = 0  # 重置时间计数器
        while running_flag.is_set(): 
            for i in range(0, SNAKE_LENGTH, 2):
                if not running_flag.is_set():
                    break
                if i == 0:
                    s = 180 * (math.sin(math.pi * t / 30 + math.pi * i / 5)) + 400 + 50
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)         
                if i == 2:
                    s = 180 * (math.sin(math.pi * t / 30 + math.pi * i / 5)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 4 or i == 6 or i == 8:
                    s = 180 * (math.sin(math.pi * t / 30 + math.pi * i / 5)) + 400
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
                if i == 10:
                    s = 180 * (math.sin(math.pi * t / 30 + math.pi * i / 5)) + 400 - 80
                    robot.servo.move_time_write(i, 800 - int(s), 4)
                    time.sleep(0.004)
            if not running_flag.is_set():
                break
            t = t + 1
        
        # 记录结束事件
        robot.data_storage.log_event("MOVEMENT", "结束蜿蜒运动")
 
class fangunL(MovementStrategy):
    def execute(self, robot, running_flag):
        print_terminal("【执行动作】Starting wanyan (winding movement)")
        # 记录事件
        robot.data_storage.log_event("MOVEMENT", "开始蜿蜒运动")
        
        # 先重置到中立位置
        robot.reset_to_neutral()
        
        t = 0  # 重置时间计数器
        while running_flag.is_set(): 
            for i in range(0, SNAKE_LENGTH, 2):
                if not running_flag.is_set():
                    break
              
            
                for i in range(0,SNAKE_LENGTH,1):
                    print(i)
                    robot.servo.move_time_write(i,400,1000)
                    if i==1 or i==3:
                        robot.servo.move_time_write(i,410,1000)
                    if i==9 or i==11:
                        robot.servo.move_time_write(i, 370, 1000) 
                time.sleep(2)
                for t in range(20):
                    robot.servo.move_time_write(3, 250, 1000)
                    robot.servo.move_time_write(9, 250, 1000)  
                    time.sleep(1)
                    robot.servo.move_time_write(2, 250, 1000)
                    robot.servo.move_time_write(10, 250, 1000)    
                    time.sleep(1)
                    robot.servo.move_time_write(3, 550, 1000)
                    robot.servo.move_time_write(9, 550, 1000)  
                    time.sleep(1)
                    robot.servo.move_time_write(2, 550, 1000)
                    robot.servo.move_time_write(10, 550, 1000)    
                    time.sleep(1)
                    #t=t+1
                if not running_flag.is_set():
                    break
                t = t + 1
        # 记录结束事件
        robot.data_storage.log_event("MOVEMENT", "结束蜿蜒运动")    
class fangunR(MovementStrategy):
    def execute(self, robot, running_flag):
        print_terminal("【执行动作】Starting wanyan (winding movement)")
        # 记录事件
        robot.data_storage.log_event("MOVEMENT", "开始蜿蜒运动")
        
        # 先重置到中立位置
        robot.reset_to_neutral()
        
        t = 0  # 重置时间计数器
        while running_flag.is_set(): 
            for i in range(0, SNAKE_LENGTH, 2):
                if not running_flag.is_set():
                    break
              
            
                for i in range(0,SNAKE_LENGTH,1):
                    print(i)
                    robot.servo.move_time_write(i,400,1000)
                    if i==1 or i==3:
                        robot.servo.move_time_write(i,410,1000)
                    if i==9 or i==11:
                        robot.servo.move_time_write(i, 370, 1000) 
                time.sleep(2)
                for t in range(20):
                    robot.servo.move_time_write(3, 550, 1000)
                    robot.servo.move_time_write(9, 550, 1000)  
                    time.sleep(1)
                    robot.servo.move_time_write(2, 250, 1000)
                    robot.servo.move_time_write(10, 250, 1000)    
                    time.sleep(1)
                    robot.servo.move_time_write(3, 250, 1000)
                    robot.servo.move_time_write(9, 250, 1000)  
                    time.sleep(1)
                    robot.servo.move_time_write(2, 550, 1000)
                    robot.servo.move_time_write(10, 550, 1000)    
                    time.sleep(1)
                    t=t+1        
        
 
 
 

class WanyanStrategy2(MovementStrategy):
    def execute(self, robot, running_flag):
        print_terminal("【执行动作】Starting wanyan (winding movement)")
        # 记录事件
        robot.data_storage.log_event("MOVEMENT", "开始蜿蜒运动")
        
        # 先重置到中立位置
        robot.reset_to_neutral()
        
        t = 0  # 重置时间计数器
        while running_flag.is_set(): 
            for i in range(0, SNAKE_LENGTH, 2):
                if not running_flag.is_set():
                    break
                robot.servo.move_time_write(0,430,2)
                    #time.sleep(0.002)
                for i in range(1,SNAKE_LENGTH,2):    ##1，3，5，7，9，11
                        s=210*(math.sin(math.pi*t/110+math.pi*(i-1)/(2.0*4)))+410+i*1.2
                        #print(12-i,int(s),self.t)
                        print('蜿蜒直行')
                        robot.servo.move_time_write(12-i,int(s),4)
                        time.sleep(0.004)
                        t = t + 1
 
        # 记录结束事件
        robot.data_storage.log_event("MOVEMENT", "结束蜿蜒运动")
 
 
 
 
 
 
 
 
 
 
 
 
 
 
# 复位策略
class FuweiStrategy(MovementStrategy):
    def execute(self, robot, running_flag):
        print_terminal("【执行动作】Executing fuwei (reset)")
        # 记录事件
        robot.data_storage.log_event("MOVEMENT", "执行复位操作")
        
        for i in range(0, SNAKE_LENGTH, 1):
            if not running_flag.is_set():
                return
            robot.servo.move_time_write(i, 400, 1000)
            if i == 1 or i == 3:
                robot.servo.move_time_write(i, 410, 1000)
            if i == 9 or i == 11:
                robot.servo.move_time_write(i, 370, 1000)    
        time.sleep(2)
        print_terminal("Reset complete")
        
        # 记录完成事件
        robot.data_storage.log_event("MOVEMENT", "复位操作完成")

# 策略工厂
class StrategyFactory:
    @staticmethod
    def get_strategy(mode, direction):
        if direction == "复位" or mode == "复位模式":
            return FuweiStrategy()
        
        if mode == "蠕动模式":
            if direction == "前进":
                return RudongStrategy()
            elif direction == "后退":
                return HoutuiStrategy()
            elif direction == "左转":
                return ZuozhuanStrategy()
            elif direction == "右转":
                return YouzhuanStrategy()
        
        elif mode == "蜿蜒模式":
            if direction == "前进":
                return WanyanStrategy2()
            elif direction == "后退":
                return HoutuiStrategy()
            elif direction == "左转":
                return fangunL()
            elif direction == "右转":
                return fangunR()
        
        # 默认返回复位策略
        return FuweiStrategy()

# 摄像头适配器模式
class CameraInterface(ABC):
    @abstractmethod
    def initialize(self):
        pass
    
    @abstractmethod
    def capture_frame(self):
        pass
    
    @abstractmethod
    def release(self):
        pass

class OpenCVCamera(CameraInterface):
    def __init__(self, resolution=CAMERA_RESOLUTION):
        self.resolution = resolution
        self.camera = None
        self.dummy_image = None
        self._create_dummy_image()
    
    def _create_dummy_image(self):
        """创建默认测试图像，当摄像头失败时使用"""
        width, height = self.resolution
        # 创建黑色图像
        img = np.zeros((height, width, 3), np.uint8)
        
        # 添加一些彩色矩形和文字
        cv2.rectangle(img, (0, 0), (width, height), (0, 0, 128), -1)
        cv2.rectangle(img, (20, 20), (width-20, height-20), (0, 0, 0), -1)
        
        # 添加文字
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, 'No Camera Available', (30, height//2-20), font, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f'User: {CURRENT_USER}', (30, height//2+20), font, 0.5, (200, 200, 200), 1)
        cv2.putText(img, 'Snake Robot Control', (30, height//2+50), font, 0.5, (200, 200, 200), 1)
        
        # 编码图像
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        _, buffer = cv2.imencode('.jpg', img, encode_param)
        self.dummy_image = base64.b64encode(buffer).decode('utf-8')
        print_terminal(f"Created dummy image: {len(self.dummy_image)/1024:.1f} KB")
    
    def initialize(self):
        """初始化摄像头，包含备选方案"""
        # 尝试不同的后端和API访问摄像头
        if TRY_ALTERNATE_BACKENDS:
            # 按照优先级排序的选项
            api_preferences = [
                cv2.CAP_V4L2,      # 先尝试V4L2
                cv2.CAP_V4L,       # 然后是普通V4L
                cv2.CAP_GSTREAMER, # 然后是GStreamer
                cv2.CAP_ANY        # 最后，任何可用的API
            ]
            
            for api in api_preferences:
                try:
                    print_terminal(f"Trying camera with API preference: {api}")
                    self.camera = cv2.VideoCapture(0, api)
                    
                    if self.camera is not None and self.camera.isOpened():
                        print_terminal(f"Successfully opened camera with API: {api}")
                        
                        # 配置基本摄像头设置
                        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
                        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
                        
                        # 测试是否可以读取帧
                        ret, frame = self.camera.read()
                        if ret:
                            print_terminal(f"Camera initialized: {frame.shape[1]}x{frame.shape[0]}")
                            return True
                        else:
                            print_terminal("Camera opened but can't read frames, trying next API")
                            self.camera.release()
                            self.camera = None
                    else:
                        print_terminal(f"Failed to open camera with API: {api}")
                except Exception as e:
                    print_terminal(f"Error with camera API {api}: {e}")
        
        # 如果上面的方法都失败了，尝试默认方法
        try:
            print_terminal("Trying default camera initialization")
            self.camera = cv2.VideoCapture(0)
            if self.camera.isOpened():
                # 测试是否可以读取帧
                ret, frame = self.camera.read()
                if ret:
                    print_terminal(f"Camera initialized using default approach: {frame.shape[1]}x{frame.shape[0]}")
                    return True
                else:
                    print_terminal("Default camera approach failed to read frames")
                    self.camera.release()
                    self.camera = None
            else:
                print_terminal("Failed to open camera with default approach")
        except Exception as e:
            print_terminal(f"Error with default camera initialization: {e}")
            
        # 如果所有方法都失败了，我们没有可用的摄像头
        print_terminal("No camera available - will use dummy image instead")
        return False
    
    def capture_frame(self):
        """捕获摄像头帧，如果摄像头不可用则返回默认图像"""
        # 如果没有可用的摄像头，返回默认图像
        if self.camera is None or not self.camera.isOpened():
            return self.dummy_image
            
        try:
            # 尝试从摄像头读取一帧
            ret, frame = self.camera.read()
            if not ret:
                # 如果读取失败，回退到默认图像
                return self.dummy_image
                
            # 如果需要，调整到目标分辨率
            if frame.shape[1] != self.resolution[0] or frame.shape[0] != self.resolution[1]:
                frame = cv2.resize(frame, self.resolution)
            
            # 简单的JPEG编码
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            _, buffer = cv2.imencode('.jpg', frame, encode_param)
            
            if buffer is None:
                # 如果编码失败，回退到默认图像
                return self.dummy_image
                
            # 转换为base64
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            return jpg_as_text
            
        except Exception as e:
            print_terminal(f"Error capturing camera frame: {e}")
            return self.dummy_image
    
    def release(self):
        """释放摄像头资源"""
        if self.camera is not None:
            self.camera.release()
            self.camera = None

# MQTT观察者模式 - 修复了连接和订阅逻辑
class MQTTObserver(metaclass=Singleton):
    def __init__(self, client_id=MQTT_CLIENT_ID, broker=MQTT_BROKER, port=MQTT_PORT, 
                username=MQTT_USERNAME, password=MQTT_PASSWORD):
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.mqtt_client = None
        self.is_connected = False
        self.observers = []
        self.connection_event = threading.Event()
        
    def connect(self):
        """连接到MQTT代理"""
        try:
            print_terminal(f"===== 尝试连接MQTT代理 =====")
            print_terminal(f"代理地址: {self.broker}:{self.port}")
            print_terminal(f"客户端ID: {self.client_id}")
            print_terminal(f"将订阅主题: {MQTT_TOPIC_SUBSCRIBE}")
            print_terminal(f"发布主题: {MQTT_TOPIC_PUBLISH}")
            
            self.mqtt_client = mqtt.Client(client_id=self.client_id)
            self.mqtt_client.username_pw_set(username=self.username, password=self.password)
            self.mqtt_client.on_connect = self._on_connect
            self.mqtt_client.on_message = self._on_message
            self.mqtt_client.on_disconnect = self._on_disconnect
            self.mqtt_client.on_subscribe = self._on_subscribe  # 添加订阅回调
            
            self.mqtt_client.connect(self.broker, self.port, 60)
            self.mqtt_client.loop_start()
            print_terminal(f"MQTT连接已启动，等待连接确认...")
            
            # 等待连接建立
            if self.connection_event.wait(timeout=5.0):
                print_terminal("MQTT连接已确认建立")
                return True
            else:
                print_terminal("MQTT连接超时")
                return False
                
        except Exception as e:
            print_terminal(f"连接MQTT代理失败: {e}")
            return False
    
    def disconnect(self):
        """断开与MQTT代理的连接"""
        if self.mqtt_client:
            try:
                self.mqtt_client.disconnect()
                self.mqtt_client.loop_stop()
                print_terminal("已断开与MQTT代理的连接")
            except Exception as e:
                print_terminal(f"断开MQTT连接时出错: {e}")
    
    def publish(self, topic, message):
        """发布消息到主题"""
        if not self.mqtt_client or not self.is_connected:
            print_terminal("无法发布 - 未连接到MQTT代理")
            return False
        
        try:
            result = self.mqtt_client.publish(topic, message)
            if result.rc != 0:
                print_terminal(f"发布消息错误: {result.rc}")
                return False
            return True
        except Exception as e:
            print_terminal(f"发布到MQTT时出错: {e}")
            return False
    
    def subscribe(self, topic):
        """订阅主题"""
        if not self.mqtt_client or not self.is_connected:
            print_terminal("无法订阅 - 未连接到MQTT代理")
            return False
        
        try:
            result = self.mqtt_client.subscribe(topic)
            if result[0] != 0:
                print_terminal(f"订阅主题 {topic} 时出错: {result[0]}")
                return False
            print_terminal(f"已发送订阅请求: {topic}")
            return True
        except Exception as e:
            print_terminal(f"订阅主题 {topic} 时出错: {e}")
            return False
    
    def register_observer(self, observer):
        """注册消息回调的观察者"""
        self.observers.append(observer)
        print_terminal(f"已注册观察者: {observer.__class__.__name__}")
    
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            print_terminal("===== MQTT连接成功 =====")
            self.is_connected = True
            self.connection_event.set()  # 设置连接事件
            
            # 连接成功后订阅主题
            print_terminal(f"正在订阅主题: {MQTT_TOPIC_SUBSCRIBE}")
            client.subscribe(MQTT_TOPIC_SUBSCRIBE)
        else:
            print_terminal(f"MQTT连接失败，返回码: {rc}")
            self.is_connected = False
    
    def _on_subscribe(self, client, userdata, mid, granted_qos):
        """MQTT订阅回调"""
        print_terminal(f"===== 订阅确认 =====")
        print_terminal(f"订阅主题成功，消息ID: {mid}, QoS: {granted_qos}")
    
    def _on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        print_terminal(f"与MQTT代理断开连接，返回码: {rc}")
        self.is_connected = False
        self.connection_event.clear()  # 清除连接事件
        
        # 尝试重新连接
        try:
            print_terminal("尝试重新连接MQTT...")
            client.reconnect()
        except Exception as e:
            print_terminal(f"重新连接失败: {e}")
    
    def _on_message(self, client, userdata, msg):
        """MQTT接收消息回调"""
        try:
            payload = msg.payload.decode()
            print_terminal(f"===== 收到MQTT消息 =====")
            print_terminal(f"主题: {msg.topic}")
            print_terminal(f"内容: {payload}")
            
            # 通知所有观察者
            for observer in self.observers:
                observer.on_message(msg.topic, payload)
        except Exception as e:
            print_terminal(f"处理MQTT消息时出错: {e}")

# 串口查找工具
class SerialPortFinder:
    @staticmethod
    def find_available_serial_port(port_list=SERIAL_PORTS):
        """从可能的端口列表中找到可用的串口"""
        for port in port_list:
            if os.path.exists(port):
                print_terminal(f"Found serial port: {port}")
                try:
                    # 尝试打开端口以验证其可访问性
                    s = serial.Serial(port, 115200, timeout=0.1)
                    s.close()
                    return port
                except Exception as e:
                    print_terminal(f"Port {port} exists but couldn't be opened: {e}")
        
        print_terminal("No suitable serial port found!")
        return None

# 增强ServoDriver以使用可用端口
def patch_servo_driver():
    """修补ServoDriver类以使用可用端口"""
    try:
        original_init = ServoDriver.__init__

        def patched_init(self):
            # 初始化GPIO
            from servo_driver import gpio_init
            gpio_init()
            
            # 初始化延迟字典
            self.delay = {3: 0.00042, 5: 0.00055, 7: 0.0008}
            
            # 查找可用端口
            port_path = SerialPortFinder.find_available_serial_port()
            if not port_path:
                print_terminal("No serial port available for servo control!")
                raise IOError("No suitable serial port found")
            
            # 打开串口
            self.port = serial.Serial(port_path, 115200, timeout=0.2)
            print_terminal(f"ServoDriver initialized using port: {port_path}")

        # 应用补丁
        ServoDriver.__init__ = patched_init
        print_terminal("Successfully patched ServoDriver")
        return True
    except Exception as e:
        print_terminal(f"Failed to patch ServoDriver: {e}")
        return False

# 主处理类，实现所有组件
class SnakeRobot:
    def __init__(self):
        # 首先应用ServoDriver补丁
        patch_servo_driver()
        
        # 初始化数据存储
        self.data_storage = DataStorageManager()
        
        # 初始化MQTT通信
        self.mqtt = MQTTObserver()
        self.mqtt.register_observer(self)
        
        # 初始化舵机驱动
        try:
            self.servo = ServoDriver()
            self.servo_available = True
            self.enable_servos(1)
            self.servo_list = []  # 舵机ID列表
            print_terminal("Servo driver initialized successfully")
            
            # 记录事件
            self.data_storage.log_event("INIT", "舵机驱动初始化成功")
        except Exception as e:
            print_terminal(f"Failed to initialize servo driver: {e}")
            print_terminal("Running in simulation mode - no servos will be controlled")
            self.servo_available = False
            
            # 记录事件
            self.data_storage.log_event("ERROR", f"舵机驱动初始化失败: {e}")
        
        # 控制标志
        self.current_mode = "休眠模式"  # 当前模式
        self.current_direction = None  # 当前方向
        self.running_flag = threading.Event()
        self.running_flag.set()  # 开始于运行状态
        
        # 初始化传感器
        self._init_sensors()
        
        # 初始化摄像头
        self.camera = OpenCVCamera()
        self.camera.initialize()
        
        # 统计信息
        self.frame_count = 0
        self.successful_frames = 0
        self.failed_frames = 0
        
        # 移动线程
        self.movement_thread = None
        
        # 数据批量发送线程
        self.batch_sender_thread = None
    
    def _init_sensors(self):
        """使用工厂初始化温度、压力和空气质量传感器"""
        self.pressure_sensor = SensorFactory.create_pressure_sensor()
        self.air_quality_sensor = SensorFactory.create_air_quality_sensor()
        
        # 记录传感器状态
        self.data_storage.log_event("INIT", f"压力传感器可用: {self.pressure_sensor is not None}")
        self.data_storage.log_event("INIT", f"空气质量传感器可用: {self.air_quality_sensor is not None}")
    
    def enable_servos(self, value=1):
        """打开舵机"""
        if not self.servo_available:
            return
            
        for i in range(SNAKE_LENGTH):
            self.servo.load_or_unload_write(i, value)
    
    def disable_servos(self, value=0):
        """关闭舵机"""
        if not self.servo_available:
            return
            
        for i in range(SNAKE_LENGTH):
            self.servo.load_or_unload_write(i, value)
    
    def list_servos(self):
        """检查可用舵机并返回舵机ID列表"""
        if not self.servo_available:
            print_terminal("Servo driver not available")
            return
            
        for i in range(SNAKE_LENGTH):
            n = self.servo.id_read(i)       
            if n is not None:
                self.servo_list.append(n)
        print_terminal(f"Found servos: {self.servo_list}")
        
        # 记录找到的舵机
        self.data_storage.log_event("INFO", f"检测到舵机: {self.servo_list}")
    
    def reset_to_neutral(self):
        """重置所有舵机到中立位置"""
        if not self.servo_available:
            return
            
        for i in range(0, SNAKE_LENGTH, 1):
            self.servo.move_time_write(i, 400, 1000)
            if i == 1 or i == 3:
                self.servo.move_time_write(i, 410, 1000)
            if i == 9 or i == 11:
                self.servo.move_time_write(i, 370, 1000)    
        time.sleep(0.02)
    
    def on_message(self, topic, payload):
        """处理传入的MQTT消息"""
        try:
            # 解析消息
            data = json.loads(payload)
            print_terminal(f"===== 收到控制命令 =====")
            print_terminal(f"命令内容: {payload}")
            
            # 记录收到的命令
            self.data_storage.log_event("COMMAND", f"收到命令: {payload}")
            
            # 根据模式和方向处理命令
            mode = data.get("mode")
            direction = data.get("direction")
            
            # 如果缺少必要参数则跳过
            if not mode and not direction:
                print_terminal("收到不完整命令，缺少模式或方向")
                self.data_storage.log_event("ERROR", "不完整命令，缺少模式或方向")
                return
                
            # 如果提供了，则更新当前模式和方向
            if mode:
                self.current_mode = mode
                print_terminal(f"模式设置为: {mode}")
                
            if direction:
                self.current_direction = direction
                print_terminal(f"方向设置为: {direction}")
            
            # 停止任何现有的移动线程
            self.stop_movement()
            
            # 基于模式和方向开始新的移动
            if self.servo_available:
                self.start_movement(mode, direction)
            else:
                print_terminal(f"收到命令: mode={mode}, direction={direction} (模拟模式 - 无舵机控制)")
                self.data_storage.log_event("INFO", f"模拟模式下的命令: {mode} - {direction}")
                
        except Exception as e:
            print_terminal(f"处理命令时出错: {e}")
            self.data_storage.log_event("ERROR", f"处理命令时出错: {e}")
    
    def stop_movement(self):
        """停止任何运行中的移动线程"""
        # 发信号给线程停止
        self.running_flag.clear()
        
        # 等待线程完成
        if self.movement_thread and self.movement_thread.is_alive():
            print_terminal("等待移动停止...")
            self.movement_thread.join(timeout=1.0)
        
        # 重置标志以便新的移动
        self.running_flag.set()
    
    def start_movement(self, mode, direction):
        """基于模式和方向开始移动"""
        # 获取适当的策略
        strategy = StrategyFactory.get_strategy(mode, direction)
        
        # 在新线程中开始移动
        self.movement_thread = threading.Thread(
            target=self._execute_movement,
            args=(strategy,)
        )
        self.movement_thread.daemon = True
        self.movement_thread.start()
    
    def _execute_movement(self, strategy):
        """执行移动策略"""
        try:
            strategy.execute(self, self.running_flag)
        except Exception as e:
            print_terminal(f"执行移动时出错: {e}")
            self.data_storage.log_event("ERROR", f"执行移动时出错: {e}")
    
    def get_sensor_data(self):
        """从所有可用的传感器获取数据"""
        data = {
            "timestamp": time.time(),
            "current_gait": self.current_mode  # 使用current_mode匹配swjmain6.py
        }
        
        # 从SPL06传感器获取温度和压力
        if self.pressure_sensor:
            try:
                # 从SPL06传感器获取实际读数
                temperature = self.pressure_sensor.get_temperature()
                pressure_pa = self.pressure_sensor.get_pressure()
                pressure_hpa = pressure_pa / 100.0  # 将Pa转换为hPa
                
                # 添加到数据字典
                data["temperature"] = round(float(temperature), 1)
                data["pressure"] = round(float(pressure_hpa), 1)
                
                # 计算估计高度（附加信息）
                P0_hpa = 1013.25  # 标准海平面压力（hPa）
                altitude = 44330.0 * (1.0 - pow(pressure_hpa / P0_hpa, 1/5.255))
                data["altitude"] = round(float(altitude), 1)
            except Exception as e:
                print_terminal(f"读取压力/温度传感器错误: {e}")
                self.data_storage.log_event("ERROR", f"读取压力/温度传感器错误: {e}")
                data["temperature"] = DEFAULT_TEMPERATURE
                data["pressure"] = DEFAULT_PRESSURE
                data["altitude"] = 0.0
        else:
            # 如果没有传感器则使用默认值
            data["temperature"] = DEFAULT_TEMPERATURE
            data["pressure"] = DEFAULT_PRESSURE
            data["altitude"] = 0.0
        
        # 从SGP40传感器获取空气质量
        if self.air_quality_sensor:
            try:
                # 获取VOC指数（0-500范围）
                voc_index = self.air_quality_sensor.get_voc_index()
                
                # 如果VOC指数计算不可用，使用原始值
                if voc_index < 0:
                    # 获取温度用于补偿
                    temperature = data.get("temperature", 25.0)
                    humidity = 50.0  # 默认湿度，如果不可用
                    
                    # 获取传感器原始值并映射到VOC指数
                    raw_value = self.air_quality_sensor.measure_raw(temperature, humidity)
                    voc_index = min(500, max(0, int(raw_value / 100)))
                
                data["air_quality"] = int(voc_index)
            except Exception as e:
                print_terminal(f"读取空气质量传感器错误: {e}")
                self.data_storage.log_event("ERROR", f"读取空气质量传感器错误: {e}")
                data["air_quality"] = DEFAULT_AIR_QUALITY
        else:
            data["air_quality"] = DEFAULT_AIR_QUALITY
        
        # 添加视频统计信息用于调试
        if ENABLE_DEBUG:
            data["camera_stats"] = {
                "frames_captured": self.frame_count,
                "successful": self.successful_frames,
                "failed": self.failed_frames,
                "camera_available": True if self.camera else False
            }
        
        return data
    
    def publish_data(self):
        """发布传感器数据和摄像头帧到MQTT，同时保存到本地"""
        try:
            # 获取传感器数据
            data = self.get_sensor_data()
            
            # 如果可用，添加摄像头帧
            self.frame_count += 1
            camera_frame = self.camera.capture_frame()
            if camera_frame:
                data["camera_frame"] = camera_frame
                self.successful_frames += 1
            else:
                self.failed_frames += 1
            
            # 存储数据到本地
            self.data_storage.store_sensor_data(data)
            
            # 将数据转换为JSON可序列化格式
            data_for_mqtt = convert_to_serializable(data)
                
            # 转换为JSON并发布
            data_json = json.dumps(data_for_mqtt)
            self.mqtt.publish(MQTT_TOPIC_PUBLISH, data_json)
            
            # 定期记录统计信息
            if ENABLE_DEBUG and self.frame_count % 30 == 0:
                success_rate = (self.successful_frames / self.frame_count) * 100 if self.frame_count > 0 else 0
                print_terminal(f"Camera stats: {self.successful_frames}/{self.frame_count} frames ({success_rate:.1f}% success)")
                print_terminal(f"Camera frame size: {len(data_json)/1024:.1f} KB")
        except Exception as e:
            print_terminal(f"发布数据错误: {e}")
            self.data_storage.log_event("ERROR", f"发布数据错误: {e}")
            if ENABLE_DEBUG:
                import traceback
                traceback.print_exc()
    
    def start_batch_sender(self):
        """启动批量数据发送线程"""
        self.batch_sender_thread = threading.Thread(target=self.batch_sender_loop, daemon=True)
        self.batch_sender_thread.start()
        print_terminal("批量数据发送线程已启动")
    
    def batch_sender_loop(self):
        """定时发送批量数据的线程循环"""
        while not self.shutdown_event.is_set():
            try:
                # 获取待发送文件列表
                pending_files = self.data_storage.get_pending_files()
                
                if pending_files:
                    print_terminal(f"正在准备发送 {len(pending_files)} 个文件...")
                    
                    # 准备批量数据
                    batch_data = self.data_storage.prepare_batch_data(pending_files)
                    
                    # 转换为JSON并发布
                    batch_json = json.dumps(batch_data)
                    
                    # 发送到批量数据主题
                    if self.mqtt.publish(MQTT_TOPIC_DATA_BATCH, batch_json):
                        print_terminal(f"成功发送批量数据，包含 {len(pending_files)} 个文件")
                        
                        # 标记文件为已发送
                        self.data_storage.mark_files_as_sent(pending_files)
                        
                        # 记录事件
                        self.data_storage.log_event("BATCH", f"成功发送批量数据: {len(pending_files)}个文件")
                    else:
                        print_terminal("批量数据发送失败")
                        self.data_storage.log_event("ERROR", "批量数据发送失败")
                
            except Exception as e:
                print_terminal(f"批量数据发送错误: {e}")
                self.data_storage.log_event("ERROR", f"批量数据发送错误: {e}")
            
            # 等待下一次发送周期
            time.sleep(DATA_SEND_INTERVAL)
    
    def data_publishing_loop(self):
        """连续数据发布的线程函数"""
        while not self.shutdown_event.is_set():
            self.publish_data()
            time.sleep(0.1)  # 10Hz数据速率
    
    def signal_handler(self, sig, frame):
        """处理终止信号"""
        print_terminal("收到关闭信号，清理中...")
        self.shutdown()
        sys.exit(0)
    
    def shutdown(self):
        """关闭前清理资源"""
        # 通知线程停止
        self.shutdown_event.set()
        self.stop_movement()
        
        # 保存最后的日志
        self.data_storage.log_event("SYSTEM", "系统关闭")
        
        # 如果舵机可用，重置到home位置
        if self.servo_available:
            try:
                # 直接执行策略以立即执行
                FuweiStrategy().execute(self, threading.Event())
                self.disable_servos()
            except Exception as e:
                print_terminal(f"舵机清理过程中错误: {e}")
                self.data_storage.log_event("ERROR", f"舵机清理错误: {e}")
        
        # 释放摄像头
        if self.camera:
            self.camera.release()
        
        # 断开MQTT
        self.mqtt.disconnect()
        
        # 关闭空气质量传感器加热器
        if self.air_quality_sensor:
            try:
                self.air_quality_sensor.heater_off()
            except Exception as e:
                print_terminal(f"关闭空气质量传感器时错误: {e}")
        
        print_terminal("清理完成")
    
    def run(self):
        """主函数，包含数据发布线程和信号处理"""
        # 设置关闭事件
        self.shutdown_event = threading.Event()
        
        # 设置信号处理以便干净地关闭
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 记录系统启动
        self.data_storage.log_event("SYSTEM", "系统启动")
        self.data_storage.log_event("CONFIG", f"MQTT服务器: {MQTT_BROKER}:{MQTT_PORT}")
        self.data_storage.log_event("CONFIG", f"客户端ID: {MQTT_CLIENT_ID}")
        self.data_storage.log_event("CONFIG", f"数据存储目录: {DATA_STORAGE_DIR}")
        
        # 连接到MQTT
        print_terminal("\n===== 启动蛇形机器人控制系统 =====")
        print_terminal(f"日期/时间: {CURRENT_DATE}")
        print_terminal(f"用户: {CURRENT_USER}")
        print_terminal(f"MQTT配置: 服务器={MQTT_BROKER}, 客户端ID={MQTT_CLIENT_ID}")
        print_terminal(f"订阅主题: {MQTT_TOPIC_SUBSCRIBE}")
        print_terminal(f"发布主题: {MQTT_TOPIC_PUBLISH}")
        print_terminal(f"批量数据主题: {MQTT_TOPIC_DATA_BATCH}\n")
        
        if not self.mqtt.connect():
            print_terminal("连接MQTT失败，退出")
            self.data_storage.log_event("ERROR", "连接MQTT失败，系统退出")
            return
        
        # 移除直接订阅行，因为现在在连接成功回调中处理
        # 添加等待连接的延迟
        time.sleep(1)
        
        # 如果舵机可用，初始化到home位置
        if self.servo_available:
            try:
                FuweiStrategy().execute(self, threading.Event())
                self.data_storage.log_event("INIT", "舵机已初始化到初始位置")
            except Exception as e:
                print_terminal(f"初始化过程中错误: {e}")
                self.data_storage.log_event("ERROR", f"舵机初始化错误: {e}")
        else:
            print_terminal("舵机控制不可用 - 运行模拟模式")
            self.data_storage.log_event("INFO", "舵机控制不可用 - 运行模拟模式")
        
        # 启动数据发布线程
        data_thread = threading.Thread(target=self.data_publishing_loop)
        data_thread.daemon = True
        data_thread.start()
        
        # 启动批量数据发送线程
        self.start_batch_sender()
        
        # 主循环
        print_terminal("机器人控制系统运行中。按Ctrl+C退出。")
        try:
            while not self.shutdown_event.is_set():
                time.sleep(1)
                
                # 每分钟记录一次状态信息
                if int(time.time()) % 60 == 0:
                    self.data_storage.log_event("STATUS", 
                        f"当前模式: {self.current_mode}, 方向: {self.current_direction or '无'}, "
                        f"帧统计: {self.successful_frames}/{self.frame_count}"
                    )
                    time.sleep(1)  # 避免在同一秒内多次记录
                    
        except KeyboardInterrupt:
            print_terminal("主循环被中断")
            self.data_storage.log_event("SYSTEM", "系统被用户中断")
        finally:
            self.shutdown()

# 向后兼容的Process类
class Process(SnakeRobot):
    """兼容原始Process接口的类"""
    def __init__(self):
        super().__init__()
        # 额外的向后兼容性
        self.t = 0
        self.running = True
    
    # 使用委托到新架构实现旧方法
    def rudong(self):
        strategy = RudongStrategy()
        strategy.execute(self, self.running_flag)
    
    def houtui(self):
        strategy = HoutuiStrategy()
        strategy.execute(self, self.running_flag)
    
    def zuozhuan(self):
        strategy = ZuozhuanStrategy()
        strategy.execute(self, self.running_flag)
    
    def youzhuan(self):
        strategy = YouzhuanStrategy()
        strategy.execute(self, self.running_flag)
    
    def wanyan(self):
        strategy = WanyanStrategy()
        strategy.execute(self, self.running_flag)
    
    def fuwei(self):
        strategy = FuweiStrategy()
        strategy.execute(self, self.running_flag)
    
    # 兼容性别名
    def home(self):
        self.fuwei()
    
    # 入口点保持不变以保持向后兼容性
    def main(self):
        self.run()

# 主入口点
if __name__ == "__main__":
    # 更新当前日期/时间
    CURRENT_DATE = "2025-05-17 03:39:47"
    CURRENT_USER = "12ljf"
    
    # 启动机器人控制
    demo = Process()
    demo.main()