#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BIRobot 机器蛇智能控制系统 v2.0
Production-Grade Application

作者: 12ljf
创建日期: 2025-06-22
最后更新: 2025-06-22

功能说明:
    - MQTT通信控制
    - 实时视频流处理
    - YOLO目标检测
    - 传感器数据监控（温度、气压、空气质量）
    - 运动模式控制
    - 数据记录与导出

技术栈:
    - PySide6: GUI框架
    - OpenCV: 视频处理
    - YOLO (ultralytics): 目标检测
    - paho-mqtt: MQTT通信
    - pandas: 数据处理

使用方法:
    python main_production.py [options]
    
    选项:
        --debug         启用调试模式
        --config FILE   指定配置文件路径
        --log-level LEVEL  设置日志级别 (DEBUG/INFO/WARNING/ERROR)
"""

import os
import sys
import time
import json
import math
import argparse
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field
from queue import Queue, Empty
from concurrent.futures import ThreadPoolExecutor
import threading

# 修复高分辨率屏幕 DPI 缩放问题
os.environ["QT_FONT_DPI"] = "96"

# 第三方库导入
try:
    import cv2
    import numpy as np
    import pandas as pd
    import paho.mqtt.client as mqtt
    from PySide6.QtWidgets import *
    from PySide6.QtCore import *
    from PySide6.QtGui import *
    from PySide6.QtCharts import *
except ImportError as e:
    print(f"错误: 缺少必要的依赖库 - {e}")
    print("请运行: pip install PySide6 opencv-python numpy pandas paho-mqtt")
    sys.exit(1)

# YOLO 可选依赖
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    YOLO = None


# ============================================================================
# 日志配置
# ============================================================================

class ColoredFormatter(logging.Formatter):
    """彩色日志格式化器"""
    
    COLORS = {
        'DEBUG': '\033[36m',     # Cyan
        'INFO': '\033[32m',      # Green
        'WARNING': '\033[33m',   # Yellow
        'ERROR': '\033[31m',     # Red
        'CRITICAL': '\033[35m',  # Magenta
        'RESET': '\033[0m'       # Reset
    }
    
    def format(self, record):
        log_color = self.COLORS.get(record.levelname, self.COLORS['RESET'])
        record.levelname = f"{log_color}{record.levelname}{self.COLORS['RESET']}"
        return super().format(record)


def setup_logging(log_level: str = "INFO", log_file: Optional[str] = None) -> logging.Logger:
    """
    设置日志系统
    
    Args:
        log_level: 日志级别 (DEBUG/INFO/WARNING/ERROR/CRITICAL)
        log_file: 日志文件路径，None表示不写入文件
        
    Returns:
        配置好的Logger实例
    """
    logger = logging.getLogger("BIRobot")
    logger.setLevel(getattr(logging, log_level.upper(), logging.INFO))
    
    # 清除现有处理器
    logger.handlers.clear()
    
    # 控制台处理器
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    console_formatter = ColoredFormatter(
        '%(asctime)s - %(name)s - [%(levelname)s] - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    # 文件处理器（如果指定）
    if log_file:
        try:
            file_handler = logging.FileHandler(log_file, encoding='utf-8')
            file_handler.setLevel(logging.DEBUG)
            file_formatter = logging.Formatter(
                '%(asctime)s - %(name)s - [%(levelname)s] - %(filename)s:%(lineno)d - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
            file_handler.setFormatter(file_formatter)
            logger.addHandler(file_handler)
        except Exception as e:
            logger.warning(f"无法创建日志文件 {log_file}: {e}")
    
    return logger


# 全局日志记录器
logger = setup_logging()


# ============================================================================
# 配置管理
# ============================================================================

@dataclass
class ApplicationConfig:
    """应用程序配置类"""
    name: str = "BIRobot"
    version: str = "2.0.0"
    debug_mode: bool = False
    log_level: str = "INFO"


@dataclass
class MQTTConfig:
    """MQTT配置类"""
    broker: str = "47.107.36.182"
    port: int = 1883
    client_id: str = "USER001"
    username: str = "public"
    password: str = "UQU92K77cpxc2Tm"
    topic_publish: str = "USER001"
    topic_subscribe: str = "USER002"
    keepalive: int = 60
    reconnect_delay: int = 5
    max_reconnect_attempts: int = 10


@dataclass
class VideoConfig:
    """视频配置类"""
    default_width: int = 1280
    default_height: int = 720
    fps_limit: int = 30
    skip_frames: int = 2
    recording_fps: int = 20


@dataclass
class YOLOConfig:
    """YOLO配置类"""
    model_path: str = "yolov8n.pt"
    confidence_threshold: float = 0.7
    enable_debug: bool = False


@dataclass
class SensorConfig:
    """传感器配置类"""
    max_data_points: int = 50
    update_interval: int = 1000  # 毫秒


@dataclass
class UIConfig:
    """UI配置类"""
    window_title: str = "BIRobot 机器蛇智能控制系统"
    default_width: int = 1920
    default_height: int = 1080
    theme: str = "dark"


@dataclass
class AirQualityLevel:
    """空气质量等级"""
    name: str
    min: int
    max: int
    color: str


@dataclass
class GaitMode:
    """步态模式"""
    name: str
    color: str


@dataclass
class Config:
    """主配置类"""
    application: ApplicationConfig = field(default_factory=ApplicationConfig)
    mqtt: MQTTConfig = field(default_factory=MQTTConfig)
    video: VideoConfig = field(default_factory=VideoConfig)
    yolo: YOLOConfig = field(default_factory=YOLOConfig)
    sensors: SensorConfig = field(default_factory=SensorConfig)
    ui: UIConfig = field(default_factory=UIConfig)
    air_quality_levels: List[AirQualityLevel] = field(default_factory=list)
    gait_modes: List[GaitMode] = field(default_factory=list)
    
    @classmethod
    def load_from_file(cls, file_path: str) -> 'Config':
        """
        从JSON文件加载配置
        
        Args:
            file_path: 配置文件路径
            
        Returns:
            Config实例
            
        Raises:
            FileNotFoundError: 配置文件不存在
            json.JSONDecodeError: 配置文件格式错误
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            config = cls()
            
            # 加载各配置部分
            if 'application' in data:
                config.application = ApplicationConfig(**data['application'])
            
            if 'mqtt' in data:
                config.mqtt = MQTTConfig(**data['mqtt'])
            
            if 'video' in data:
                config.video = VideoConfig(**data['video'])
            
            if 'yolo' in data:
                config.yolo = YOLOConfig(**data['yolo'])
            
            if 'sensors' in data:
                config.sensors = SensorConfig(**data['sensors'])
            
            if 'ui' in data:
                config.ui = UIConfig(**data['ui'])
            
            # 加载空气质量等级
            if 'air_quality' in data and 'levels' in data['air_quality']:
                config.air_quality_levels = [
                    AirQualityLevel(**level) for level in data['air_quality']['levels']
                ]
            
            # 加载步态模式
            if 'gait_modes' in data:
                config.gait_modes = [
                    GaitMode(**mode) for mode in data['gait_modes']
                ]
            
            logger.info(f"配置已从文件加载: {file_path}")
            return config
            
        except FileNotFoundError:
            logger.warning(f"配置文件不存在: {file_path}，使用默认配置")
            return cls._create_default()
        except json.JSONDecodeError as e:
            logger.error(f"配置文件格式错误: {e}，使用默认配置")
            return cls._create_default()
        except Exception as e:
            logger.error(f"加载配置时发生错误: {e}，使用默认配置")
            return cls._create_default()
    
    @classmethod
    def _create_default(cls) -> 'Config':
        """创建默认配置"""
        config = cls()
        
        # 默认空气质量等级
        config.air_quality_levels = [
            AirQualityLevel("优秀", 0, 50, "#00FF88"),
            AirQualityLevel("良好", 51, 100, "#00D4FF"),
            AirQualityLevel("一般", 101, 150, "#FFD700"),
            AirQualityLevel("较差", 151, 200, "#FF8C00"),
            AirQualityLevel("危险", 201, 500, "#FF6B6B")
        ]
        
        # 默认步态模式
        config.gait_modes = [
            GaitMode("蠕动模式", "#00FF88"),
            GaitMode("蜿蜒模式", "#FF8C00"),
            GaitMode("复位模式", "#FF6B6B")
        ]
        
        return config
    
    def save_to_file(self, file_path: str) -> bool:
        """
        保存配置到JSON文件
        
        Args:
            file_path: 保存路径
            
        Returns:
            是否成功保存
        """
        try:
            data = {
                'application': {
                    'name': self.application.name,
                    'version': self.application.version,
                    'debug_mode': self.application.debug_mode,
                    'log_level': self.application.log_level
                },
                'mqtt': {
                    'broker': self.mqtt.broker,
                    'port': self.mqtt.port,
                    'client_id': self.mqtt.client_id,
                    'username': self.mqtt.username,
                    'password': self.mqtt.password,
                    'topic_publish': self.mqtt.topic_publish,
                    'topic_subscribe': self.mqtt.topic_subscribe,
                    'keepalive': self.mqtt.keepalive,
                    'reconnect_delay': self.mqtt.reconnect_delay,
                    'max_reconnect_attempts': self.mqtt.max_reconnect_attempts
                },
                'video': {
                    'default_width': self.video.default_width,
                    'default_height': self.video.default_height,
                    'fps_limit': self.video.fps_limit,
                    'skip_frames': self.video.skip_frames,
                    'recording_fps': self.video.recording_fps
                },
                'yolo': {
                    'model_path': self.yolo.model_path,
                    'confidence_threshold': self.yolo.confidence_threshold,
                    'enable_debug': self.yolo.enable_debug
                },
                'sensors': {
                    'max_data_points': self.sensors.max_data_points,
                    'update_interval': self.sensors.update_interval
                },
                'ui': {
                    'window_title': self.ui.window_title,
                    'default_width': self.ui.default_width,
                    'default_height': self.ui.default_height,
                    'theme': self.ui.theme
                },
                'air_quality': {
                    'levels': [
                        {
                            'name': level.name,
                            'min': level.min,
                            'max': level.max,
                            'color': level.color
                        }
                        for level in self.air_quality_levels
                    ]
                },
                'gait_modes': [
                    {
                        'name': mode.name,
                        'color': mode.color
                    }
                    for mode in self.gait_modes
                ]
            }
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
            
            logger.info(f"配置已保存到文件: {file_path}")
            return True
            
        except Exception as e:
            logger.error(f"保存配置失败: {e}")
            return False


# ============================================================================
# 资源管理器
# ============================================================================

class ResourceManager:
    """
    资源管理器 - 单例模式
    负责检测和管理硬件资源（GPU/CPU、摄像头等）
    """
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        
        self._initialized = True
        self.resources_info = self._detect_resources()
        logger.info("资源管理器已初始化")
        logger.info(f"GPU可用: {self.resources_info['gpu_available']}")
        logger.info(f"CPU核心数: {self.resources_info['cpu_count']}")
    
    def _detect_resources(self) -> Dict[str, Any]:
        """
        检测系统可用资源
        
        Returns:
            资源信息字典
        """
        resources = {
            "gpu_available": False,
            "gpu_info": [],
            "cpu_count": os.cpu_count() or 4,
            "camera_api": self._detect_camera_api(),
            "memory_available": self._get_available_memory()
        }
        
        # 检测CUDA GPU
        try:
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                resources["gpu_available"] = True
                for i in range(cv2.cuda.getCudaEnabledDeviceCount()):
                    resources["gpu_info"].append({
                        "index": i,
                        "name": f"GPU-{i}"
                    })
                logger.info(f"检测到 {len(resources['gpu_info'])} 个CUDA GPU")
        except Exception as e:
            logger.debug(f"CUDA GPU检测失败: {e}")
            
            # 备选方案：检查环境变量
            if 'CUDA_VISIBLE_DEVICES' in os.environ:
                resources["gpu_available"] = True
                resources["gpu_info"].append({"index": 0, "name": "GPU (from env)"})
                logger.info("从环境变量检测到GPU")
        
        return resources
    
    def _detect_camera_api(self) -> int:
        """
        检测最佳摄像头API
        
        Returns:
            OpenCV摄像头API常量
        """
        # 根据平台选择最佳API
        if sys.platform == 'win32':
            return cv2.CAP_DSHOW
        elif sys.platform == 'linux':
            return cv2.CAP_V4L2
        elif sys.platform == 'darwin':
            return cv2.CAP_AVFOUNDATION
        else:
            return cv2.CAP_ANY
    
    def _get_available_memory(self) -> float:
        """
        获取可用内存（GB）
        
        Returns:
            可用内存大小（GB）
        """
        try:
            import psutil
            return psutil.virtual_memory().available / (1024**3)
        except ImportError:
            logger.warning("psutil未安装，无法获取内存信息")
            return 4.0  # 默认假设4GB
    
    def get_camera_api(self) -> int:
        """获取摄像头API"""
        return self.resources_info["camera_api"]
    
    def is_gpu_available(self) -> bool:
        """检查GPU是否可用"""
        return self.resources_info["gpu_available"]
    
    def get_cpu_count(self) -> int:
        """获取CPU核心数"""
        return self.resources_info["cpu_count"]
    
    def get_resources_summary(self) -> Dict[str, Any]:
        """
        获取资源概要
        
        Returns:
            资源概要字典
        """
        return {
            "gpu_available": self.resources_info["gpu_available"],
            "cpu_count": self.resources_info["cpu_count"],
            "video_processor": "GPU" if self.resources_info["gpu_available"] else "CPU",
            "inference_processor": "GPU" if self.resources_info["gpu_available"] else "CPU"
        }


# ============================================================================
# 视频处理器
# ============================================================================

class VideoProcessor:
    """视频处理器基类"""
    
    def process_frame(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """
        处理视频帧
        
        Args:
            frame: 输入帧
            
        Returns:
            处理后的帧，失败返回None
        """
        raise NotImplementedError
    
    def convert_to_qt(self, frame: np.ndarray) -> Optional[QImage]:
        """
        转换为Qt图像格式
        
        Args:
            frame: OpenCV帧
            
        Returns:
            Qt图像对象，失败返回None
        """
        raise NotImplementedError


class CPUVideoProcessor(VideoProcessor):
    """CPU视频处理器"""
    
    def process_frame(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """CPU处理图像帧"""
        if frame is None or frame.size == 0:
            return None
        
        try:
            # 如果图像过大，进行缩放以提高性能
            h, w = frame.shape[:2]
            if w > 1280:
                scale = 1280 / w
                frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
            
            return frame
        except Exception as e:
            logger.error(f"CPU视频处理错误: {e}")
            return frame
    
    def convert_to_qt(self, frame: np.ndarray) -> Optional[QImage]:
        """转换为Qt图像格式"""
        try:
            if frame is None or frame.size == 0:
                return None
            
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                height, width, channel = frame.shape
                # BGR to RGB
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                bytes_per_line = 3 * width
                qt_image = QImage(
                    rgb_frame.data,
                    width,
                    height,
                    bytes_per_line,
                    QImage.Format_RGB888
                )
                return qt_image
            
            return None
        except Exception as e:
            logger.error(f"图像转换错误: {e}")
            return None


class GPUVideoProcessor(VideoProcessor):
    """GPU视频处理器"""
    
    def __init__(self):
        self.gpu_available = False
        
        try:
            self.gpu_available = cv2.cuda.getCudaEnabledDeviceCount() > 0
            if self.gpu_available:
                # GPU预热
                test_frame = np.zeros((100, 100, 3), dtype=np.uint8)
                gpu_frame = cv2.cuda_GpuMat()
                gpu_frame.upload(test_frame)
                gpu_frame.download()
                logger.info("GPU视频处理器初始化成功")
        except Exception as e:
            logger.warning(f"GPU初始化失败，将使用CPU: {e}")
            self.gpu_available = False
    
    def process_frame(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """GPU处理图像帧"""
        if frame is None or not self.gpu_available:
            # 回退到CPU处理
            cpu_processor = CPUVideoProcessor()
            return cpu_processor.process_frame(frame)
        
        try:
            # 上传到GPU
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            
            # GPU处理
            h, w = frame.shape[:2]
            if w > 1280:
                scale = 1280 / w
                gpu_resized = cv2.cuda.resize(
                    gpu_frame,
                    (int(w * scale), int(h * scale))
                )
                return gpu_resized.download()
            
            return gpu_frame.download()
        except Exception as e:
            logger.error(f"GPU视频处理错误: {e}")
            # 回退到CPU处理
            cpu_processor = CPUVideoProcessor()
            return cpu_processor.process_frame(frame)
    
    def convert_to_qt(self, frame: np.ndarray) -> Optional[QImage]:
        """转换为Qt图像格式（与CPU相同）"""
        cpu_processor = CPUVideoProcessor()
        return cpu_processor.convert_to_qt(frame)


# ============================================================================
# YOLO检测器
# ============================================================================

class YOLODetector:
    """
    YOLO目标检测器
    封装YOLO模型的加载和推理
    """
    
    def __init__(self, config: YOLOConfig, use_gpu: bool = False):
        """
        初始化YOLO检测器
        
        Args:
            config: YOLO配置
            use_gpu: 是否使用GPU
            
        Raises:
            ImportError: YOLO库不可用
            RuntimeError: 模型加载失败
        """
        if not YOLO_AVAILABLE:
            raise ImportError("YOLO库未安装，请安装ultralytics包")
        
        self.config = config
        self.model = None
        self.last_detections = []
        
        # 加载模型
        self._load_model(use_gpu)
    
    def _load_model(self, use_gpu: bool):
        """
        加载YOLO模型
        
        Args:
            use_gpu: 是否使用GPU
            
        Raises:
            RuntimeError: 模型加载失败
        """
        try:
            device = 'cuda' if use_gpu else 'cpu'
            logger.info(f"正在加载YOLO模型: {self.config.model_path} (设备: {device})")
            
            self.model = YOLO(self.config.model_path)
            
            if self.model is None:
                raise RuntimeError("模型加载失败")
            
            logger.info("YOLO模型加载成功")
        except Exception as e:
            logger.error(f"YOLO模型加载失败: {e}")
            raise RuntimeError(f"无法加载YOLO模型: {e}")
    
    def detect(
        self,
        frame: np.ndarray,
        draw: bool = True
    ) -> Tuple[Optional[np.ndarray], List[Dict[str, Any]]]:
        """
        执行目标检测
        
        Args:
            frame: 输入帧
            draw: 是否在帧上绘制检测结果
            
        Returns:
            (处理后的帧, 检测结果列表)
        """
        if frame is None or self.model is None:
            return frame, []
        
        try:
            # 执行推理
            results = self.model.predict(
                frame,
                conf=self.config.confidence_threshold,
                verbose=False
            )
            
            # 解析结果
            detections = []
            if results and len(results) > 0:
                result = results[0]
                
                if hasattr(result, 'boxes') and len(result.boxes) > 0:
                    for box in result.boxes:
                        try:
                            # 提取检测信息
                            xyxy = box.xyxy[0].cpu().numpy()
                            x1, y1, x2, y2 = map(int, xyxy)
                            conf = float(box.conf[0].cpu().numpy())
                            cls_id = int(box.cls[0].cpu().numpy())
                            cls_name = result.names[cls_id]
                            
                            detections.append({
                                'class': cls_id,
                                'name': cls_name,
                                'confidence': conf,
                                'box': [x1, y1, x2, y2]
                            })
                        except Exception as e:
                            logger.warning(f"解析检测结果失败: {e}")
                            continue
            
            self.last_detections = detections
            
            if self.config.enable_debug and detections:
                logger.debug(f"检测到 {len(detections)} 个物体")
            
            # 绘制检测结果
            if draw and detections:
                output_frame = frame.copy()
                for det in detections:
                    try:
                        x1, y1, x2, y2 = det['box']
                        name = det['name']
                        conf = det['confidence']
                        
                        # 绘制边界框
                        cv2.rectangle(output_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # 绘制标签
                        label = f'{name} {conf:.2f}'
                        label_size = cv2.getTextSize(
                            label,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            2
                        )[0]
                        
                        cv2.rectangle(
                            output_frame,
                            (x1, y1 - 25),
                            (x1 + label_size[0] + 10, y1),
                            (0, 255, 0),
                            -1
                        )
                        
                        cv2.putText(
                            output_frame,
                            label,
                            (x1 + 5, y1 - 7),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 0),
                            2
                        )
                    except Exception as e:
                        logger.warning(f"绘制检测框失败: {e}")
                
                return output_frame, detections
            
            return frame, detections
            
        except Exception as e:
            logger.error(f"目标检测失败: {e}")
            return frame, []
    
    def get_last_detections(self) -> List[Dict[str, Any]]:
        """获取最后一次检测结果"""
        return self.last_detections


# ============================================================================
# MQTT通信线程
# ============================================================================

class MQTTThread(QThread):
    """
    MQTT通信线程
    负责MQTT连接、消息发送和接收
    """
    
    # 信号定义
    sensor_data_signal = Signal(dict)      # 传感器数据
    connection_signal = Signal(bool)       # 连接状态
    video_frame_signal = Signal(np.ndarray)  # 视频帧
    error_signal = Signal(str)             # 错误信息
    
    def __init__(self, config: MQTTConfig, parent=None):
        """
        初始化MQTT线程
        
        Args:
            config: MQTT配置
            parent: 父对象
        """
        super().__init__(parent)
        self.config = config
        self.client = None
        self.is_connected = False
        self.should_stop = False
        self.reconnect_count = 0
        
        logger.info("MQTT线程已创建")
    
    def run(self):
        """线程主循环"""
        try:
            self._setup_client()
            self._connect()
            
            # 保持连接
            while not self.should_stop:
                if not self.is_connected and self.reconnect_count < self.config.max_reconnect_attempts:
                    logger.info(f"尝试重新连接 MQTT ({self.reconnect_count + 1}/{self.config.max_reconnect_attempts})")
                    time.sleep(self.config.reconnect_delay)
                    self._connect()
                
                time.sleep(0.1)
            
        except Exception as e:
            logger.error(f"MQTT线程异常: {e}")
            self.error_signal.emit(f"MQTT通信错误: {e}")
        finally:
            self._disconnect()
            logger.info("MQTT线程已停止")
    
    def _setup_client(self):
        """设置MQTT客户端"""
        try:
            self.client = mqtt.Client(self.config.client_id)
            self.client.username_pw_set(self.config.username, self.config.password)
            
            # 设置回调
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            self.client.on_message = self._on_message
            
            logger.info("MQTT客户端已配置")
        except Exception as e:
            logger.error(f"MQTT客户端配置失败: {e}")
            raise
    
    def _connect(self):
        """连接到MQTT代理"""
        try:
            self.client.connect(
                self.config.broker,
                self.config.port,
                self.config.keepalive
            )
            self.client.loop_start()
            logger.info(f"正在连接到 MQTT 代理: {self.config.broker}:{self.config.port}")
        except Exception as e:
            logger.error(f"MQTT连接失败: {e}")
            self.reconnect_count += 1
            self.is_connected = False
            self.connection_signal.emit(False)
    
    def _disconnect(self):
        """断开MQTT连接"""
        if self.client:
            try:
                self.client.loop_stop()
                self.client.disconnect()
                logger.info("MQTT连接已断开")
            except Exception as e:
                logger.warning(f"MQTT断开连接时出错: {e}")
    
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            self.is_connected = True
            self.reconnect_count = 0
            self.connection_signal.emit(True)
            
            # 订阅主题
            self.client.subscribe(self.config.topic_subscribe)
            logger.info(f"MQTT已连接并订阅主题: {self.config.topic_subscribe}")
        else:
            self.is_connected = False
            self.connection_signal.emit(False)
            logger.error(f"MQTT连接失败，返回码: {rc}")
    
    def _on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        self.is_connected = False
        self.connection_signal.emit(False)
        
        if rc != 0:
            logger.warning(f"MQTT意外断开连接，返回码: {rc}")
        else:
            logger.info("MQTT正常断开连接")
    
    def _on_message(self, client, userdata, msg):
        """MQTT消息接收回调"""
        try:
            # 解析消息
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            # 根据消息类型处理
            if 'temperature' in data or 'pressure' in data:
                # 传感器数据
                self.sensor_data_signal.emit(data)
            elif 'frame' in data:
                # 视频帧数据（Base64编码）
                import base64
                frame_data = base64.b64decode(data['frame'])
                nparr = np.frombuffer(frame_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    self.video_frame_signal.emit(frame)
            
        except json.JSONDecodeError:
            logger.warning(f"无法解析MQTT消息: {msg.payload}")
        except Exception as e:
            logger.error(f"处理MQTT消息时出错: {e}")
    
    def publish_command(self, command: Dict[str, Any]) -> bool:
        """
        发布控制命令
        
        Args:
            command: 命令字典
            
        Returns:
            是否成功发布
        """
        if not self.is_connected:
            logger.warning("MQTT未连接，无法发布命令")
            return False
        
        try:
            payload = json.dumps(command)
            result = self.client.publish(self.config.topic_publish, payload)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logger.debug(f"命令已发布: {command}")
                return True
            else:
                logger.error(f"命令发布失败，返回码: {result.rc}")
                return False
        except Exception as e:
            logger.error(f"发布命令时出错: {e}")
            return False
    
    def stop(self):
        """停止线程"""
        self.should_stop = True
        logger.info("正在停止MQTT线程...")


# File continues with UI components...
# Due to length limitations, I'll create the UI components in the next section

