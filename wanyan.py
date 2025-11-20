
from servo_driver import ServoDriver
import time
import math
import numpy as np
import threading
import scipy.optimize as optimize
# from driver_imu import Imu
        
SNAKE_LENGTH = 12

class Process(object):
    def __init__(self):
        self.servo = ServoDriver()   ###创建类的对象
        self.enable(1)
        self.servo_list = []   ##舵机id列表
        # self.demo1 = Imu()#111

        self.t=0    #全局变量
        self.k=0      #转弯成都系数    左加右减      6：左转90°   10:左转掉头    -6：右转90°   -11:右转掉头  
        self.d=4    #蜿蜒直行速度挡位    3：慢档     4：中档    5：快档
        self.y=0
        self.N=400   ##直径85cm左右
        self.M=0
        self.functions = (self.wanyan_zhixing
             )##复位0 蠕动1 蠕动2 蜿蜒直行3 翻滚左4  翻滚右5 侧滑左边6 侧滑右7

    def list_servo(self):       ###检测舵机是否启动，返回舵机id列表
        for i in range(SNAKE_LENGTH):
            n = self.servo.id_read(i)       
            if n is not None:
                self.servo_list.append(n)
            print(self.servo_list)

    def offset_adjust(self):  ##读取舵机位置
        for i in range(SNAKE_LENGTH):
            a, b = self.servo.pos_read(i)
            print(i, a, b)
            self.servo.angle_offset_adjust(i, a-400)
            self.servo.angle_offset_write(i)

    def enable(self, value=1):   ###舵机上电
        for i in range(SNAKE_LENGTH):
            self.servo.load_or_unload_write(i, value)
    def disable(self, value=0):  #
        for i in range(SNAKE_LENGTH):  # 遍历12个舵机
            self.servo.load_or_unload_write(i, value)  # 0 掉电，1装电，恒为0
    def read(self):     ##读取舵机id
        for i in range(SNAKE_LENGTH):
            a, b = self.servo.pos_read(i)
            print(i, a, b)
    def target_func(self, x, a0, a1, a2, a3):
        return a0 * np.sin(a1 * x + a2) + a3


    def fuwei(self):       ##复位
        for i in range(0, 12, 1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000)    
        time.sleep(2)
        print("复位")

    ######  蜿蜒
    def wanyan_zhixing(self):  
        while True:
            self.servo.move_time_write(0,430,2)
            for i in range(1,12,2):    ##1，3，5，7，9，11
               # s=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2.0*self.d)+math.pi/4))+410+i*1.2
                s=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2.0*self.d)+math.pi/4))+410+(i-1)*self.k
                #print(12-i,int(s),self.t)
                print('蜿蜒直行')
                self.servo.move_time_write(12-i,int(s),4)
                time.sleep(0.002)
                self.t = self.t + 1
    
    def main(self):
        while True:
            user_input = input("请输入数字0到7(蜿蜒)（输入'q'退出）：")
            if user_input.lower() == 'q':
                print("退出程序。")
                break
            if user_input.isdigit() and 0 <= int(user_input) <= 7:
                try:
                    self.functions[int(user_input)]()  # 调用函数
                except IndexError:
                    print("输入数字超出范围，请重新输入。")
            else:
                print("输入无效，请重新输入一个0到7之间的数字。")

if __name__ == "__main__":
    demo=Process()
    demo.main()

