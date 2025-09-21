
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
        self.functions = (self.fuwei,self.rudong_u,self.rudong_X,self.wanyan_zhixing, self.fangun_L,self.fangun_R,self.cehua_L, self.cehua_R
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

######越障
    def yuezhang(self):
        for i in range(0, 12, 1):
            self.servo.move_time_write(i,400,1000)
            if i==1:
                self.servo.move_time_write(i,self.N,1000)
            # if i==2:
            #     self.servo.move_time_write(i,690,1000)
            if i==3:
                self.servo.move_time_write(i,self.N+50,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000)
        time.sleep(1.2)
        while True:
            for i in range(2, SNAKE_LENGTH, 2):
                # if i==2 and 150 * (math.sin( math.pi* self.t/25))>0:
                #     s = 150 * (math.sin( math.pi* self.t/25)) + 690
                #     self.servo.move_time_write(i, int(s), 4)
                #     print (i,int(s),self.t)
                #     time.sleep(0.004)
                if i==4:
                    s = 190 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                    self.servo.move_time_write(i, int(s), 4)
                    print (i,int(s),self.t)
                    time.sleep(0.004)
                if i==10:
                    s = 220 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400-80
                    self.servo.move_time_write(i, int(s), 4)
                    print (i,int(s),self.t)
                    time.sleep(0.004)
                if i==6 or i==8:
                    s = 200 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                    self.servo.move_time_write(i, int(s), 4)
                    print (i,int(s),self.t)
                    time.sleep(0.004)
            self.t=self.t+1 
        # for i in range(1,10,2):    ## 3，5，7，9，11
        #     s=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2*self.d)))+400+i*1.8
        #     print(12-i,int(s),self.t)
        #     self.servo.move_time_write(12-i,int(s),300)
        # time.sleep(0.3)
        # self.t=self.t+1

        # while self.t<=500:
        #     for i in range(1,10,2):    ## 3，5，7，9，11
        #         s=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2*self.d)))+400+i*2.1
        #         print(12-i,int(s),self.t)
        #         self.servo.move_time_write(12-i,int(s),3)
        #     time.sleep(0.003)
        #     self.t=self.t+1
        # time.sleep(0.5)
        # for i in range(1, 12, 2):
        #     print(i)  
        #     self.servo.move_time_write(i,400,500)        
        #     if i==1 or i==3:
        #         self.servo.move_time_write(i,410,500)
        #     if i==9 or i==11:
        #         self.servo.move_time_write(i, 370, 500)   
        # time.sleep(1)
        # for i in range(0, SNAKE_LENGTH, 2):
        #     if i==0:
        #         s = 130 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400+50
        #         self.servo.move_time_write(i, int(s), 500)
        #         print (i,int(s),self.t)
        #         time.sleep(0.5)         
        #     if i==2:
        #         s = 160 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
        #         self.servo.move_time_write(i, int(s), 500)
        #         print (i,int(s),self.t)
        #         time.sleep(0.5)
        #     if i==4 or i==6 or i==8:
        #         s = 190* (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
        #         self.servo.move_time_write(i, int(s), 200)
        #         print (i,int(s),self.t)
        #         time.sleep(0.2)
        #     if i==10:
        #         s = 210 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400-80
        #         self.servo.move_time_write(i, int(s), 200)
        #         print (i,int(s),self.t)
        #         time.sleep(0.2)
        # self.t=self.t+1

        # while self.t>400 and self.t<=1200:
        #     for i in range(0, SNAKE_LENGTH, 2):
        #         if i==0:
        #             s = 170 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400+50
        #             self.servo.move_time_write(i, int(s), 4)
        #             print (i,int(s),self.t)
        #             time.sleep(0.004)         
        #         if i==2:
        #             s = 210 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
        #             self.servo.move_time_write(i, int(s), 4)
        #             print (i,int(s),self.t)
        #             time.sleep(0.004)
        #         if i==4 or i==6 or i==8:
        #             s = 240* (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
        #             self.servo.move_time_write(i, int(s), 4)
        #             print (i,int(s),self.t)
        #             time.sleep(0.004)
        #         if i==10:
        #             s = 240 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400-80
        #             self.servo.move_time_write(i, int(s), 5)
        #             print (i,int(s),self.t)
        #             time.sleep(0.005)
        #     self.t=self.t+1


    ######  蜿蜒
    def wanyan_zhixing(self):  ###直行
        # self.servo.move_time_write(11, 370, 100)  
        # self.servo.move_time_write(9, 370, 1000)  
        while True:
            self.servo.move_time_write(0,430,2)
            for i in range(1,SNAKE_LENGTH,2):    ##1，3，5，7，9，11
                s=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2*self.d)))+400+i*1.8
                # print(12-i,int(s),self.t)
                print('蜿蜒直行')
                self.servo.move_time_write(12-i,int(s),2)
            time.sleep(0.002)
            self.t=self.t+1
    def wanyan_zhuanwan(self):  ###蜿蜒转弯
        while True:
            for i in range(0, SNAKE_LENGTH, 2):
                if i==0:
                    s = 170 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400+50
                    self.servo.move_time_write(i, int(s), 4)
                    print (i,int(s),self.t)
                    time.sleep(0.004)         
                if i==2:
                    s = 210 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                    self.servo.move_time_write(i, int(s), 4)
                    print (i,int(s),self.t)
                    time.sleep(0.004)
                if i==4 or i==6 or i==8:
                    s = 240* (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                    self.servo.move_time_write(i, int(s), 4)
                    print (i,int(s),self.t)
                    time.sleep(0.004)
                if i==10:
                    s = 240 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400-80
                    self.servo.move_time_write(i, int(s), 5)
                    print (i,int(s),self.t)
                    time.sleep(0.005)
                # if not self._run:
                #     print('rudong_s exit')
                #     return
            self.t = self.t + 1
            
        # while 1:
        #     # if self.k==0:      
        #         a=3*self.k
        #         b=0
        #         for m in range(0,3*self.k,1):  
        #             for i in range(1, SNAKE_LENGTH, 2):
        #                 s1=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2*self.d)))+400-i*1.8
        #                 s2=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2*self.d)))+400-i*1.8+i*self.k
        #                 s=(b*s1+a*s2)/(3*self.k)
        #                 print(12-i,int(s),b,a)
        #                 self.servo.move_time_write(12-i,int(s),2)
        #             a=a-1
        #             b=b+1
        #             time.sleep(2) 
                # while self.k==0:
                #     for i in range(1,SNAKE_LENGTH,2):    ##1，3，5，7，9，11
                #         s=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2*self.d)))+400-i*1.8
                #         print(12-i,int(s),self.t)
                #         self.servo.move_time_write(12-i,int(s),2)
                #     time.sleep(0.002)
                #     self.t=self.t+1
                    # if not self._run:
                    #     print('home exit')
                    #     return 
            # if self.k!=0:
                # a=3*self.k
                # b=0
                # for m in range(0,3*self.k,1):  
                #     for i in range(1, SNAKE_LENGTH, 2):
                #         s1=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2*self.d)))+400-i*1.8
                #         s2=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2*self.d)))+400-i*1.8+i*self.k
                #         s=(a*s1+b*s2)/(3*self.k)
                #         print(12-i,int(s),a,b)
                #         self.servo.move_time_write(12-i,int(s),2)
                #     a=a-1
                #     b=b+1
                #     time.sleep(2)       
            #     while self.k!=0:           
            #         for i in range(1,SNAKE_LENGTH,2):
            #             s=190*(math.sin(math.pi*self.t/125+math.pi*(i-1)/(2*self.d)))+400-i*1.8+i*self.k  
            #             print(i,int(s),self.t)
            #             self.servo.move_time_write(12-i,int(s),1)
            #         time.sleep(0.002)
            #         self.t=self.t+1
                    # if not self._run:
                    #     print('home exit')
                    #     return
    def fanshen_L90(self):   #####左边侧翻90°   回正
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000)    
        time.sleep(2)
        self.servo.move_time_write(3, 150, 1000)
        self.servo.move_time_write(9, 150, 1000)  
        time.sleep(0.8)
        self.servo.move_time_write(2, 150, 1000)
        self.servo.move_time_write(10, 150, 1000)    
        time.sleep(1.5)
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000) 
    def fanshen_R90(self):   #####右边90°侧翻回正
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000)    
        time.sleep(1)
        self.servo.move_time_write(3, 650, 1000)
        self.servo.move_time_write(9, 650, 1000)  
        time.sleep(0.9)
        self.servo.move_time_write(2, 150, 1000)
        self.servo.move_time_write(10, 150, 1000)    
        time.sleep(1.5)
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000)
    def fanshen_L180(self):   #####180°侧翻回正
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000)    
        time.sleep(2)
        self.servo.move_time_write(2, 150, 1000)
        self.servo.move_time_write(4, 200, 1000)
        self.servo.move_time_write(8, 150, 1000)
        self.servo.move_time_write(10, 200, 1000)  
        time.sleep(0.9)
        self.servo.move_time_write(1, 100, 1000)
        self.servo.move_time_write(11, 100, 1000)    
        time.sleep(1.5)
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000) 
        time.sleep(1)
        self.servo.move_time_write(3, 650, 1000)
        self.servo.move_time_write(9, 650, 1000)  
        time.sleep(0.9)
        self.servo.move_time_write(2, 150, 1000)
        self.servo.move_time_write(10, 150, 1000)    
        time.sleep(1.5)
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000)
    def fanshen_R180(self):   #####180°侧翻回正
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000)    
        time.sleep(2)
        self.servo.move_time_write(2, 650, 1000)
        self.servo.move_time_write(4, 600, 1000)
        self.servo.move_time_write(8, 650, 1000)
        self.servo.move_time_write(10, 600, 1000)  
        time.sleep(0.9)
        self.servo.move_time_write(1, 700, 1000)
        self.servo.move_time_write(11, 700, 1000)    
        time.sleep(1.5)
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000) 
        time.sleep(1)
        self.servo.move_time_write(3, 150, 1000)
        self.servo.move_time_write(9, 150, 1000)  
        time.sleep(0.9)
        self.servo.move_time_write(2, 650, 1000)
        self.servo.move_time_write(10, 650, 1000)    
        time.sleep(1.5)
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000) 

                
    def fangun_R(self):
        self.n=0
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000) 
        time.sleep(2)
        for self.n in range(20):
            self.servo.move_time_write(3, 550, 1000)
            self.servo.move_time_write(9, 550, 1000)  
            time.sleep(1)
            self.servo.move_time_write(2, 250, 1000)
            self.servo.move_time_write(10, 250, 1000)    
            time.sleep(1)
            self.servo.move_time_write(3, 250, 1000)
            self.servo.move_time_write(9, 250, 1000)  
            time.sleep(1)
            self.servo.move_time_write(2, 550, 1000)
            self.servo.move_time_write(10, 550, 1000)    
            time.sleep(1)
            self.n=self.n+1
    def fangun_L(self):
        for i in range(0,SNAKE_LENGTH,1):
            print(i)
            self.servo.move_time_write(i,400,1000)
            if i==1 or i==3:
                self.servo.move_time_write(i,410,1000)
            if i==9 or i==11:
                self.servo.move_time_write(i, 370, 1000) 
        time.sleep(2)
        for n in range(20):
            self.servo.move_time_write(3, 250, 1000)
            self.servo.move_time_write(9, 250, 1000)  
            time.sleep(1)
            self.servo.move_time_write(2, 250, 1000)
            self.servo.move_time_write(10, 250, 1000)    
            time.sleep(1)
            self.servo.move_time_write(3, 550, 1000)
            self.servo.move_time_write(9, 550, 1000)  
            time.sleep(1)
            self.servo.move_time_write(2, 550, 1000)
            self.servo.move_time_write(10, 550, 1000)    
            time.sleep(1)
            self.n=self.n+1


    def cehua_R(self):
        while self._run:
            for i in range(0, SNAKE_LENGTH, 2):
                s = 150 * (math.sin( math.pi* self.t/60 + math.pi * i /4)) + 400
                self.servo.move_time_write(10-i, int(s), 2)
                self.servo.move_time_write(11-i, int(s), 2)
                time.sleep(0.003)
                if not self._run:
                    print('fangun exit')
                    return
            self.t = self.t + 1
            print (self.t)
    def cehua_L(self):
        while self._run:
            for i in range(0, SNAKE_LENGTH, 2):
                s = 160 * (math.sin( math.pi* self.t/60 + math.pi * i /4)) + 400
                self.servo.move_time_write(10-i, 800-int(s), 2)
                self.servo.move_time_write(11-i, 800-int(s), 2)
                time.sleep(0.003)
                if not self._run:
                    print('fangun exit')
                    return
            self.t = self.t + 1
            print (self.t)
    def rudong_u(self):
        
        while True:
            if self.M !=0:
                for i in range(0, 12, 1):
                    self.servo.move_time_write(i,400,1000)
                    if i==0:
                        self.servo.move_time_write(i,560,1000)
                    if i==2:
                        self.servo.move_time_write(i,110,1000)
                    if i==3:
                        self.servo.move_time_write(i,410,1000)
                    if i==9 or i==11:
                        self.servo.move_time_write(i, 370, 1000)
                time.sleep(1.2)
                while self.M !=0:
                    for i in range(2, SNAKE_LENGTH, 2):
                        if i==2 and 150 * (math.sin( math.pi* self.t/25))>0:
                            s = 150 * (math.sin( math.pi* self.t/25)) + 690
                            self.servo.move_time_write(i, 800-int(s), 4)
                            print (i,int(s),self.t)
                            time.sleep(0.004)
                        if i==4:
                            s = 190 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                            self.servo.move_time_write(i, 800-int(s), 4)
                            print (i,int(s),self.t)
                            time.sleep(0.004)
                        if i==10:
                            s = 220 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400-80
                            self.servo.move_time_write(i, 800-int(s), 4)
                            print (i,int(s),self.t)
                            time.sleep(0.004)
                        if i==6 or i==8:
                            s = 200 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                            self.servo.move_time_write(i, 800-int(s), 4)
                            print (i,int(s),self.t)
                            time.sleep(0.004)
                        if not self._run:
                            print('rudong_s exit')
                            return
                    self.t=self.t+1 
            while self.M==0:
                for i in range(1, SNAKE_LENGTH, 2):
                    if i==1:
                        self.servo.move_time_write(i,self.N+10,500)
                    if i==3:
                        self.servo.move_time_write(i,self.N+10,500)
                    if i==9:
                        self.servo.move_time_write(i, self.N-30, 500)
                    if i==11:
                        self.servo.move_time_write(i, self.N-30, 500)
                    if i==5: 
                        self.servo.move_time_write(i,self.N,10)
                    if i==7: 
                        self.servo.move_time_write(i,self.N,10)
                for i in range(0, SNAKE_LENGTH, 2):
                    if i==0:
                        s = 170 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400+50
                        self.servo.move_time_write(i, 800-int(s), 4)
                        print (i,int(s),self.t)
                        time.sleep(0.004)         
                    if i==2:
                        s = 210 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                        self.servo.move_time_write(i, 800-int(s), 4)
                        print (i,int(s),self.t)
                        time.sleep(0.004)
                    if i==4 or i==6 or i==8:
                        s = 240* (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                        self.servo.move_time_write(i, 800-int(s), 4)
                        print (i,int(s),self.t)
                        time.sleep(0.004)
                    if i==10:
                        s = 240 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400-80
                        self.servo.move_time_write(i, 800-int(s), 5)
                        print (i,int(s),self.t)
                        time.sleep(0.005)
   
                self.t = self.t + 1
    def rudong_X(self):
        # self.servo.move_time_write(1,330,20)
        # self.servo.move_time_write(5,330,20)
        # self.servo.move_time_write(9,330,20)
        # self.servo.move_time_write(3,470,20)
        # self.servo.move_time_write(7,470,20)
        # self.servo.move_time_write(11,450,20)
        # time.sleep(0.050)
        while True:  
            for i in range(0, SNAKE_LENGTH, 2):
                if i==0:
                    s = 170 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400+50
                    self.servo.move_time_write(i, 800-int(s), 4)
                    print (i,800-int(s),self.t)
                    time.sleep(0.004)         
                if i==2:
                    s = 210 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                    self.servo.move_time_write(i, 800-int(s), 4)
                    print (i,int(s),self.t)
                    time.sleep(0.004)
                if i==4 or i==6 or i==8:
                    s = 240* (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400
                    self.servo.move_time_write(i, 800-int(s), 4)
                    print (i,int(s),self.t)
                    time.sleep(0.004)
                if i==10:
                    s = 240 * (math.sin( math.pi* self.t/25 + math.pi * i /4)) + 400-80
                    self.servo.move_time_write(i, 800-int(s), 5)
                    print (i,int(s),self.t)
                    time.sleep(0.005)
            self.t=self.t+1

    def main(self):
        while True:
            user_input = input("请输入数字0到7(复位0 蠕动1 蠕动2 蜿蜒直行3 翻滚左4  翻滚右5 侧滑左边6 侧滑右7)（输入'q'退出）：")
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
# if __name__ == "__main__":
#     demo = Process()
#     demo.fuwei()
