import actionlib
import rm_msgs.msg
import std_msgs.msg 
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import math
import time
import numpy as np
from robotic_arm_package.robotic_arm import *
import socket
import time

class InspireController(object):

    def __init__(self, prefix='right_'):
        
        #python API for realman
        if prefix == 'right_':
            ip = '192.168.31.105'
        elif prefix == 'left_':
            ip = '192.168.31.106'
        else:
            ip =''

        self.realman_arm = Arm(RM75, ip)
        print('connected to realman arm: '+ prefix)
        # print(self.realman_arm.)

        self.prefix = prefix
        self.ip = ip
        self.port = 8080
    
    # def Socket_Connect(self):

    #     client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    #     try:

    #         client.connect((self.ip,self.port))

    #         print("机械臂末端工具Socket连接成功")

    #         return True

    #     except socket.error as e:

    #         print("机械臂末端工具Socket连接失败")

    #         return False

    
if __name__ == "__main__":

    # inspire_R = InspireController('right_')
    # inspire_R.realman_arm.Set_Modbus_Mode(1,115200,5)
    # inspire_R.realman_arm.Set_Hand_Posture(1)
    # inspire_R.realman_arm.Set_Hand_Angle([900,500,900,900,900,200])


    inspire_L = InspireController('left_')
    inspire_L.realman_arm.Set_Modbus_Mode(1,115200,5)
    # inspire_L.realman_arm.Set_Hand_Posture(1)
    inspire_L.realman_arm.Set_Hand_Angle([300,500,900,900,300,300])
    # time.sleep(0.01)

    # try:
    #     while True:
    #         time.sleep(0.01)
    #         read = inspire_L.realman_arm.Read_Multiple_Holding_Registers(1,1546,6,1)
    #         print(read)
    # except KeyboardInterrupt:
    #     pass