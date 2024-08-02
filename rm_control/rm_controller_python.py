#! /usr/bin/python3
import rospy
import actionlib
import rm_msgs.msg
import std_msgs.msg 
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import math
import time
import numpy as np
from robotic_arm_package.robotic_arm import *

class RealmanController(object):

    def __init__(self, prefix='right_'):

        # try:
        #     rospy.init_node(prefix + 'Dexarm', disable_signals = True, anonymous=True)
        # except:
        #     pass
        
        #python API for realman
        if prefix == 'right_':
            ip = '192.168.31.105'
        elif prefix == 'left_':
            ip = '192.168.31.106'
        else:
            ip =''

        self.realman_arm = Arm(RM75, ip)
        self.realman_arm.Set_Modbus_Mode(1,115200,5)
        print('connected to realman arm: '+ prefix)

        self.prefix = prefix
    
    # Get states
    # joint_position: (°)
    # pose.position:(mm)
    # pose.orientation:(rad)
    def get_arm_state(self):
        return self.realman_arm.Get_Current_Arm_State()
    
    def get_arm_err(self):
        return self.realman_arm.Get_Current_Arm_State()[3]
    
    def clear_arm_err(self):
        self.realman_arm.Clear_System_Err(False)

    def get_current_joint_position(self):
        return self.realman_arm.Get_Current_Arm_State()[1]
    
    def get_current_pose(self):
        return self.realman_arm.Get_Current_Arm_State()[2]
    
    def solve_ik(self, joint, goal, flag):
        # du m rad
        res = Arm.Algo_Inverse_Kinematics(joint, goal, flag = 0)
        tag = res[0]
        joints = res[1]
        return tag, joints
    
    # movements
    def joint_movement(self, input_joint, velocity, trajectory_connect = 0, r = 0, block = False):
        self.realman_arm.Movej_Cmd(input_joint, velocity, trajectory_connect, r, block)
        print('moved to ' + str(input_joint))

    def joint_move_touchuan(self, joint, follow = False):
        # 角度透传，不需要插值
        self.realman_arm.Movej_CANFD(joint, follow)

    def cartesian_control(self, target_cartesian):
        # 位姿透传，需要插值
        self.realman_arm.Movep_CANFD(target_cartesian, False)

    def cartesian_movement(self, target_cartesian, velocity=8, trajectory_connect = 0, r = 0, block = True):
        self.realman_arm.Movej_P_Cmd(target_cartesian,velocity, trajectory_connect, r, block)

    def IK_solver_quaternion(self, pose,flag = 0):
        joint = self.get_current_joint_position()
        return self.realman_arm.Algo_Inverse_Kinematics(joint, pose, flag)

# Hand
    def get_hand_joint_position(self):
        res = self.realman_arm.Read_Multiple_Holding_Registers(1,1546,6,1)
        if res[0] == 0:
            return res[1]
        else:
            print('No hand joint!!')
            return None
        
    def move_finger(self, joints):
        # input is percent
        # To (100,900)
        controller = []
        controller[:5] = np.array(joints) * 600 + 350
        controller[:2] = [0,0]
        controller.append(100)
        # print(np.array(controller).astype(int))
        tag = self.realman_arm.Set_Hand_Angle(np.array(controller).astype(int))

        return tag
    
    def eu2qua(self, eular):
        return self.realman_arm.Algo_Euler2Quaternion(eular)
    
    def qua2eu(self, qua):
        return self.realman_arm.Algo_Quaternion2Euler(qua)
    
    def get_limit(self):
        return self.realman_arm.Algo_Get_Joint_Min_Limit()
    
    def get_set_up_angle(self):
        return self.realman_arm.Algo_Get_Angle()
    
    def set_up_angle(self,x,y,z):
        self.realman_arm.Algo_Set_Angle(x, y, z)

    def cart_tool(self, joints, dx, dy, dz):
        # optional
        return self.realman_arm.Algo_Cartesian_Tool(joints, dx, dy, dz)


if __name__ == '__main__':
    controller = RealmanController(prefix='left_')
    # joint = [1,1,1,1,1]
    # pose = controller.get_current_pose()

    # print(controller.eu2qua(pose[-3:]))

    print('joint:',controller.get_current_joint_position())

    controller.set_up_angle(-90, 0, 0)

    print(controller.get_set_up_angle())

    print(controller.solve_ik(controller.get_current_joint_position(), [0.41520199179649353, 0.18467099964618683, -0.0906430035829544, 0.3792132310902548, 0.8211619988188442, -0.2609225139900808, 0.33735699007879694], 0 ))

    # qua = [0.44 , 0.85 , 0.09 , 0.26]

    # print(controller.qua2eu(qua))

    # print(controller.get_arm_state())
    # target =[ 0.4 , -0.18 , 0.33 , 0.1 ,  0.79 ,-0.41 , 0.44]
    # controller.cartesian_movement(target, velocity=5)

    # joint_position = controller.get_current_pose()
    # print(joint_position)

    # try:
    #     while True:
    #         ik_pose = controller.IK_solver_quaternion([0.31035100519657135, 0.24086299538612366, 0.08026599884033203, -0.9666169671015036, 0.25609960230190737, 0.007937018514344838, 0.0012791987330616457])
    #         print(ik_pose)
    #         time.sleep(0.05)
    
    # except KeyboardInterrupt:
    #     pass



    # pose = controller.get_arm_state()[1]
    # target = [0.3705959916114807, 0.11114300042390823, 0.4618769884109497, -2.4730000495910645, -1.0800000429153442, -0.32899999618530273]
    # controller.cartesian_movement(target, velocity=30)
    # print(controller.get_arm_err())
    # print(controller.clear_arm_err())
    # print(controller.get_arm_err())


    # print(pose[1])

    # for i in range(40):
    #     pose[1] -= 0.001
    #     controller.cartesian_movement(pose)
    #     time.sleep(0.02)
    #     print(pose[1])

    # joint_position[0] += 20

    # controller.joint_movement(joint_position, velocity = 20)
    # joint_position[0] -= 10
    # controller.joint_movement(joint_position, velocity = 8)

    # rospy.spin()