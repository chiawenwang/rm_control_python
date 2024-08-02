# ! /usr/bin/python3
import rospy
import actionlib
import rm_msgs.msg
import std_msgs.msg 
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import math
from robotic_arm_package.robotic_arm import *

class RealmanController(object):

    def __init__(self, prefix='right_'):
        
        try:
            rospy.init_node(prefix + 'realman_controller', anonymous=True)
        except:
            pass

        #python API for realman
        if prefix == 'right_':
            ip = '192.168.31.105'
        elif prefix == 'left_':
            ip = '192.168.31.106'
        else:
            ip =''

        self.realman_arm = Arm(RM75, ip)
        print('connected to realman arm: '+ prefix)
        print(self.realman_arm.API_Version())


        self.current_joint_state = None
        self.arm_joint_number = 7
        self.prefix = prefix

        rospy.Subscriber('/joint_states', JointState, self._callback_joint_state)
        print('Subscribed to /joint_state')

        # self.velocity_publisher = rospy.Publisher(
        #     '/j2n6s300_driver/in/cartesian_velocity',
        #     kinova_msgs.msg.PoseVelocity,
        #     queue_size = 1
        # )

    # # 筛选左右关节，仅仅包含关节角度
    # def _callback_joint_state(self, data):
    #     if data is not None and data.position is not None:
    #         for i in range(len(data.name)):
    #             if data.name[i].startswith(self.prefix):
    #                 self.current_joint_state = data.position
    def _callback_joint_state(self, data):
        if any(self.prefix in name for name in data.name):
            self.current_joint_state = data


    def joint_movement(self, input_joint, follow = False , expand = 0):
        self.realman_arm.Movej_CANFD(input_joint, follow, expand)
        print('moved to ' + str(joint))

    
    def get_current_position(self):
        if self.current_joint_state == None:
            print('No joint pose read!')
            return

        print(self.current_joint_state.position)

if __name__ == '__main__':
    controller = RealmanController(prefix='right_')
    joint=[0, 0.6056308069229126, -2.6268935775756835, -1.851636527633667, 0.017973499500751496, -1.3489199340820313, -0.0016752000145614147]
    # controller.joint_movement(joint)
    rospy.spin()