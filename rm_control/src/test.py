      
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class MySubscriber:
    def __init__(self):
        rospy.init_node('my_subscriber', anonymous=True)
        self.sub = rospy.Subscriber("/my_topic", String, self.callback)
        rospy.loginfo("Subscriber node started")

    def callback(self, data):
        rospy.loginfo("Received message: %s", data.data)

def main():
    try:
        my_subscriber = MySubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
