#!/usr/bin/env python3

import rospy
import threading
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

class TestNode1:
    def __init__(self):
        rospy.init_node('test_node1', anonymous=True)

        self.vel_pub = rospy.Publisher('API/cmd_vel', Twist, queue_size=10)
        self.vel_sub = rospy.Subscriber('feedback_vel', Twist, self.callback)

        self.service = rospy.Service('API/follow_me_mode', SetBool, self.handle_service)
        self.service_client = rospy.ServiceProxy('API/follow_me_mode', SetBool)
        
        self.vel_msg = Twist()
        self.timer_thread = threading.Thread(target=self.timer_func)
        self.timer_thread.start()

    def callback(self, msg):
        rospy.loginfo("Received velocity: %s", msg)  # Log received message

    def handle_service(self, req):
        rospy.loginfo("Service request received: %s", req.data)
        response = SetBoolResponse()
        response.success = True
        response.message = "The request was received!"
        return response

    def timer_func(self):
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            self.vel_msg.linear.x = 1.0
            self.vel_pub.publish(self.vel_msg)
            try:
                response = self.service_client(True)
                rospy.loginfo("Service call succeeded: %s", response.message)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s", e)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TestNode1()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass