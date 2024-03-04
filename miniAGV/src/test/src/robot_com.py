#!/usr/bin/env python
import rospy
from kv_host_link import *
from constant import *
from test_feedforward import *

class RobotCommunication:

    #def callback_velocity_command(self,msg):
    #    rospy.loginfo("Message '{}' recieved".format(msg.data))
    #    self.v_l = msg.data[0]
    #    self.v_r = msg.data[1]
        # rospy.loginfo("v_l: '{}' ".format(self.v_l))
        # rospy.loginfo("VELOCITY_LEFT : {}".format(msg.data[0]))
        # rospy.loginfo("VELOCITY_RIGHT : {}".format(msg.data[1]))
        

    #def write_robot(self):
    #    if not kv.write_plc(VELOCITY_LEFT, self.v_l):
    #        rospy.loginfo("VL command sent.")
    #    else:
    #        rospy.logwarn("VL command sent error")

    #    if not kv.write_plc(VELOCITY_RIGHT, self.v_r):
    #        rospy.loginfo("VR command sent.")
    #    else:
    #        rospy.logwarn("VR command sent error")



    def robot_com(self):

        rospy.init_node('robot_communication', anonymous=True)
        
        if not kv.connect_plc():
            rospy.loginfo("Connection to PLC successful.")

        else:
            rospy.logwarn("PLC connection error!")

        rospy.spin()

        rate = rospy.Rate(20) # ROS Rate at 20Hz

        while not rospy.is_shutdown():
            ffw.write_to_plc()
            rate.sleep()

        kv.close_plc()


if __name__ == '__main__':
    kv = KvHostLink()
    rc = RobotCommunication()
    ffw = TestFeedForwardControl()
    rc.robot_com()

