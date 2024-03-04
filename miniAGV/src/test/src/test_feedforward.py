#!/usr/bin/env python

import rospy
import pandas as pd
from kv_host_link import *
from constant import *
from std_msgs.msg import Int16MultiArray

class TestFeedForwardControl:
    def __init__(self):
        #self.count = 0
        self.v_l = 0
        self.v_r = 0
        
    def callback_velocity_command(self,msg):
       rospy.loginfo("Message '{}' recieved".format(msg.data))
       self.v_l = msg.data[0]
       self.v_r = msg.data[1]

    def write_to_plc(self):
        # ref_vel_ = pd.read_csv('/home/syseng402/test_ws/trajectory_with_rpm.csv', usecols=[' v_l [rpm]',' v_r [rpm]'])
        # velocity = ref_vel_.values.tolist()
        # if  self.count == 1400 :
        #     self.v_l = 0
        #     self.v_r = 0
        # else:
        #     self.v_l = velocity[self.count][0]
        #     self.v_r = velocity[self.count][1]

        #     self.count += 1

        # print(self.v_l, self.v_r)

        rospy.init_node('control_node', anonymous = True)

        rospy.Subscriber("/cmd_vel", Int16MultiArray, self.callback_velocity_command)

        v_l = self.v_l
        v_r = self.v_r

        if not kv.write_plc(VELOCITY_LEFT, v_l):
            rospy.loginfo("VL command sent.")
        else:
            rospy.logwarn("VL command sent error")

        if not kv.write_plc(VELOCITY_RIGHT, v_r):
            rospy.loginfo("VR command sent.")
        else:
            rospy.logwarn("VR command sent error")

if __name__ == '__main__':

    kv = KvHostLink()
   
    controller = TestFeedForwardControl(0.0, 0.0)
    
    rospy.Timer(rospy.Duration(0.1), controller.write_to_plc)

    rospy.spin()