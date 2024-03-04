#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Header
import geometry_msgs.msg
import tf

rospy.init_node('odom_pub')

odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()
header = Header()
header.frame_id = '/odom'

model = GetModelStateRequest()
model.model_name = 'musashi_robot'

r = rospy.Rate(50)

while not rospy.is_shutdown():

    result = get_model_srv(model)

    odom.pose.pose = result.pose
    # makes no difference (covariance)
    odom.pose.covariance = \
    [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, \
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0, \
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0, \
    0.0, 0.0, 0.0, 0.01, 0.0, 0.0, \
    0.0, 0.0, 0.0, 0.0, 0.01, 0.0, \
    0.0, 0.0, 0.0, 0.0, 0.0, 0.03]

    odom.twist.twist = result.twist
    odom.twist.covariance = \
    [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, \
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0, \
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0, \
    0.0, 0.0, 0.0, 0.01, 0.0, 0.0, \
    0.0, 0.0, 0.0, 0.0, 0.01, 0.0, \
    0.0, 0.0, 0.0, 0.0, 0.0, 0.03]

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish(odom)

    # dont know what exactly that is doing
    t = tf.Transformer(True, rospy.Duration(10.0))
    odom_trans = geometry_msgs.msg.TransformStamped()
    odom_trans.header.stamp = rospy.Time.now()
    odom_trans.header.frame_id = 'odom'
    odom_trans.child_frame_id = 'base_link'
    odom_trans.transform.translation.x = odom.pose.pose.position.x
    odom_trans.transform.translation.y = odom.pose.pose.position.y
    odom_trans.transform.translation.z = odom.pose.pose.position.z
    odom_trans.transform.rotation.x = odom.pose.pose.orientation.x
    odom_trans.transform.rotation.y = odom.pose.pose.orientation.y
    odom_trans.transform.rotation.z = odom.pose.pose.orientation.z
    odom_trans.transform.rotation.w = odom.pose.pose.orientation.w
    t.setTransform(odom_trans)
    #

    br = tf.TransformBroadcaster()
    br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z), \
    (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w), rospy.Time.now(), "base_link", "odom")

    r.sleep()
