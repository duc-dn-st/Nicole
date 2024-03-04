#!/usr/bin/env python
# Standard libaries
import io
import math

# External libaries
import serial
import rospy
from std_msgs.msg import Float32

# Constant
BAUDRATE = 9600

def publish():
    params = rospy.get_param_names()

    port, node_name = "default", "default"

    for param in params:
        if "/port" in param:
            port = rospy.get_param(param)
            node_name = port.split("/")[1]
            break

    rospy.init_node(node_name, anonymous=False)

    publisher = rospy.Publisher(node_name, Float32, queue_size=10)

    serial_connection = serial.Serial(
                port=port,
                baudrate=BAUDRATE,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.SEVENBITS,
                timeout=0.1
            )
    serial_io = io.TextIOWrapper(
                io.BufferedRWPair(serial_connection, serial_connection, 1),
                encoding='ascii', newline='\r')

    while not rospy.is_shutdown():
        if not serial_connection.isOpen():
            continue

        data = serial_io.readline()

        # Get encoder data
        msg = create_msg(data, node_name)

        data_temp = data[2:].split(",")
        multiturn_count = int(data_temp[0])
        singleturn_count = int(data_temp[1])
        msg_temp = msg *180/math.pi
        str = multiturn_count, singleturn_count, msg_temp
        rospy.loginfo(str)
        publisher.publish(msg)


def create_msg(data, node_name):

    msg = Float32()

    data_array = data[2:].split(",")

    # degree or radians?
    angle = calculate_angle(data_array)

    msg.data = angle

    return angle

def calculate_angle(data_array):

    if len(data_array) != 2:
        raise ValueError("Incorrect format of encoder reading")

    multiple_turn_count = int(data_array[0])

    single_turn_count = int(data_array[1])

    if multiple_turn_count > (65535 - 96):
        multiple_turn_count -= 65535

    # TODO
    if abs(multiple_turn_count) > 24:
        angle = math.pi/2
        return angle

    angle = (multiple_turn_count * 131071.0 + single_turn_count) / (131071.0 * 96.0) * 2 * math.pi

    return angle


if __name__ == "__main__":
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
