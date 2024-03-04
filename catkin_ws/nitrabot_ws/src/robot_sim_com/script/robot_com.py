#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import rospy

if __name__ == '__main__':
    robot_port = serial.Serial()
    robot_port.baudrate = 38400
    robot_port.port = '/dev/ttyUSB0'
    robot_port.parity = serial.PARITY_EVEN
    robot_port.stopbits = serial.STOPBITS_ONE
    robot_port.bytesize = serial.EIGHTBITS
    print "Port is configured"
    robot_port.open()
    if robot_port.is_open:
        print "Port is open"
        send = []
        send.append(bytes([0]))
        print bin(0x00)
        send.append(bytes([160]))
        print bin(0xa0)
        send.append(bytes([0]))  # right speed
        print bin(0x32)
        send.append(bytes([50]))
        print bin(0x00)
        send.append(bytes([0]))  # left speed
        print bin(0x32)
        send.append(bytes([50]))
        print bin(0x00)
        send.append(bytes([128]))
        print bin(0x80)
        send.append(bytes([124]))  # checksum
        print bin(0x7b)
        send = bytearray("\x00\xA0\x32\x00\x32\x00\x80\x7b",'UTF-8')
        #send = "\x00\xA0\x32\x00\x32\x00\x80\x7b"
        print send
        #print robot_port.in_waiting
        robot_port.write(send)
    else:
        print "Could not open"
    
