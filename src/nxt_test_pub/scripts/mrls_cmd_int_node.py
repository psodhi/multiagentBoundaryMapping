#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import roslib

import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
import serial

global serial_port

sensor_max_reading_i = 420 # max reading our sensors were giving to subtract 
sensor_max_reading_j = 420
sensor_max_reading_k = 420
sensor_max_reading_l = 420
sensor_max_reading_m = 420

motor_max_translate = 50
motor_max_scale = 400
motor_max_rotate = 100
motor_min_rotate = 70

sensor_a_trim = 1000
sensor_b_trim = 2000
sensor_c_trim = 3000
sensor_d_trim = 4000
sensor_e_trim = 5000

motor_a_acknowledge = 118 # ASCII Value for 'v'
motor_b_acknowledge = 119
motor_c_acknowledge = 120
motor_d_acknowledge = 121
motor_e_acknowledge = 122

intensity_flag = [0,0,0,0,0]

def callback0(cmd):
	print "callback0"
	if cmd.linear.x == 1.0:
		exception_error = True
		while (exception_error==True):
			try:
				serial_port.flushOutput()
				serial_port.write("mmmmm")
				time.sleep(1)
				temp_str = serial_port.readline()
				if temp_str != "":
					print "mmmmm ACK "+ temp_str[0]
					if ord(temp_str[0]) == motor_a_acknowledge: # ASCII Value for v
						pub_front0.publish(motor_a_acknowledge)
						exception_error = False
						print "Exception Error BOT 0: "+ str(exception_error)
			except:
				exception_error = True # waste statement		
	elif cmd.angular.z < 0.0:
				motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
				print "al"+motor_cmd_val.zfill(3)
				serial_port.flushOutput()		
				serial_port.write("al"+motor_cmd_val.zfill(3))
	elif cmd.angular.z > 0.0:
				motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
				print "ar"+motor_cmd_val.zfill(3)
				serial_port.flushOutput()
				serial_port.write("ar"+motor_cmd_val.zfill(3))				

def light_callback0(msg):
	print "light_callback0"
	intensity_flag[0] = 0
	while intensity_flag[0]==0:
		try:
			serial_port.flushOutput()
			serial_port.write("iiiii")
			time.sleep(1)
			temp_str = serial_port.readline()
			if temp_str != "":
				temp = int(temp_str)
				print "iiiii readline() "+str(temp)+"    /     "+temp_str
				intensity = temp

				if ((temp - sensor_a_trim)>=0) and ((temp - sensor_a_trim) <= sensor_max_reading_i):
					intensity=sensor_max_reading_i - (temp - sensor_a_trim)
					pub_intensity0.publish(intensity)
					print "light_callback0 for bot_0 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[0] = 1

				elif ((temp - sensor_b_trim)>=0) and ((temp - sensor_b_trim) <= sensor_max_reading_j):
					intensity=sensor_max_reading_j - (temp - sensor_b_trim)
					pub_intensity1.publish(intensity)
					print "light_callback0 for bot_1 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[1] = 1

				elif ((temp - sensor_c_trim)>=0) and ((temp - sensor_c_trim) <= sensor_max_reading_k):
					intensity=sensor_max_reading_k - (temp - sensor_c_trim)
					pub_intensity2.publish(intensity)
					print "light_callback0 for bot_2 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[2] = 1					
					
				elif ((temp - sensor_d_trim)>=0) and ((temp - sensor_d_trim) <= sensor_max_reading_l):
					intensity=sensor_max_reading_l - (temp - sensor_d_trim)
					pub_intensity3.publish(intensity)
					print "light_callback0 for bot_3 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[3] = 1
					
				elif ((temp - sensor_e_trim)>=0) and ((temp - sensor_e_trim) <= sensor_max_reading_m):
					intensity=sensor_max_reading_m - (temp - sensor_e_trim)
					pub_intensity4.publish(intensity)
					print "light_callback0 for bot_4 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[4] = 1
		except:
			intensity_flag[0] = 0 # waste statement
			

def callback1(cmd):
	print "callback1"		
	if cmd.linear.x == 1.0:
		exception_error = True
		while (exception_error == True):
			try:
				serial_port.flushOutput()
				serial_port.write("nnnnn")
				time.sleep(1)
				temp_str = serial_port.readline()
				if temp_str != "":
					print "nnnnn ACK "+temp_str[0]
					if ord(temp_str[0]) == motor_b_acknowledge:
						pub_front1.publish(motor_b_acknowledge)
						exception_error = False
						print "Exception Error BOT 1: "+str(exception_error)
			except:
				exception_error = True # waste statement			
	elif cmd.angular.z<0.0:
		motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
		print "bl"+motor_cmd_val.zfill(3)
		serial_port.flushOutput()		
		serial_port.write("bl"+motor_cmd_val.zfill(3))
	elif cmd.angular.z>0.0:
		motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
		print "br"+motor_cmd_val.zfill(3)
		serial_port.flushOutput()
		serial_port.write("br"+motor_cmd_val.zfill(3))

def light_callback1(msg):
	print "light_callback1"
	intensity_flag[1] = 0
	while intensity_flag[1]==0:
		try:
			serial_port.flushOutput()
			serial_port.write("jjjjj")
			time.sleep(1)
			temp_str = serial_port.readline()
			if temp_str != "":
				temp = int(temp_str)
				print "jjjjj readline() "+str(temp)+"    /     "+temp_str
				intensity = temp

				if ((temp - sensor_a_trim)>=0) and ((temp - sensor_a_trim) <= sensor_max_reading_i):
					intensity=sensor_max_reading_i - (temp - sensor_a_trim)
					pub_intensity0.publish(intensity)
					print "light_callback1 for bot_0 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[0] = 1

				elif ((temp - sensor_b_trim)>=0) and ((temp - sensor_b_trim) <= sensor_max_reading_j):
					intensity=sensor_max_reading_j - (temp - sensor_b_trim)
					pub_intensity1.publish(intensity)
					print "light_callback1 for bot_1 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[1] = 1

				elif ((temp - sensor_c_trim)>=0) and ((temp - sensor_c_trim) <= sensor_max_reading_k):
					intensity=sensor_max_reading_k - (temp - sensor_c_trim)
					pub_intensity2.publish(intensity)
					print "light_callback1 for bot_2 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[2] = 1		
					
				elif ((temp - sensor_d_trim)>=0) and ((temp - sensor_d_trim) <= sensor_max_reading_l):
					intensity=sensor_max_reading_l - (temp - sensor_d_trim)
					pub_intensity3.publish(intensity)
					print "light_callback1 for bot_3 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[3] = 1
					
				elif ((temp - sensor_e_trim)>=0) and ((temp - sensor_e_trim) <= sensor_max_reading_m):
					intensity=sensor_max_reading_m - (temp - sensor_e_trim)
					pub_intensity4.publish(intensity)
					print "light_callback1 for bot_4 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[4] = 1
		except:
			intensity_flag[1] = 0 # waste statement
	
	
def callback2(cmd):
	print "callback2"
	if cmd.linear.x == 1.0:
		exception_error = True
		while (exception_error == True):
			try:	
				serial_port.flushOutput()
				serial_port.write("ooooo")
				time.sleep(1)
				temp_str = serial_port.readline()
				if temp_str != "":
					print "ooooo ACK "+temp_str[0]
					if ord(temp_str[0]) == motor_c_acknowledge: # ASCII Value for x
						pub_front2.publish(motor_c_acknowledge)
						exception_error = False
						print "Exception Error BOT 2: "+ str(exception_error)
			except:
				exception_error = True # waste statement
	elif cmd.angular.z < 0.0:
				motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
				print "cl"+motor_cmd_val.zfill(3)
				serial_port.flushOutput()		
				serial_port.write("cl"+motor_cmd_val.zfill(3))
	elif cmd.angular.z>0.0:
				motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
				print "cr"+motor_cmd_val.zfill(3)
				serial_port.flushOutput()
				serial_port.write("cr"+motor_cmd_val.zfill(3))
		
def light_callback2(msg):
	print "light_callback2"
	intensity_flag[2] = 0
	while intensity_flag[2]==0:
		try:
			serial_port.flushOutput()
			serial_port.write("kkkkk")
			time.sleep(1)
			temp_str = serial_port.readline()
			if temp_str != "":
				temp = int(temp_str)
				print "kkkkk readline() "+str(temp)+"    /     "+temp_str
				intensity = temp

				if ((temp - sensor_a_trim)>=0) and ((temp - sensor_a_trim) <= sensor_max_reading_i):
					intensity=sensor_max_reading_i - (temp - sensor_a_trim)
					pub_intensity0.publish(intensity)
					print "light_callback2 for bot_0 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[0] = 1

				elif ((temp - sensor_b_trim)>=0) and ((temp - sensor_b_trim) <= sensor_max_reading_j):
					intensity=sensor_max_reading_j - (temp - sensor_b_trim)
					pub_intensity1.publish(intensity)
					print "light_callback2 for bot_1 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[1] = 1

				elif ((temp - sensor_c_trim)>=0) and ((temp - sensor_c_trim) <= sensor_max_reading_k):
					intensity=sensor_max_reading_k - (temp - sensor_c_trim)
					pub_intensity2.publish(intensity)
					print "light_callback2 for bot_2 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[2] = 1
					
				elif ((temp - sensor_d_trim)>=0) and ((temp - sensor_d_trim) <= sensor_max_reading_l):
					intensity=sensor_max_reading_l - (temp - sensor_d_trim)
					pub_intensity3.publish(intensity)
					print "light_callback2 for bot_3 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[3] = 1
					
				elif ((temp - sensor_e_trim)>=0) and ((temp - sensor_e_trim) <= sensor_max_reading_m):
					intensity=sensor_max_reading_m - (temp - sensor_e_trim)
					pub_intensity4.publish(intensity)
					print "light_callback2 for bot_4 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[4] = 1
		except:
			intensity_flag[2] = 0 # waste statement

def callback3(cmd):
	print "callback3"
	if cmd.linear.x == 1.0:
		exception_error = True
		while (exception_error == True):
			try:	
				serial_port.flushOutput()
				serial_port.write("ppppp")
				time.sleep(1)
				temp_str = serial_port.readline()
				if temp_str != "":
					print "ppppp ACK "+temp_str[0]
					if ord(temp_str[0]) == motor_d_acknowledge: # ASCII Value for x
						pub_front3.publish(motor_d_acknowledge)
						exception_error = False
						print "Exception Error BOT 3: "+ str(exception_error)
			except:
				exception_error = True # waste statement
	elif cmd.angular.z < 0.0:
				motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
				print "dl"+motor_cmd_val.zfill(3)
				serial_port.flushOutput()		
				serial_port.write("dl"+motor_cmd_val.zfill(3))
	elif cmd.angular.z>0.0:
				motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
				print "dr"+motor_cmd_val.zfill(3)
				serial_port.flushOutput()
				serial_port.write("dr"+motor_cmd_val.zfill(3))
		
def light_callback3(msg):
	print "light_callback3"
	intensity_flag[3] = 0
	while intensity_flag[3]==0:
		try:
			serial_port.flushOutput()
			serial_port.write("lllll")
			time.sleep(1)
			temp_str = serial_port.readline()
			if temp_str != "":
				temp = int(temp_str)
				print "lllll readline() "+str(temp)+"    /     "+temp_str
				intensity = temp

				if ((temp - sensor_a_trim)>=0) and ((temp - sensor_a_trim) <= sensor_max_reading_i):
					intensity=sensor_max_reading_i - (temp - sensor_a_trim)
					pub_intensity0.publish(intensity)
					print "light_callback3 for bot_0 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[0] = 1

				elif ((temp - sensor_b_trim)>=0) and ((temp - sensor_b_trim) <= sensor_max_reading_j):
					intensity=sensor_max_reading_j - (temp - sensor_b_trim)
					pub_intensity1.publish(intensity)
					print "light_callback3 for bot_1 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[1] = 1

				elif ((temp - sensor_c_trim)>=0) and ((temp - sensor_c_trim) <= sensor_max_reading_k):
					intensity=sensor_max_reading_k - (temp - sensor_c_trim)
					pub_intensity2.publish(intensity)
					print "light_callback3 for bot_2 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[2] = 1
					
				elif ((temp - sensor_d_trim)>=0) and ((temp - sensor_d_trim) <= sensor_max_reading_l):
					intensity=sensor_max_reading_l - (temp - sensor_d_trim)
					pub_intensity3.publish(intensity)
					print "light_callback3 for bot_3 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[3] = 1
					
				elif ((temp - sensor_e_trim)>=0) and ((temp - sensor_e_trim) <= sensor_max_reading_m):
					intensity=sensor_max_reading_m - (temp - sensor_e_trim)
					pub_intensity4.publish(intensity)
					print "light_callback3 for bot_4 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[4] = 1
		except:
			intensity_flag[3] = 0 # waste statement

def callback4(cmd):
	print "callback4"
	if cmd.linear.x == 1.0:
		exception_error = True
		while (exception_error == True):
			try:	
				serial_port.flushOutput()
				serial_port.write("qqqqq")
				time.sleep(1)
				temp_str = serial_port.readline()
				if temp_str != "":
					print "qqqqq ACK "+temp_str[0]
					if ord(temp_str[0]) == motor_e_acknowledge: # ASCII Value for x
						pub_front4.publish(motor_e_acknowledge)
						exception_error = False
						print "Exception Error BOT 4: "+ str(exception_error)
			except:
				exception_error = True # waste statement
	elif cmd.angular.z < 0.0:
				motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
				print "el"+motor_cmd_val.zfill(3)
				serial_port.flushOutput()		
				serial_port.write("el"+motor_cmd_val.zfill(3))
	elif cmd.angular.z>0.0:
				motor_cmd_val = str(min(max(motor_min_rotate,abs(int(cmd.angular.z*motor_max_scale))),motor_max_rotate))
				print "er"+motor_cmd_val.zfill(3)
				serial_port.flushOutput()
				serial_port.write("er"+motor_cmd_val.zfill(3))
		
def light_callback4(msg):
	print "light_callback4"
	intensity_flag[4] = 0
	while intensity_flag[4]==0:
		try:
			serial_port.flushOutput()
			serial_port.write("ggggg") # mmmmm not used as it is move_front command for Bot 0 
			time.sleep(1)
			temp_str = serial_port.readline()
			if temp_str != "":
				temp = int(temp_str)
				print "ggggg readline() "+str(temp)+"    /     "+temp_str
				intensity = temp

				if ((temp - sensor_a_trim)>=0) and ((temp - sensor_a_trim) <= sensor_max_reading_i):
					intensity=sensor_max_reading_i - (temp - sensor_a_trim)
					pub_intensity0.publish(intensity)
					print "light_callback4 for bot_0 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[0] = 1

				elif ((temp - sensor_b_trim)>=0) and ((temp - sensor_b_trim) <= sensor_max_reading_j):
					intensity=sensor_max_reading_j - (temp - sensor_b_trim)
					pub_intensity1.publish(intensity)
					print "light_callback4 for bot_1 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[1] = 1

				elif ((temp - sensor_c_trim)>=0) and ((temp - sensor_c_trim) <= sensor_max_reading_k):
					intensity=sensor_max_reading_k - (temp - sensor_c_trim)
					pub_intensity2.publish(intensity)
					print "light_callback4 for bot_2 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[2] = 1
					
				elif ((temp - sensor_d_trim)>=0) and ((temp - sensor_d_trim) <= sensor_max_reading_l):
					intensity=sensor_max_reading_l - (temp - sensor_d_trim)
					pub_intensity3.publish(intensity)
					print "light_callback4 for bot_3 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[3] = 1
					
				elif ((temp - sensor_e_trim)>=0) and ((temp - sensor_e_trim) <= sensor_max_reading_m):
					intensity=sensor_max_reading_m - (temp - sensor_e_trim)
					pub_intensity4.publish(intensity)
					print "light_callback4 for bot_4 " + str(intensity)# + "    " + str((temp - sensor_a_trim))
					intensity_flag[4] = 1
		except:
			intensity_flag[4] = 0 # waste statement

if __name__ == '__main__':

	serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout = 0)
	print "Serial Read"
	try:

		pub_intensity0 = rospy.Publisher('/nxt0/intensity', UInt16)
		pub_intensity1 = rospy.Publisher('/nxt1/intensity', UInt16)
		pub_intensity2 = rospy.Publisher('/nxt2/intensity', UInt16)
		pub_intensity3 = rospy.Publisher('/nxt3/intensity', UInt16)
		pub_intensity4 = rospy.Publisher('/nxt4/intensity', UInt16)

		pub_front0 = rospy.Publisher('/nxt0/front', UInt16)
		pub_front1 = rospy.Publisher('/nxt1/front', UInt16)
		pub_front2 = rospy.Publisher('/nxt2/front', UInt16)
		pub_front3 = rospy.Publisher('/nxt3/front', UInt16)
		pub_front4 = rospy.Publisher('/nxt4/front', UInt16)

		rospy.Subscriber("/nxt0/cmd_vel", Twist, callback0)
		rospy.Subscriber("/nxt0/find_intensity", Empty, light_callback0)
		print "Subscriber0"
		
		rospy.Subscriber("/nxt1/cmd_vel", Twist, callback1)
		rospy.Subscriber("/nxt1/find_intensity", Empty, light_callback1)
		print "Subscriber1"
		
		rospy.Subscriber("/nxt2/cmd_vel", Twist, callback2)
		rospy.Subscriber("/nxt2/find_intensity", Empty, light_callback2)
		print "Subscriber2"
		
		rospy.Subscriber("/nxt3/cmd_vel", Twist, callback3)
		rospy.Subscriber("/nxt3/find_intensity", Empty, light_callback3)
		print "Subscriber3"
		
		rospy.Subscriber("/nxt4/cmd_vel", Twist, callback4)
		rospy.Subscriber("/nxt4/find_intensity", Empty, light_callback4)
		print "Subscriber4"
		
		rospy.init_node('mrl_cmd_int_node', anonymous=True)
		
		r = rospy.Rate(10) # 20hz
		
		while not rospy.is_shutdown():	
#			light_callback()			
			r.sleep()
	except rospy.ROSInterruptException: 
		pass
