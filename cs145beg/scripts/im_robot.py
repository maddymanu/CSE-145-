#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import Empty 
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist 
from PySide import QtCore, QtGui

custom_cont = None




class DroneStatus(object):
	Emergency = 0
	Inited    = 1
	Landed    = 2
	Flying    = 3
	Hovering  = 4
	Test      = 5
	TakingOff = 6
	GotoHover = 7
	Landing   = 8
	Looping   = 9

class BasicDronecustom_cont(object):
	def __init__(self):
		# Holds the current drone status
		self.status = 2
		print "Initializing Drone custom_cont Basic"
		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Allow the custom_cont to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		self.pubFlatTrim = rospy.Publisher('/ardrone/flattrim', Empty)
		
		# Allow the custom_cont to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state

	def SendFlatTrim(self):
		if(self.status == DroneStatus.Landed):
			self.pubFlatTrim.publish(Empty())
			
	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			print "sending takeoff"
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0, hover=False):
		# Called by the main program to set the current command
		if hover:
			self.command.linear.x  = 0
			self.command.linear.y  = 0
			self.command.linear.z  = 0
			self.command.angular.z = 0
			self.command.angular.y = 0
			self.command.angular.x = 0	
		else:
			if z_velocity > 0:
				print "ZVEL > 00000000000000000000000000000000000000000000000000000000000000000000000000"
			self.command.linear.x  = pitch
			self.command.linear.y  = roll
			self.command.linear.z  = 0
			self.command.angular.z = yaw_velocity

	def SendCommand(self,event):

		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			#print "custom_cont - publising new command" , self.command
			self.pubCommand.publish(self.command)







def parseData(data):
	global tags_xc, tags_yc, tags_width, tags_height, tags_distance, tags_count, tag_location_boolean, found_bottom_tag
	tags_count = data.tags_count
	tag_location_boolean = False
	found_bottom_tag = False
	#print data.tags_type
	for i in range(int(tags_count)):
		if(data.tags_type[i] == 0):

			tag_location_boolean = True
			tags_xc = float(data.tags_xc[i])
			tags_yc = float(data.tags_yc[i])
			tags_width = float(data.tags_width[i])
			tags_height = float(data.tags_height[i])
			tags_distance = data.tags_distance[i]
			tags_count = int(data.tags_count)

			tags_distance = tags_distance / 100
			tags_distance = DISTANCE_CORRECTION_FACTOR * float(tags_distance)




			



def MIDPOINTTAGG():

	move_yaw_variance = 0
	move_follow_variance = 0



	if tag_location_boolean == True:
		if(tags_xc > (MIDPOINT_PIXEL + MIDPOINT_DEADBAND) or tags_xc < (MIDPOINT_PIXEL - MIDPOINT_DEADBAND)):
			move_yaw_variance = pidcustom_cont.update(tags_xc) / 2500
			print "New Yay Command" , move_yaw_variance
		else:
			custom_cont.SetCommand(hover=True)
				


		if(tags_distance > 5):
			move_follow_variance = -1 * (followPIDcustom_cont.update(tags_distance) / 2100)
			print "FRONT FOLLOW" , move_follow_variance
		else:
			custom_cont.SetCommand(hover=True)
		'''if(tags_distance < 3):	
			move_follow_variance = 1 * (followPIDcustom_cont.update(tags_distance) / 2100)
			#print "BACK FOLLOW" , move_follow_variance'''
				
				
		
					
		if(move_follow_variance == 0 and move_yaw_variance == 0):
			custom_cont.SetCommand(hover=True)
		elif tag_location_boolean == True:
			print "SENDING YAW / FOLLOW"
			custom_cont.SetCommand(pitch=move_follow_variance, yaw_velocity=move_yaw_variance)
	else:
		#print "TAG NOT FOUND ------- HOVER"
		custom_cont.SetCommand(hover=True)
	
	
		

def ReceiveData(data):

	parseData(data)
	MIDPOINTTAGG()



if __name__ == '__main__':
	rospy.init_node("im_robot")
	app = QtGui.QApplication("sss")
	rospy.init_node("im_robot")

	navdata_sub = rospy.Subscriber('/ardrone/navdata' , Navdata , ReceiveData)

	custom_cont = BasicDronecustom_cont()
	display = Keyboardcustom_cont()
	display.show()


	pidcustom_cont.setPoint(MIDPOINT_PIXEL)
	followPIDcustom_cont.setPoint(DISTANCE)

	sys.exit(app.exec_())
	
