#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from std_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import Empty 
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist 
from PySide import QtCore, QtGui
from PID_lib import PID
value = None



image_xc = 0
image_yc = 0
image_width = 0
image_height = 0
image_distance = 0
image_count = 0

MIDDLE_PIXEL = 480

MIDDLE_DEADBAND = 30

DISTANCE = 3

DISTANCE_DEADBAND = 1



value_PERIOD = 100
done_total = False



detcted_front_image_n = False
detcted_bottom_image_n = False



STATE_MIDDLEING = True
STATE_FLYING = False
STATE_LAST_SPINNING = 1


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

class BasicDronevalue(object):
	def __init__(self):

		self.status = 2
		print "Initializing Drone value Basic"

		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		

		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		self.pubFlatTrim = rospy.Publisher('/ardrone/flattrim', Empty)
		

		self.pubvalue = rospy.Publisher('/cmd_vel',Twist)


		self.value = Twist()
		self.valueTimer = rospy.Timer(rospy.Duration(value_PERIOD/1000.0),self.Sendvalue)


		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		
		self.status = navdata.state

	def SendFlatTrim(self):
		if(self.status == DroneStatus.Landed):
			self.pubFlatTrim.publish(Empty())
			
	def SendTakeoff(self):
		
		if(self.status == DroneStatus.Landed):
			print "sending takeoff"
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		
		self.pubLand.publish(Empty())

	def SendEmergency(self):

		self.pubReset.publish(Empty())

	def Setvalue(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0, hover=False):
		
		if hover:
			self.value.linear.x  = 0
			self.value.linear.y  = 0
			self.value.linear.z  = 0
			self.value.angular.z = 0
			self.value.angular.y = 0
			self.value.angular.x = 0	
		else:
			
			self.value.linear.x  = pitch
			self.value.linear.y  = roll
			self.value.linear.z  = z_velocity
			self.value.angular.z = yaw_velocity

	def Sendvalue(self,event):


		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			#print "value - publising new value" , self.value
			#print self.value
			self.pubvalue.publish(self.value)




class Keyboardvalue(QtGui.QMainWindow):
	def __init__(self):
		
		super(Keyboardvalue,self).__init__()
		self.setGeometry(200,500,400,400)
		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)


		self.setCentralWidget(self.imageBox)

		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0
		print "calling show within init"
		self.show()
		
		
	def keyPressEvent(self, event):
		global STATE_MIDDLEING, STATE_FLYING, pidvalue, pidAdjustType,reached_goal_target
		key = event.key()
		
		if(key == 84): 
			print "Takeoff"

			value.SendFlatTrim()
			rospy.sleep(.5)
			value.SendTakeoff()
			value.Setvalue(hover=True)


			STATE_FLYING = True
		elif(key == 76): 
			print "Land"
			value.SendLand()







def gotData(data):
	global image_xc, image_yc, image_width, image_height, image_distance, image_count, detcted_front_image_n, detcted_bottom_image_n
	image_count = data.image_count
	detcted_front_image_n = False
	detcted_bottom_image_n = False
	image_distance = 0


	for i in range(int(image_count)):
		#orange blue orange tag only
		if(data.image_type[i] == 0):

			detcted_front_image_n = True
			image_xc = float(data.image_xc[i])
			image_yc = float(data.image_yc[i])
			image_width = float(data.image_width[i])
			image_height = float(data.image_height[i])
			image_distance = data.image_distance[i]
			image_count = int(data.image_count)

			image_distance = image_distance / 100
			
			image_distance = DISTANCE_CORRECTION_FACTOR * float(image_distance)
			print image_distance , image_distance




			



def MIDDLEOnimage_nPID():
	global detcted_front_image_n , image_distance

	yaw_value = 0
	follow_value = 0
	z_value = 0



	if detcted_front_image_n == True:
		if(image_xc > (MIDDLE_PIXEL + MIDDLE_DEADBAND) or image_xc < (MIDDLE_PIXEL - MIDDLE_DEADBAND)):
			yaw_value = pidvalue.get_new_val(image_xc) / 2500
			#print "New Yay value" , yaw_value
		'''else:
			value.Setvalue(hover=True)'''

		print image_yc
		if(image_yc < 250 or image_yc > 550):
			z_value = pidvalue2.get_new_val(image_yc) / 2500
			print "New ZZZZZZZZZZZZZ value" , z_value , image_yc
		'''else:
			value.Setvalue(hover=True)'''
				


		if(image_distance > 6 or (image_distance < 3)):
			follow_value = -1 * (followPIDvalue.get_new_val(image_distance) / 2700)
			print "FRONT FOLLOW" , follow_value
			#image_distance = 0
		#if image_dis
		'''else:
			value.Setvalue(hover=True)'''
		'''if(image_distance < 3):	
			follow_value = 1 * (followPIDvalue.get_new_val(image_distance) / 2100)
			#print "BACK FOLLOW" , follow_value'''
				
				
		
					
		if(follow_value == 0 and yaw_value == 0 and z_value==0):
			value.Setvalue(hover=True)
		elif detcted_front_image_n == True:

			value.Setvalue(pitch=follow_value, yaw_velocity=yaw_value , z_velocity=z_value)
	else:

		value.Setvalue(hover=True)
	
	
		

def gotNData(data):

	gotData(data)
	MIDDLEOnimage_nPID()



if __name__ == '__main__':
	rospy.init_node("im_robot")
	app = QtGui.QApplication("sss")


	rospy.init_node("im_robot")

	navdata_sub = rospy.Subscriber('/ardrone/navdata' , Navdata , gotNData)

	value = BasicDronevalue()
	display = Keyboardvalue()
	display.show()


	pidvalue = PID(0.3, 0.5, 0.2)
	pidvalue2 = PID(0.3, 0.5, 0.2)

	followPIDvalue = PID(3, 0.5, 4)

	pidvalue.setPoint(MIDDLE_PIXEL)
	pidvalue2.setPoint(MIDDLE_PIXEL)
	followPIDvalue.setPoint(DISTANCE)

	sys.exit(app.exec_())
	
