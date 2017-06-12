
import sys
import math
from time import sleep
from serial.tools import list_ports
import rospy
import pyuarm

# Import messages type
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import Int32
from uarm.msg import Angles
from uarm.msg import Coords
from uarm.msg import CoordsWithTime
from uarm.msg import CoordsWithTS4


# UARM SPECIFICATIONS
MATH_PI	= 3.141592653589793238463
MATH_TRANS  = 57.2958
MATH_L1	= (10.645+0.6)
MATH_L2	= 2.117
MATH_L3	= 14.825
MATH_L4	= 16.02
MATH_L43 = MATH_L4/MATH_L3
MATH_SUCKER_L = 6
# UARM OFFSETS
TopOffset = -1.5
BottomOffset = 1.5


def get_uarm_ports():
	ports = []
	for i in list_ports.comports():
		if i[2] == "USB VID:PID=0403:6001 SNR=AI04HYB1":
			ports.append(i[0])
	return ports

def connectFunc():
	global loop_flag
	global m_Uarm
	ports = get_uarm_ports()
    # print("Here!!!!")
	if len(ports)>0:
		m_Uarm = pyuarm.get_uarm()
		loop_flag = True
		print 'uArm-Metal Connected!'
	else:
		printf("There is no uArm port available",ERROR)
		loop_flag = False
		return None

def function_Test():
	start_pnt = {}
	start_pnt[1] = 90
	start_pnt[2] = 90
	start_pnt[3] = 60

	pick_pnt1 = {}
	pick_pnt1[1] = 90
	pick_pnt1[2] = 55
	pick_pnt1[3] = 85
	#Pump on
	lift_pnt1 = {}
	lift_pnt1[1] = 90
	lift_pnt1[2] = 90
	lift_pnt1[3] = 60

	lift_pnt2 = {}
	lift_pnt2[1] = 155
	lift_pnt2[2] = 90
	lift_pnt2[3] = 60

	lift_pnt3 = {}
	lift_pnt3[1] = 155
	lift_pnt3[2] = 55
	lift_pnt3[3] = 80
	#Pump off
	hang_pnt1 = {}
	hang_pnt1[1] = 155
	hang_pnt1[2] = 90
	hang_pnt1[3] = 60
	#Sleep
	
	#Pump on
	pick_pnt2 = {}
	pick_pnt2[1] = 155
	pick_pnt2[2] = 55
	pick_pnt2[3] = 85

	lift_pnt4 = {}
	lift_pnt4[1] = 155
	lift_pnt4[2] = 90
	lift_pnt4[3] = 60

	lift_pnt5 = {}
	lift_pnt5[1] = 90
	lift_pnt5[2] = 90
	lift_pnt5[3] = 60

	lift_pnt6 = {}
	lift_pnt6[1] = 90
	lift_pnt6[2] = 55
	lift_pnt6[3] = 80
	#Pump off
	#Sleep

	while loop_flag:
		print "New test loop"
		writeServoAngle(start_pnt)
		sleep(3)
		writeServoAngle(pick_pnt1)
		pumpControl(1)
		sleep(1)
		writeServoAngle(lift_pnt1)
		sleep(1)
		writeServoAngle(lift_pnt2)
		sleep(1)
		writeServoAngle(lift_pnt3)
		pumpControl(0)
		sleep(1)
		writeServoAngle(hang_pnt1)
		sleep(5)
		pumpControl(1)
		writeServoAngle(pick_pnt2)
		sleep(1)
		writeServoAngle(lift_pnt4)
		sleep(1)
		writeServoAngle(lift_pnt5)
		sleep(1)
		writeServoAngle(lift_pnt6)
		sleep(1)
		pumpControl(0)

## Callbacks
# angles control function once received data from topic
def writeServoAngle(target_angles):
	m_Uarm.set_servo_angle(0, target_angles[1])
	m_Uarm.set_servo_angle(1, target_angles[2])
	m_Uarm.set_servo_angle(2, target_angles[3])
	m_Uarm.set_servo_angle(3, 90)
	print 'Movement: Moved Once'

# pump control function once received data from topic
def pumpControl(data):
	data_input = data
	if data_input == 0:
		m_Uarm.set_pump(0)
		print 'Pump: Off'
	elif data_input == 1:
		m_Uarm.set_pump(1)
		print 'Pump: On'
	else:
		pass

## ROS Listener
def listener():
	print 'Initiating ROS node: m_FK_IK_Test'	
	# rospy.init_node('m_FK_IK_Test',anonymous=True)
	# rospy.Subscriber("uarm_status",String, attchCallback)
	# rospy.Subscriber("pump_control",UInt8, pumpCallack)
	# rospy.Subscriber("pump_str_control",String, pumpStrCallack)
	# rospy.Subscriber("read_coords",Int32, currentCoordsCallback)
	# rospy.Subscriber("read_angles",Int32, readAnglesCallback)
	# rospy.Subscriber("stopper_status",Int32, stopperStatusCallback)	
	# rospy.Subscriber("write_angles",Angles, writeAnglesCallback)
	# rospy.Subscriber("move_to",Coords, moveToCallback)
	# rospy.Subscriber("move_to_time",CoordsWithTime, moveToTimeCallback)
	# rospy.Subscriber("move_to_time_s4",CoordsWithTS4, moveToTimeAndS4Callback)
	# rospy.spinonce()
	pass


## Kinematics
def FK_func(theta_1,theta_2,theta_3):
	coord = {}
	g_l3_1 = MATH_L3 * math.cos(theta_2/MATH_TRANS)
	g_l4_1 = MATH_L4 * math.cos(theta_3 / MATH_TRANS);
	g_l5 = (MATH_L2 + g_l3_1 + g_l4_1)
	## Using right hand coordinate sys
	coord[1] = math.sin(abs(theta_1 / MATH_TRANS))*g_l5;
	coord[2] = math.cos(abs(theta_1 / MATH_TRANS))*g_l5;	
	coord[3] = MATH_L1 + MATH_L3*math.sin(abs(theta_2 / MATH_TRANS)) - MATH_L4*math.sin(abs(theta_3 / MATH_TRANS))- MATH_SUCKER_L;
	print "Calling FK with following joint Values:"
	print theta_1
	print theta_2
	print theta_3
	print "Setting to x: %2.2f y: %2.2f z: %2.2f." %(coord[1],coord[2],coord[3])
	# print coord[1]
	# print coord[1]
	# print coord[1]
	return coord

def IK_func(x, y, z):
		if z > (MATH_L1 + MATH_L3 + TopOffset):
			z = MATH_L1 + MATH_L3 + TopOffset
		# if z < (MATH_L1 - MATH_L4 + BottomOffset):
		# 	z = MATH_L1 - MATH_L4 + BottomOffset
		if z < 0:
			z = 0
		g_y_in = (-y-MATH_L2)/MATH_L3
		g_z_in = (z - MATH_L1) / MATH_L3
		g_right_all = (1 - g_y_in*g_y_in - g_z_in*g_z_in - MATH_L43*MATH_L43) / (2 * MATH_L43)
		g_sqrt_z_y = math.sqrt(g_z_in*g_z_in + g_y_in*g_y_in)

		if x == 0:
			# Calculate value of theta 1
			g_theta_1 = 90;
			# Calculate value of theta 3
			if g_z_in == 0:
				g_phi = 90
			else:
				g_phi = math.atan(-g_y_in / g_z_in)*MATH_TRANS
			if g_phi > 0:
				g_phi = g_phi - 180
	    		g_theta_3 = math.asin(g_right_all / g_sqrt_z_y)*MATH_TRANS - g_phi

	    		if g_theta_3<0:
				g_theta_3 = 0
			# Calculate value of theta 2
	    		g_theta_2 = math.asin((z + MATH_L4*math.sin(g_theta_3 / MATH_TRANS) - MATH_L1) / MATH_L3)*MATH_TRANS
		else:
			# Calculate value of theta 1
			g_theta_1 = math.atan(y / x)*MATH_TRANS
			if (y/x) > 0:
				g_theta_1 = g_theta_1
			if (y/x) < 0:
				g_theta_1 = g_theta_1 + 180
			if y == 0:
				if x > 0:
					g_theta_1 = 180
				else: 
					g_theta_1 = 0
			# Calculate value of theta 3
			g_x_in = (-x / math.cos(g_theta_1 / MATH_TRANS) - MATH_L2) / MATH_L3;
			if g_z_in == 0:  
				g_phi = 90
			else:
				g_phi = math.atan(-g_x_in / g_z_in)*MATH_TRANS
			if g_phi > 0:
				g_phi = g_phi - 180 

			g_sqrt_z_x = math.sqrt(g_z_in*g_z_in + g_x_in*g_x_in)

			g_right_all_2 = -1 * (g_z_in*g_z_in + g_x_in*g_x_in + MATH_L43*MATH_L43 - 1) / (2 * MATH_L43)
			g_theta_3 = math.asin(g_right_all_2 / g_sqrt_z_x)*MATH_TRANS
			g_theta_3 = g_theta_3 - g_phi

			if g_theta_3 <0 :
				g_theta_3 = 0
			# Calculate value of theta 2
			g_theta_2 = math.asin(g_z_in + MATH_L43*math.sin(abs(g_theta_3 / MATH_TRANS)))*MATH_TRANS

		g_theta_1 = abs(g_theta_1);
		g_theta_2 = abs(g_theta_2);

		# if g_theta_3 < 0 :
		# 	pass
		# else:
		# 	self.fwdKine(g_theta_1,g_theta_2, g_theta_3)
		# 	if (self.coord[2]>y+0.1) or (self.coord[2]<y-0.1):
		# 		g_theta_2 = 180 - g_theta_2

		# if(math.isnan(g_theta_1) or math.isinf(g_theta_1)):
		# 	g_theta_1 = self.readAngle(1)
		# if(math.isnan(g_theta_2) or math.isinf(g_theta_2)):
		# 	g_theta_2 = self.readAngle(2)
		# if(math.isnan(g_theta_3) or math.isinf(g_theta_3) or (g_theta_3<0)):
		# 	g_theta_3 = self.readAngle(3)
		
		# self.angle[1] = g_theta_1
		# self.angle[2] = g_theta_2
		# self.angle[3] = g_theta_3
		# return self.angle
		angle = {}
		angle[1] = g_theta_1
		angle[2] = g_theta_2
		angle[3] = g_theta_3
		return angle
		pass

if __name__ == '__main__':
	# try:
	# 	## Connecting Uarm
	# 	connectFunc()
	# 	sleep(1)
	# 	## Start ROS node
	# 	# listener()
	# 	# angles = IK_func(0,12,10)
	# 	# print angles
	# 	## Testing Functions
	# 	function_Test()
	# except:
	# 	print 'ERROR!'
	# 	pass
	# finally:
	# 	print 'Program Stopped'
	# 	pass
		# Connecting Uarm

	connectFunc()
	sleep(1)
	function_Test()
	## Start ROS node
	# listener()
	# angles = IK_func(0,12,10)
	# print angles
	# coords = FK_func(90,90,0)
	# print coords

	## Testing Functions
	# function_Test()