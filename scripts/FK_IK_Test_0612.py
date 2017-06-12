
from serial.tools import list_ports
# Import uarm for python library
# import uarm_python
import pyuarm
from time import sleep

def get_uarm_ports():
	ports = []
	for i in list_ports.comports():
		if i[2] == "USB VID:PID=0403:6001 SNR=AI04HYB1":
			ports.append(i[0])
	return ports

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
	while loop_flag:
		print "New test loop"
		sleep(5)		
		pumpControl(1)
		sleep(5)
		pumpControl(0)

if __name__ == '__main__':
	try:
		connectFunc()
		sleep(1)
		function_Test()
	except:
		print 'ERROR!'
		pass
	finally:
		print 'Program Stopped'
		pass