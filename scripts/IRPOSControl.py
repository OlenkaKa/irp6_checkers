#!/usr/bin/env python

import sys
sys.path.append('/home/akarbarc/ws_irp6/underlay/src/irp6_robot/irp6_bringup/scripts/IRPOS')

from IRPOS import *
from irp6_checkers.srv import *

class CheckersIRPOS(IRPOS):
	
	def __init__(self, nodeName, robotName, robotJointNumbers):
		IRPOS.__init__(self, nodeName, robotName, robotJointNumbers)
	
	def handle_control(self, req):
		print "[IRPOSControl] New control received."
		print req.Controls[0].X
		print req.Controls[0].Y
		print req.Controls[0].Action
		print "[IRPOSControl] Returning status."
		resp = ControlResponse()
		resp.Status = ControlResponse.SUCCESS
		return resp
	
	def control_server(self):
		server = rospy.Service('irp6_control', Control, self.handle_control)
		print "[IRPOSControl] Ready to control robot."
		rospy.spin()


# jeśli nie pracujemy na robocie, bo wtedy CheckersIRPOS zawiesza się na konstruktorze
class Temp:
	
	def __init__(self):
		rospy.init_node('irp6_control_server')
	
	def handle_control(self, req):
		print "[IRPOSControl] New control received."
		print req.Controls[0].X
		print req.Controls[0].Y
		print req.Controls[0].Action
		print "[IRPOSControl] Returning status."
		resp = ControlResponse()
		resp.Status = ControlResponse.SUCCESS
		return resp
	
	def control_server(self):
		server = rospy.Service('irp6_control', Control, self.handle_control)
		print "[IRPOSControl] Ready to control robot."
		rospy.spin()



if __name__ == '__main__':
	#irpos = CheckersIRPOS("IRpOS", "Irp6p", 6)
	irpos = Temp()
	print "[IRPOSControl] IRPOSControl init."
	irpos.control_server()
	
