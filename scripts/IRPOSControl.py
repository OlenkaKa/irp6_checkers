#!/usr/bin/env python

import sys
sys.path.append('/home/akarbarc/ws_irp6/underlay/src/irp6_robot/irp6_bringup/scripts/IRPOS')

from IRPOS import *
from irp6_checkers.srv import *

class CheckersIRPOS(IRPOS):
	
	def __init__(self, nodeName, robotName, robotJointNumbers):
		IRPOS.__init__(self, nodeName, robotName, robotJointNumbers)
	
	def prepare_to_game(self):
		IRPOS.move_to_joint_position(self, [0, -1.4791420483915523, -0.16173032244035423, 0.07007528019972864, 4.712388138719054, -1.5707949127454675], 6.00)
		IRPOS.move_to_cartesian_pose(self, 3.00, Pose(Point(0.9, 0, 1.25), Quaternion(0, 1, 0, 0)))
	
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
	irpos = CheckersIRPOS("IRpOS", "Irp6p", 6)
	#irpos = Temp()
	irpos.prepare_to_game()
	print "[IRPOSControl] IRPOSControl init."
	irpos.control_server()
	
