#!/usr/bin/env python

from irpos import *
from irp6_checkers.srv import *
import math
#from irp6_checkers.msg import *

class CheckersIRPOS(IRPOS):
	
	def __init__(self, nodeName, robotName, robotJointNumbers):
		IRPOS.__init__(self, nodeName, robotName, robotJointNumbers)
		
	def test(self):
		self.prepare_to_game()
		self.get_checker()
		self.touch_chessboard()
		self.put_checker()
		self.get_checker()
		self.drop_checker()
	
	def prepare_to_game(self):
		print "[CheckersIRPOS] Move to start position"
		IRPOS.move_to_joint_position(self, [0, -1.4791420483915523, -0.16173032244035423, 0.07007528019972864, 4.712388138719054, -1.5707949127454675], 4.00)
		IRPOS.move_to_cartesian_pose(self, 2.00, Pose(Point(0.9, 0, 1.30), Quaternion(0, 1, 0, 0)))
		IRPOS.tfg_to_joint_position(self, 0.08, 3.00)
	
	def get_checker(self):
		print "[CheckersIRPOS] Get checker action"
		#IRPOS.tfg_to_joint_position(self, 0.078, 10.00)
		IRPOS.move_rel_to_cartesian_pose_with_contact(self, 20.00, Pose(Point(0, 0, 0.40), Quaternion(0, 0, 0, 1)), Wrench(Vector3(0.0, 0.0, 5.0), Vector3(0.0, 0.0, 0.0)))
		IRPOS.move_rel_to_cartesian_pose(self, 1.00, Pose(Point(0, 0, -0.004), Quaternion(0, 0, 0, 1)))
		IRPOS.tfg_to_joint_position(self, 0.063, 3.00)
		IRPOS.move_rel_to_cartesian_pose(self, 2.00, Pose(Point(0, 0, -0.04), Quaternion(0, 0, 0, 1)))
	
	def touch_chessboard(self):
		print "[CheckersIRPOS] Touch chessboard action"
		IRPOS.move_rel_to_cartesian_pose_with_contact(self, 20.00, Pose(Point(0, 0, 0.40), Quaternion(0, 0, 0, 1)), Wrench(Vector3(0.0, 0.0, 5.0), Vector3(0.0, 0.0, 0.0)))
		IRPOS.move_rel_to_cartesian_pose(self, 2.00, Pose(Point(0, 0, -0.04), Quaternion(0, 0, 0, 1)))
	
	def put_checker(self):
		print "[CheckersIRPOS] Put checker action"
		IRPOS.move_rel_to_cartesian_pose_with_contact(self, 20.00, Pose(Point(0, 0, 0.40), Quaternion(0, 0, 0, 1)), Wrench(Vector3(0.0, 0.0, 5.0), Vector3(0.0, 0.0, 0.0)))
		IRPOS.tfg_to_joint_position(self, 0.08, 3.00)
		IRPOS.move_rel_to_cartesian_pose(self, 2.00, Pose(Point(0, 0, -0.04), Quaternion(0, 0, 0, 1)))
	
	def drop_checker(self):
		print "[CheckersIRPOS] Drop checker action"
		IRPOS.tfg_to_joint_position(self, 0.08, 3.00)
	
	def handle_control(self, req):
		print "[CheckersIRPOS] Control start"
		for c in req.Controls:
			print "[CheckersIRPOS] Move horizontal"
			time = math.sqrt(c.X**2 + c.Y**2)*20
			IRPOS.move_rel_to_cartesian_pose(self, time, Pose(Point(c.X, c.Y, 0), Quaternion(0, 0, 0, 1)))
			if c.Action == c.GET_CHECKER:
				self.get_checker()
			elif c.Action == c.PUT_CHECKER:
				self.put_checker()
			elif c.Action == c.DROP_CHECKER:
				self.drop_checker()
			elif c.Action == c.TOUCH_CHESSBOARD:
				self.touch_chessboard()
		self.prepare_to_game()
		print "[CheckersIRPOS] Control end"
		resp = ControlResponse()
		resp.Status = ControlResponse.SUCCESS
		return resp
	
	def control_server(self):
		server = rospy.Service('irp6_control', Control, self.handle_control)
		print "[CheckersIRPOS] Ready to control robot."
		rospy.spin()

if __name__ == '__main__':
	irpos = CheckersIRPOS("warcaby", "Irp6p", 6)
	#irpos.test()
	irpos.prepare_to_game()
	irpos.control_server()
	
