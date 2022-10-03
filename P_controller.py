#!/usr/bin/python
# -*- coding: utf-8 -*-

from cmath import atan
from numpy import arctan, arctan2, sqrt
from E160_state import *
from E160_robot import *
import math
import time


class P_controller:

	def __init__(self, robot, logging = True):
		self.robot = robot  # do not delete this line
  			#ASSUME VALUES FOR k_rho, k_ alpha, and k_beta.... Evaluate how these values effect the robot controls
		self.kp = 3  # k_rho
		self.ka = 8  # k_alpha
		self.kb = -1.5 # k_beta
		self.logging = logging
		if(logging == True):
			self.robot.make_headers(['pos_X','posY','posZ','vix','viy','wi','vr','wr', 'Dpos_X','DposY','DposZ','rho', 'alpha', 'beta', 'Destination Reached'])

		self.set_goal_points()

	#Edit goal point list below, if you click a point using mouse, the points programmed
	#will be washed out
	def set_goal_points(self):
		# here is the example of destination code
  		#------------Triangle----
		#self.robot.state_des.add_destination(x=-150,y=-100,theta=0)    	#goal point 1 TOP LEFT
		#self.robot.state_des.add_destination(x=0,y=100,theta=0)   		#goal point 2 BOT MIDDLE
		#self.robot.state_des.add_destination(x=150,y=-100,theta=0)   	#goal point 3 BOT RIGHT
		#------------square----
		self.robot.state_des.add_destination(x=160,y=0,theta=-math.pi/2)    	#goal point 1 TOP RIGHT  (theta in radians)
		self.robot.state_des.add_destination(x=0,y=-160,theta=-math.pi)   	#goal point 2 BOT RIGHT
		self.robot.state_des.add_destination(x=-160,y=0,theta=math.pi/2)   	#goal point 3 BOT LEFT
		self.robot.state_des.add_destination(x=0,y=160,theta=0)     	#goal point 4 TOP LEFT



	def track_point(self):
     	# REV 4.0 (w/ homogenious transform)
		#------------Main PController Code-------------------------------------------------------------------
 		#STEP 1:  begin by pulling current  position, velocity data 
			# All d_ means destination
		(d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  # get next destination configuration in the global frame
			# All c_ means current_
		(c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration in the global frame
		(c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
		(c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration


		#STEP 2:  Calculate robot pos in terms of Goal reference position 
  		#GLOBAL TO ROBOT TRANSFORM
		RobotH =np.array([[math.cos(c_theta),-math.sin(c_theta),0,c_posX],
  				[math.sin(c_theta),math.cos(c_theta),0,c_posY],
      			[0,0,1,0],
          		[0,0,0,1]])  #  Translation with x and y Rotate z-axis (delta) TxyRz
  		#GLOBAL TO GO TRANSFORM
		GoH =np.array([[math.cos(d_theta),-math.sin(d_theta),0,d_posX],
  				[math.sin(d_theta),math.cos(d_theta),0,d_posY],
      			[0,0,1,0],
          		[0,0,0,1]]  )  #  Translation with x and y Rotate z-axis (delta) TxyRz
		
 
  		#Globe frame TO GO frame  TRANSFORM
		invGoH = np.linalg.inv(GoH) 
		P_globe_robot= np.array([[c_posX],[c_posY],[0],[1]]) 		# Robot location with respects of robot global frame
		P_go_robot= invGoH.dot(P_globe_robot) 						# Robot location with respects to go frame
		dx_go_robot= P_go_robot[0][0]						 #  First element of first row
		dY_go_robot= P_go_robot[1][0]						 #  First element of second row
    
		#Globe frame TO Robot frame  TRANSFORM
		invRobotH = np.linalg.inv(RobotH) 
		P_globe_go= np.array([[d_posX],[d_posY],[0],[1]]) 		# Robot location with respects of robot global frame
		P_robot_go= invRobotH.dot(P_globe_go) 				# Robot location with respects to go frame
		dx_robot_go= P_robot_go[0][0]						 #  First element of first row
		dY_robot_go= P_robot_go[1][0]						 #  First element of second row
  

  #STEP 3:  Calculate the rho, alpha, and beta of the system at the current instance
		rho= math.sqrt((dx_robot_go)**2+(dY_robot_go)**2) 		# distance between robot's current location and the desired location---- in robot frame
		alpha= math.atan2((dY_robot_go),(dx_robot_go))         	# angle between robot's XR axis and the rho vector---- in robot frame
		beta= -(d_theta-c_theta)- alpha      	# angle between rho vector and the Xg axis---- in robot frame

 	
  		#STEP 4:  Regulate the rho, alpha, and beta of the system at the current instance. limit within -/+ pi/2
		if alpha > math.pi:
			alpha =  math.pi		
		if alpha < - math.pi:
			alpha =  - math.pi
		if beta > math.pi:
			beta =  math.pi
		if beta < - math.pi:
			beta =  - math.pi

		#STEP 5: # Propotional Controller for local linear and angular velocity
		c_v = self.kp*rho  #randomly assigned c_v and c_w for demonstration purpose
		c_w = self.ka*alpha-self.kb*beta  

		if c_v > 16:
			c_v = 16
		if c_w > 1:
                        c_w = 1
		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
		
		# you need to write code to find the wheel speed for your c_v, and c_w, the program won't calculate it for you.
		phi_l = round(c_v/3-c_w*12/3,2)
		phi_r = round(2*c_v/3-phi_l,2)

		self.robot.send_wheel_speed(phi_l,phi_r) #unit rad/sw
		
  		#STEP 6: # reach way point criteria
		#you need to modify the reach way point criteria
		if  abs(d_theta-c_theta) < 2 and abs(rho) < 1 : #you need to modify the reach way point criteria
			self.robot_destination_reached = True
		else:
			self.robot_destination_reached = False

		#--------------------------------------------------------------------------------------------------

		# use the following to log the variables, use [] to bracket all variables you want to store
		# stored values are in log folder
		if self.logging == True:
			self.robot.log_data([c_posX,c_posY,c_theta,c_vix,c_viy,c_wi,c_v,c_w, d_posX, d_posY, d_theta, rho, alpha, beta, self.robot_destination_reached])

		if (self.robot_destination_reached) == True: #you need to modify the reach way point criteria
			if(self.robot.state_des.reach_destination()): 
				print("final goal reached")
				self.robot.set_motor_control(.0, .0)  # stop the motor
				return True
			else:
				print("one goal point reached, continute to next goal point")
		return False
