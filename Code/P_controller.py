#!/usr/bin/python
# -*- coding: utf-8 -*-

from E160_state import *
from E160_robot import *
import math
import time


class P_controller:

	def __init__(self, robot, logging = True):
		self.robot = robot  # do not delete this line
		self.kp = 3  # k_rho (5) (1) (4)
		self.ka = 8  # k_alpha(12) (9)(-1)
		self.kb = -1.5  # k_beta
		self.finish = 1
		self.logging = logging
		if(logging == True):
			self.robot.make_headers(['pos_X','posY','posZ','vix','viy','wi','vr','wr'])
		self.set_goal_points()
  
		
  
	#Edit goal point list below, if you click a point using mouse, the points programmed
	#will be washed out
	def set_goal_points(self):
		# here is the example of destination code
  		#------------Triangle----
		#self.robot.state_des.add_destination(x=-150,y=-100,theta=0)    	#goal point 1 TOP LEFT
		#self.robot.state_des.add_destination(x=0,y=100,theta=0)   		#goal point 2 BOT MIDDLE
		#self.robot.state_des.add_destination(x=150,y=-100,theta=0)   	#goal point 3 BOT RIGHT
		print("")



	def track_point(self):
     	# REV 5.0 (w/ homogenious transform)
		#  Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos

		#------------Main PController Code-------------------------------------------------------------------

		#STEP 1:  begin by pulling current  position, velocity data 
		# All d_ means destination
		(d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  # get next destination configuration
		# All c_ means current_
		(c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
		(c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
		(c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration
		deltaX = (d_posX-c_posX)
		deltaY = (d_posY-c_posY)
		deltaA = (d_theta-c_theta)


 		 #STEP 2:  Calculate the rho, alpha, and beta of the system at the current instance
		# Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos
		rho=math.sqrt((deltaX)**2+(deltaY)**2)
		omega=math.atan2(deltaY,deltaX)
		alpha=omega-c_theta
		'''''

  		#STEP 3: # Fixed Steering Kinematics  
		self.cw_limit = 50
		self.c_v_limit = 50
		self.wheelv_limit = 16
		# #STEP 3A:  Set Beta
		# Normalize Angle : Ensure that angles are between -pi and pi
		if alpha < -math.pi: alpha = alpha + 2*math.pi 
		if alpha > math.pi: alpha = alpha - 2*math.pi 
		# Set Beta
		beta =  -(c_theta - d_theta) - alpha
		if beta < -math.pi:
			beta = beta + 2*math.pi 
			#beta = -2*math.pi - beta
		if beta > math.pi: beta = beta - 2*math.pi 
		#STEP 3B: Robot Linear/Anglular Velocity
		# pcontroller desired robot velocity
		c_v = self.kp*rho
        # pcontroller desired velocity
		c_w = self.ka*alpha + self.kb*beta
        # Limit Robot Linear velocity
		if c_v > self.c_v_limit: c_v = self.c_v_limit
		if c_v < -self.c_v_limit: c_v = -self.c_v_limit
		# Limit Robot angular velocity
		if c_w > self.cw_limit: c_w = self.cw_limit
		if c_w < -self.cw_limit: c_w = -self.cw_limit

		#STEP 3C: # WHEEL SPEED Kinematics 
		r= 3 # wheel diameter
		L = 12 ##24
		phi_l =round( (1/r)*c_v +(L/r)*c_w ,2 )  # Wheel Anglar Velocity
		phi_r =round( (1/r)*c_v -(L/r)*c_w ,2 )  # Wheel Anglar Velocity
		# Limit Wheel Angular Velocity 
		if phi_l > self.wheelv_limit: phi_l =self.wheelv_limit
		if phi_l < -self.wheelv_limit: phi_l = -self.wheelv_limit
		if phi_r > self.wheelv_limit: phi_r = self.wheelv_limit
		if phi_r < -self.wheelv_limit: phi_r = -self.wheelv_limit
		# Final robot velocity -- pcontroller + wheel kinematics
		c_v = ((phi_r*r)/2) +((phi_l*r)/2)
        # Final robot angular velocity  --  pcontroller + wheel kinematics
		c_w = -((phi_r*r)/(2*L))+((phi_l*r)/(2*L))
  		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
		self.robot.send_wheel_speed(phi_l,phi_r) #unit rad/sw
		'''''		


  		#STEP 4: # Truck Kinematics   .
		#STEP 4A: # Set Known Values 
		L = 8	# robot body width is 2L = 16 cm				(Car WIDTH)
		d = 20	# distance between front and rear axles is 20 cm ( Car LENGTH)
		r = 3	# wheel radius is 3 cm 
		steering_angle_outer_limit = 40 # steering angle limit
		steering_angle_outer_limit = 40 # steering angle limit
		self.wheelv_limit = 20 # limit wheel speed to 20 rad/s 
		self.cw_limit = 50
		self.c_v_limit = 50
		self.wheelv_limit = 16
		#STEP 3B: P CONTROLLER DESIRED Robot Linear/Anglular Velocity
		# Normalize Angle : Ensure that angles are between -pi and pi
		if alpha < -math.pi: alpha = alpha + 2*math.pi 
		if alpha > math.pi: alpha = alpha - 2*math.pi 
		beta =  -(c_theta - d_theta) - alpha		# Set P controller Beta
		if beta < -math.pi:
			beta = beta + 2*math.pi 
			#beta = -2*math.pi - beta
		if beta > math.pi: beta = beta - 2*math.pi 
		# pcontroller desired linear velocity
		c_v = self.kp*rho
        # pcontroller desired angular velocity
		c_w = self.ka*alpha + self.kb*beta
        # Limit Robot Linear velocity
		if c_v > self.c_v_limit: c_v = self.c_v_limit
		if c_v < -self.c_v_limit: c_v = -self.c_v_limit
		# Limit Robot angular velocity
		if c_w > self.cw_limit: c_w = self.cw_limit
		if c_w < -self.cw_limit: c_w = -self.cw_limit
		
		#STEP 3C: # Ackerman_steering Kinematics
		# Ackerman_steering (middle steering angle)
		steering_angle_ideal=math.atan((c_w*d)/c_v)		

		if abs(steering_angle_ideal) >= 0.1:
			radius_of_rotation = d / math.atan(steering_angle_ideal)	
			# inner wheel steering angle)
			steering_angle_inner=math.atan((2*d*math.sin(steering_angle_ideal))/(2*d+math.cos(steering_angle_ideal) - 2*L*math.sin(steering_angle_ideal)))
			# outer wheel steering angle)
			steering_angle_outer=math.atan((2*d*math.sin(steering_angle_ideal))/(2*d*math.cos(steering_angle_ideal) + 2*L*math.sin(steering_angle_ideal)))		
			# Limit outer wheel steering angle 
			if steering_angle_outer > steering_angle_outer_limit: steering_angle_outer = steering_angle_outer_limit
			if steering_angle_outer < -steering_angle_outer_limit: steering_angle_outer = -steering_angle_outer_limit
			#STEP 3C: # WHEEL SPEED Kinematics
			phi_l =round( (1/r)*c_v - (c_v/d )* math.atan(steering_angle_ideal) ,2 )  # Wheel Anglar Velocity Angular from linear velocity and turining radius
			phi_r =round( (1/r)*c_v + (c_v/d )*math.atan(steering_angle_ideal) ,2 )  # Wheel Anglar Velocity
			# Limit Wheel Angular Velocity # wheels can only go forward
			if phi_l > self.wheelv_limit: phi_l =self.wheelv_limit
			if phi_l < -self.wheelv_limit: phi_l = 0
			if phi_r > self.wheelv_limit: phi_r = self.wheelv_limit
			if phi_r < -self.wheelv_limit: phi_r = 0
			# Final robot velocity -- pcontroller + wheel kinematics
			ackerman_c_v = (((phi_r*r)/2) +((phi_l*r)/2) )
			radius_of_rotation = d / math.atan(steering_angle_ideal)	

			# Final robot angular velocity  --  pcontroller + wheel kinematics
			ackerman_c_w= ackerman_c_v/radius_of_rotation
			# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
			self.robot.set_motor_control(ackerman_c_v, ackerman_c_w)  # use this command to set robot's speed in local frame
			self.robot.send_wheel_speed(phi_l,phi_r) #unit rad/sw   
   
		else:
			radius_of_rotation = 0
			# inner wheel steering angle)
			steering_angle_inner = 0
			# outer wheel steering angle)
			steering_angle_outer = 0	
			#STEP 3C: # WHEEL SPEED Kinematics
			phi_l =round((1/r)*c_v)    # Wheel Anglar Velocity Angular from linear velocity and turining radius
			phi_r =round( (1/r)*c_v)   # Wheel Anglar Velocity
			# Limit Wheel Angular Velocity # wheels can only go forward
			if phi_l > self.wheelv_limit: phi_l =self.wheelv_limit
			if phi_l < -self.wheelv_limit: phi_l = 0
			if phi_r > self.wheelv_limit: phi_r = self.wheelv_limit
			if phi_r < -self.wheelv_limit: phi_r = 0
			# Final robot velocity -- pcontroller + wheel kinematics
			ackerman_c_v = ((phi_r*r)/2) +((phi_l*r)/2)
			radius_of_rotation = 0	
			# Final robot angular velocity  --  pcontroller + wheel kinematics
			ackerman_c_w= 0
			# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
			self.robot.set_motor_control(ackerman_c_v, ackerman_c_w)  # use this command to set robot's speed in local frame
			self.robot.send_wheel_speed(phi_l,phi_r) #unit rad/sw   
   




				
  		#STEP 6: # reach way point criteria
		#you need to modify the reach way point criteria
		if abs(c_posX - d_posX) < 10 and abs(c_posY - d_posY) < 10 and abs(c_theta - d_theta) < 25:
			self.robot_destination_reached = True
		else:
			self.robot_destination_reached = False
		#--------------------------------------------------------------------------------------------------

		# use the following to log the variables, use [] to bracket all variables you want to store
		# stored values are in log folder
		if self.logging == True:
			self.robot.log_data([c_posX,c_posY,c_theta,c_vix,c_viy,c_wi,c_v,c_w])

		if (self.robot_destination_reached) == True: #you need to modify the reach way point criteria
			if(self.robot.state_des.reach_destination()): 
				print("final goal reached")
				self.robot.set_motor_control(.0, .0)  # stop the motor
				self.robot.send_wheel_speed(.0, .0)
				return True
			else:
				print("one goal point reached, continute to next goal point")
		
		return False
