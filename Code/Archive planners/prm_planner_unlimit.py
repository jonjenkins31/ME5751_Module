import cv2
import numpy as np
import math
import random
from PIL import Image, ImageTk
from queue import Queue
from queue import PriorityQueue

from Path import *
# from Queue import Queue

class prm_node:
	def __init__(self,map_i=int(0),map_j=int(0)):
		self.map_i = map_i
		self.map_j = map_j
		self.edges = [] #edges of child nodes
		self.parent = None #parent node


class prm_edge:
	def __init__(self,node1=None,node2=None):
		self.node1 = node1 #parent node
		self.node2 = node2 #child node

#You may modify it to increase efficiency as list.append is slow
class prm_tree:
	def __init__(self):
		self.nodes = []
		self.edges = []

	def add_nodes(self,node):
		self.nodes.append(node)

	#add an edge to our PRM tree, node1 is parent, node2 is the kid
	def add_edges(self,node1,node2): 
		edge = prm_edge(node1,node2)
		self.edges.append(edge)
		node1.edges.append(edge)
		node2.parent=edge.node1


class path_planner:
	def __init__(self,graphics):
		self.graphics = graphics
		# self.graphics.scale = 400 #half pixel number on canvas, the map should be 800 x 800
		# self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2
		self.costmap = self.graphics.map
		self.map_width = self.costmap.map_width
		self.map_height = self.costmap.map_height
		self.pTree=prm_tree()
		self._init_path_img()
		self.path = Path()
		
  		#------------SET GOAL POINT -------------------------------------------------------------------
		# Specify Starting Location (0,0)
		self.set_start(world_x = 0, world_y = 0)
		#Set Goal Location in World coordinate ...  Location will be updated to Map coordinate system automaticly
		self.set_goal(world_x = 0.0, world_y = -120.0, world_theta = .0)
		#Run Path Planner Function 
		self.plan_path()
		# Show Calculated Path
		self._show_path()

	def set_start(self, world_x = 0, world_y = 0, world_theta = 0):
		self.start_state_map = Pose()
		map_i, map_j = self.world2map(world_x,world_y)
		print("Start with %d, %d on map"%(map_i,map_j))
		self.start_state_map.set_pose(map_i,map_j,world_theta)
		self.start_node = prm_node(map_i,map_j)
		self.pTree.add_nodes(self.start_node)

	def set_goal(self, world_x, world_y, world_theta = 0):
		self.goal_state_map = Pose()
		goal_i, goal_j = self.world2map(world_x, world_y)
		print ("goal is %d, %d on map"%(goal_i, goal_j))
		self.goal_state_map.set_pose(goal_i, goal_j, world_theta)
		self.goal_node = prm_node(goal_i,goal_j)
		self.pTree.add_nodes(self.goal_node)

	#convert a point a map to the actual world position
	def map2world(self,map_i,map_j):
		world_x = -self.graphics.environment.width/2*self.graphics.scale + map_j
		world_y = self.graphics.environment.height/2*self.graphics.scale - map_i
		return world_x, world_y

	#convert a point in world coordinate to map pixel
	def world2map(self,world_x,world_y):
		map_i = int(self.graphics.environment.width/2*self.graphics.scale - world_y)
		map_j = int(self.graphics.environment.height/2*self.graphics.scale + world_x)
		if(map_i<0 or map_i>=self.map_width or map_j<0 or map_j>=self.map_height):
			warnings.warn("Pose %f, %f outside the current map limit"%(world_x,world_y))

		if(map_i<0):
			map_i=int(0)
		elif(map_i>=self.map_width):
			map_i=self.map_width-int(1)

		if(map_j<0):
			map_j=int(0)
		elif(map_j>=self.map_height):
			map_j=self.map_height-int(1)

		return map_i, map_j

	def _init_path_img(self):
		self.map_img_np = 255*np.ones((int(self.map_width),int(self.map_height),4),dtype = np.int16)
		# self.map_img_np[0:-1][0:-1][3] = 0
		self.map_img_np[:,:,3] = 0

	def _show_path(self):
		for pose in self.path.poses:
			map_i = pose.map_i
			map_j = pose.map_j 
			self.map_img_np[map_i][map_j][1] =0
			self.map_img_np[map_i][map_j][2] =0
			self.map_img_np[map_i][map_j][3] =255

		np.savetxt("file.txt", self.map_img_np[1])

		self.path_img=Image.frombytes('RGBA', (self.map_img_np.shape[1],self.map_img_np.shape[0]), self.map_img_np.astype('b').tostring())
		# self.path_img = toimage(self.map_img_np)
		#self.path_img.show()
		self.graphics.draw_path(self.path_img)

	def check_vicinity(self,x1,y1,x2,y2,threshold = 1.0):
		if(math.sqrt((x1-x2)**2+(y1-y2)**2)<threshold):
			return True
		else:
			return False

	def plan_path(self):
		#this is the function you are going to work on

		#------------Main Code-------------------------------------------------------------------

		
		print("-----------")							#  print line to  identify start in terminal


		#STEP 1:  Setup/Variables/Arrays
		#Grids
		grid= self.costmap.costmap   												# EMPTY = 0 / OCCUPIED = 1000 /Saftey Zone = 500+ / Grids = 500 <--> 0 
		self.row_length = len(grid)                              					# Length of the rows
		self.col_length = len(grid[0])                           					# Length of the columns
		#Arrays
		start =(self.start_state_map.map_i,self.start_state_map.map_j) 				# Start position array
		goal =(self.goal_state_map.map_i,self.goal_state_map.map_j)					# Goal position array
		#variables
		prm_loop_counter=0															# Counter to idenify the amount of iterations conducted
		path_found=False															# Path Found logic 
		random_node_radius = 100

		#Queues and Dictionary
		node_list=[]												# List of nodes  formed
		node_list.append(start)										# Add Start point into the  List of nodes  formed
		node_edge = dict()											# Stores the path to each grid
		node_edge[start] = None										# Assign parent to desired node


		#STEP 2 PRM ALGORITHM LOOP
		#Perform Path Plan search                  
		while  path_found == False:								# loop as long as a list of neighbors to solve for exists
			
			# STEP 2A RANDOMLY SELECT NODE C  FROM LIST
			random_node_c = random.choice(node_list)  			# Randomly select a node c from the list of nodes
			(rx,ry) = random_node_c       						# get the node position  from the current node                 

			# STEP 2B RANDOMLY GENERATE NEW NODE C' FROM C 
			random_node_c_prime_valid = False					# Reset the logic bit to false
			ri = random.randint(0,self.map_width-1)			# Random number i variable
			rj = random.randint(0,self.map_height-1 )			# Random number j variable
			random_node_c_prime = (ri,rj)						# save current grid  position to be used as the new node position
			if  grid[random_node_c] < 500 and (ri,rj) not in node_list  :                	# Make sure random node is in a unoccupied space
				random_node_c_prime_valid = True
			if(self.check_vicinity(self.goal_node.map_i,self.goal_node.map_j,ri,rj,2.0)):			# define check_vicinity function and decide if you have reached the goal
				print ("We hit goal!")
				random_node_c_prime_valid = False


			# STEP 2C GENERATE EDGE E FROM NODE C' FROM C  - MAKE SURE IT IS COLLISION -FREE
			if  random_node_c_prime_valid == True:								# Perferm only if new node is valid
				points = bresenham(rx,ry,ri,rj)									# Generate pint line between nodes
				hit_obstacle = False											# Reset Hit obstacle variable
				for p in points:												# check if straightline from the two nodes hits any obstacle
					if(self.costmap.costmap[p[0]][p[1]]) >= 500: 				# path is not clear
						hit_obstacle = True
						break
				if(hit_obstacle==False):										# Path is clear
					node_list.append(random_node_c_prime)						# Add new node to list of nodes
					node_edge[random_node_c_prime] = random_node_c				# store the parent node of the new node .. later used for path reconstruction

			# STEP 2D GENERATE EDGE BETWEEN NODE C' TO GOAL - MAKE SURE IT IS COLLISION FREE
			if  random_node_c_prime_valid == True and hit_obstacle==False:									# Check if path to goal can be found 
					points = bresenham(ri,rj,self.goal_state_map.map_i,self.goal_state_map.map_j)			# Generate pint line new node and goal node
					hit_obstacle = False											# Reset Hit obstacle variable
					for p in points:												# check if straightline from the two nodes hits any obstacle
						if(self.costmap.costmap[p[0]][p[1]]) >= 500:				# Path is not clear
							hit_obstacle = True
							break
					if(hit_obstacle==False):								# path is clear .. path to goal found
						path_found=True										# Variable to be used to determain what is done after loop... Either generate path if found or inform user no path possible
						node_edge[goal] = random_node_c_prime				# store the parent node of the new node .. later used for path reconstruction

			# STEP 2E OPTIMALSATION GENERATE EDGE E FROM NODE C' FROM C  - MAKE SURE IT IS COLLISION -FREE
			if  random_node_c_prime_valid == True:														# Perferm only if new node is valid
				points = bresenham(self.start_state_map.map_i,self.start_state_map.map_j,ri,rj)			# Generate point line new node and goal node
				hit_obstacle = False										# check if straightline from start point to randomed (ri, rj) hits any obstacle
				for p in points:											# check if straightline from the two nodes hits any obstacle
					if(self.costmap.costmap[p[0]][p[1]]) >= 500: 			# depends on how you set the value of obstacle
						hit_obstacle = True
						break
				if(hit_obstacle==False):								# We didn't hit an obstacle
					node_edge[random_node_c_prime] = start			# determine if new node can directly go to start position... if so skip all previous nodes  and start a new branch.... later used for path reconstruction
		
			prm_loop_counter = prm_loop_counter+1					# Loop Counter
			#STOPPING CRITERIA
			if prm_loop_counter >= 100000:   						# Stop condition by loop counter 
				path_found=False									# Variable to be used to determain what is done after loop... Either generate path if found or inform user no path possible
				break

		#SET 4   Generate output path
		# Output... Depends if path was found or not
		if path_found==True:
			points = reconstruct_path(node_edge, start, goal)							# Run function to generate path points from node edge list
			for p in points:															# check if straightline from the two nodes hits any obstacle
				self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0)) 				# Export pathpoints to program
			self.path.save_path(file_name="Log\prm_path.csv")	
			print("Number of Iterations")												# CHECK OUTPUT DURING TESTING
			print(prm_loop_counter)														# CHECK OUTPUT DURING TESTING

		if path_found==False:
			print("Iteration Limit Reached: NO PATH TO GOAL FOUND")						# Output notification that no path was found 
 		#------------Main PController Code-------------------------------------------------------------------








# bresenham algorithm for line generation on grid map
# from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
def bresenham(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()

    # print points
    return points


#------------ ADDITONAL CODE-------------------------------------------------------------------

def reconstruct_path(node_edge, start, goal):
    path = []														# add frist path point as goal loaction
    current = goal															# Start the path reconstruction from the goal location
    N1 = current															# Start the path reconstruction from the goal location
    while current != start:													# Index through the Came_from list untill u get to the start location
        current = node_edge[current]										# get the stored  [grid -1]  location that was saved in the Came_From dictiionary
        N2 = current										# get the stored  [grid -1]  location that was saved in the Came_From dictiionary
        (N1_X,N1_Y) = N1   
        (N2_X,N2_Y) = N2   
        node_path = bresenham(N1_X,N1_Y,N2_X,N2_Y)
        path=path+node_path
        N1=N2
    path.reverse()															# reverse the array so that u start from the starting location instead of the goal location

    return path

