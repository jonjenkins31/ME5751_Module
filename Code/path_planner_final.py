from tkinter import Grid
from tracemalloc import start
import turtle
import cv2
import numpy as np
import math
import warnings
from PIL import Image, ImageTk
from queue import Queue
from queue import PriorityQueue
import time

from bsTree import *
from Path import *
# from Queue import Queue


class path_planner:
	def __init__(self,graphics):
		self.graphics = graphics
		# self.graphics.scale = 400 #half pixel number on canvas, the map should be 800 x 800
		# self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2
		self.costmap = self.graphics.map
		self.map_width = self.costmap.map_width
		self.map_height = self.costmap.map_height
		self._init_path_img()
		self.path = Path()
		self.controller = self.graphics.environment.robots[0].controller
		self.robot = self.graphics.environment.robots[0]		
		# Specify Starting Location (0,0)
		self.set_start(world_x = 0, world_y = 0)
		#Set Goal Location in World coordinate ...  Location will be updated to Map coordinate system automaticly
		self.set_goal(world_x = -100.0, world_y = -220.0, world_theta = .0)
		#Run Path Planner Function 
		self.plan_path()
		# Show Calculated Path
		self._show_path()



	def set_start(self, world_x = 0, world_y = 0, world_theta = 0):
		self.start_state_map = Pose()
		map_i, map_j = self.world2map(world_x,world_y)
		print("Start with %d, %d on map"%(map_i,map_j))
		self.start_state_map.set_pose(map_i,map_j,world_theta)


	def set_goal(self, world_x, world_y, world_theta = 0):
		self.goal_state_map = Pose()
		map_i, map_j = self.world2map(world_x, world_y)
		print ("our new goal is %d, %d on map"%(map_i,map_j))
		self.goal_state_map.set_pose(map_i, map_j, world_theta)


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

		self.path_img=Image.frombytes('RGBA', (self.map_img_np.shape[1],self.map_img_np.shape[0]), self.map_img_np.astype('b').tostring())
		self.graphics.draw_path(self.path_img)

		# If you want to save the path as an image, un-comment the following line:
		# self.path_img.save('Log\path_img.png')
		
		# If you want to output an image of map and path, un-comment the following two lines
		# self.path_img = toimage(self.map_img_np)
		# self.path_img.show()

	def plan_path(self):
		start_time = time.time()

		# The major program you are going to write!
		# The following simply demo that how you can add pose to path
		self.path.clear_path()

		#------------Main Code-------------------------------------------------------------------

		
		print("-----------")							# CHECK OUTPUT DURING TESTING

		#STEP 1:  Setup/Variables/Arrays
		#Grids
		grid= self.costmap.costmap   												#  OCCUPIED = 1000 /Saftey Zone = 800 / UNOCCUPIED = 800 -- 0 
		unoccupied_space_max_value=800
		self.row_length = len(grid)                              					# Length of the rows
		self.col_length = len(grid[0])                           					# Length of the columns
		search_mapA=np.zeros((self.row_length,self.col_length), dtype=int)           # search_map map ---- THIS MATRIX REPRESENTS THE A* Cost MAP 
		search_mapB=np.zeros((self.row_length,self.col_length), dtype=int)           # search_map map ---- THIS MATRIX REPRESENTS THE  A* Cost MAP 
		#Arrays
		start =(self.start_state_map.map_i,self.start_state_map.map_j)
		goal =(self.goal_state_map.map_i,self.goal_state_map.map_j)
		direction = ((-1, 0), (0, -1), (1, 0), (0, 1))                              # Left, bottom, Right, Right
		points_all=[]
		#variables
		nodes_search_counter=0
		path_found=False
		u=1
		all_path_found=False
		loop_counter=0															# Counter to idenify the amount of iterations conducted
		Convergence=False
		waypoint_max_distance = 150

		#Queues and Dictionary
		frontierA = PriorityQueue()									# the list of current spaces being expanded and neibors are being assign numbers (the smallest priority value always comes out first)
		frontierA.put((0,start))
		frontierB = PriorityQueue()									# the list of current spaces being expanded and neibors are being assign numbers
		frontierB.put((0,goal))
		node_listA=[]												# List of nodes  formed
		node_listA.append(start)										# Add Start point into the  List of nodes  formed
		node_listB=[]												# List of nodes  formed
		node_listB.append(goal)										# Add Start point into the  List of nodes  formed
		node_edgeA = dict()											# Stores the path to each grid
		node_edgeA[start] = None
		node_edgeB = dict()											# Stores the path to each grid
		node_edgeB[goal] = None
		cost_so_farA = dict()										# Stores the Sum cost of each grid
		cost_so_farA[start] = 0
		cost_so_farB = dict()										# Stores the Sum cost of each grid
		cost_so_farB[goal] = 0



		if  grid[goal] > unoccupied_space_max_value:									# If Goal Location is within a wall or the saftey zone  then send message to user
			print("Your Selected Goal Location lies in a Wall -- Please Change Goal Location")							# CHECK OUTPUT DURING TESTING

		#SET 2  Reused Brushfire Algorithm used for Breadth First Search
		# Perform Burstfire to give each grid a distance value away from the startpoint
		bfs_A=brushfire(grid,start,unoccupied_space_max_value)
		bfs_B=brushfire(grid,goal,unoccupied_space_max_value)


		#SET 3  			# A* ALGORITHM

		while  path_found == False:								# loop as long as a list of neighbors to solve for exists
			
			# A* ALGORITHM FROM START 
			#Perform Path Plan search and find the path with the lowest total cost:         
			if not frontierA.empty():								# loop as long as a list of neighbors to solve for exists
				priority,current=frontierA.get()						# Pull the position with the lowest cost 
				(r,c) = current										# save current grid  position to be used when indexing to neighbors
				if current == goal:   								# Early exit
					path_found=True									# Variable to be used to determain what is done after loop... Either generate path if found or inform user no path possible
					break
				for i ,j in direction:                    				# INDEX TO ALL NEIGHBORS AROUND THE SELECTED CURRENT POSITION 
					current_r=  r + i
					current_c=  c + j
					next=(current_r, current_c)
					if 0 <= current_r < self.row_length and 0 <= current_c < self.col_length and grid[next] < unoccupied_space_max_value-300   :                #requirements for neighbor selection
						search_mapA[next] =  grid[next]+ bfs_A[next] +bfs_B[next]+u*heuristic(goal, next)												# Search map cost = Cost from costmap + Distance from starting point + Manhattan Distance from Goal(Heuristic)
						new_cost = cost_so_farA[current] + search_mapA[next] 															# Cell Cost = Actual Cost + Search Map Grid Cost 
						nodes_search_counter=nodes_search_counter+1																	# Simple Counter to keep track how many times did the algorithm visit a grid 
						if next not in cost_so_farA or new_cost < cost_so_farA[next]:											#Only if the new node has a lower cost  or is not stored already do the program use in the next path
							cost_so_farA[next] = new_cost													# Store the node cost
							priority = new_cost																# Copies cost to be used in priority queue managment... Lowest cost = highest priority
							frontierA.put((priority,next)) 													# Store the node to the list of expanding nodes ... organized by priority 
							node_edgeA[next] = current														# store the next node with the name of the current node used to get their... later used for path reconstruction
							points_all.append(next)
							node_listA.append(next)										# Add Start point into the  List of nodes  formed
						if search_mapB[next] >  0: 
							Convergence=next
							path_found=True									# Variable to be used to determain what is done after loop... Either generate path if found or inform user no path possible
							break



			# A* ALGORITHM FROM GOAL
			if not frontierB.empty():								# loop as long as a list of neighbors to solve for exists
				priority,current=frontierB.get()						# Pull the position with the lowest cost 
				(r,c) = current										# save current grid  position to be used when indexing to neighbors
				if current == start:   								# Early exit
					path_found=True									# Variable to be used to determain what is done after loop... Either generate path if found or inform user no path possible
					break
				for i ,j in direction:                    				# INDEX TO ALL NEIGHBORS AROUND THE SELECTED CURRENT POSITION 
					current_r=  r + i
					current_c=  c + j
					next=(current_r, current_c)
					if 0 <= current_r < self.row_length and 0 <= current_c < self.col_length and grid[next] < unoccupied_space_max_value-300   :                #requirements for neighbor selection
						search_mapB[next] =  grid[next]+ bfs_A[next] +bfs_B[next]+u*heuristic(start, next)												# Search map cost = Cost from costmap + Distance from starting point + Manhattan Distance from Goal(Heuristic)
						new_cost = cost_so_farB[current] + search_mapB[next] 															# Cell Cost = Actual Cost + Search Map Grid Cost 
						nodes_search_counter=nodes_search_counter+1																	# Simple Counter to keep track how many times did the algorithm visit a grid 
						if next not in cost_so_farB or new_cost < cost_so_farB[next]:											#Only if the new node has a lower cost  or is not stored already do the program use in the next path
							cost_so_farB[next] = new_cost													# Store the node cost
							priority = new_cost																# Copies cost to be used in priority queue managment... Lowest cost = highest priority
							frontierB.put((priority,next)) 													# Store the node to the list of expanding nodes ... organized by priority 
							node_edgeB[next] = current														# store the next node with the name of the current node used to get their... later used for path reconstruction
							points_all.append(next)
							node_listB.append(next)										# Add Start point into the  List of nodes  formed
						if search_mapA[next] >  0:  
							Convergence=next
							path_found=True									# Variable to be used to determain what is done after loop... Either generate path if found or inform user no path possible
							break



			loop_counter = loop_counter+1					# Loop Counter
			#STOPPING CRITERIA
			if  frontierA.empty() and  frontierB.empty():   						# Stop condition by loop counter 
				path_found=False									# Variable to be used to determain what is done after loop... Either generate path if found or inform user no path possible
				break
			if loop_counter >= 500000:   						# Stop condition by loop counter 
				path_found=False									# Variable to be used to determain what is done after loop... Either generate path if found or inform user no path possible
				break
			elif loop_counter == 2000 :
				print ("2000 iterations completed")
			elif loop_counter == 10000:
				print ("5000 iterations completed")
			elif loop_counter == 50000:
				print ("10000 iterations completed")



		
		#SET 4   Generate output path
		# Output... Depends if path was found or not
		if path_found==True:
			print("PATH TO GOAL FOUND")													# CHECK OUTPUT DURING TESTING
			print("the number of nodes searched")						# CHECK OUTPUT DURING TESTING
			print(nodes_search_counter)	
			print("the number of iterations")						# CHECK OUTPUT DURING TESTING
			print(loop_counter)
			pointsA= reconstruct_path(node_edgeA, start, Convergence,True)							# Run function to generate path points from node_edge list
			pointsB = reconstruct_path(node_edgeB, goal ,Convergence, False)							# Run function to generate path points from node_edge list
			points = pointsA+pointsB
			print("the number of points in path ")						# CHECK OUTPUT DURING TESTING
			print(len(points))		
			
			#SET 5    light PATH OPTIMIZATION																	# CHECK OUTPUT DURING TESTING
			reduced_points=[]												# List of nodes  formed
			last_saved=start
			for x in range(len(points)):
				if points[x] == points[len(points)-1]:  
					reduced_points.append(points[x]) 	
				else:
					P1,P2=points[x]
					P4,P5=points[x+1]
					point_n=(P1,P2)
					point_n1=(P4,P5)		
					point_n_collision_free = collision(last_saved,point_n,grid)
					point_n_distance = manhattan(last_saved,point_n)
					point_n1_collision_free = collision(last_saved,point_n1,grid)
					point_n1_distance = manhattan(last_saved,point_n1)
					if point_n_collision_free ==True and  point_n_distance < waypoint_max_distance/4:  
						if point_n1_collision_free ==False or  point_n1_distance >= waypoint_max_distance/4:  
							reduced_points.append(points[x]) 
							last_saved=point_n
					else:
						print("path optimization error")
			print("the number of points in reduced path ")						# CHECK OUTPUT DURING TESTING
			print(len(reduced_points))		

			#SET 6    Assign Theta 																	# CHECK OUTPUT DURING TESTING
			points_theta=[]												# List of nodes  formed
			for x in range(len(reduced_points) ):
				#get 3 points
				currentx,currenty=reduced_points[x]
				if (x+1) >= (len(reduced_points)):  
					nextx,nexty=reduced_points[x]	
				else:
					nextx,nexty=reduced_points[x+1]	
				slope = find_theta((nextx-currentx)*100,(nexty-currenty)*100)
				theta= (slope)
				if theta == 999:  
					theta= last_good_theta
				else: 
					last_good_theta= (slope)	
				points_theta.append((currentx,currenty,theta)) 
   
			
			#SET 6    Max OPTIMIZATION																	# CHECK OUTPUT DURING TESTING
			reduced_points_theta=[]												# List of nodes  formed
			last_saved=start
			for x in range(len(points_theta)):
				if points_theta[x] == points_theta[len(points_theta)-1]:  
					reduced_points_theta.append(points_theta[x]) 	
				else:
					P1,P2,P3=points_theta[x]
					P4,P5,P6=points_theta[x+1]
					point_n=(P1,P2)
					point_n1=(P4,P5)		
					point_n_collision_free = collision(last_saved,point_n,grid)
					point_n_distance = manhattan(last_saved,point_n)
					point_n1_collision_free = collision(last_saved,point_n1,grid)
					point_n1_distance = manhattan(last_saved,point_n1)		
					if point_n_collision_free ==True and last_saved==start and point_n_distance < waypoint_max_distance:#/3:  
						if point_n1_collision_free ==False or  point_n1_distance >= waypoint_max_distance:#/3:  
							reduced_points_theta.append(points_theta[x]) 
							last_saved=point_n
					if point_n_collision_free ==True and last_saved!=start and  point_n_distance < waypoint_max_distance:  
						if point_n1_collision_free ==False or  point_n1_distance >= waypoint_max_distance:  
							reduced_points_theta.append(points_theta[x]) 
							last_saved=point_n
					else:
						print("path optimization error")

			print("the number of points in Final path ")						# CHECK OUTPUT DURING TESTING
			print(len(reduced_points_theta))


			#SET 7   OUTPUT FINAL PATH TO P CONTROLLER
			for p in points:
           		#-------Visual Points----
				self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0)) #theta is wrong			# Export pathpoints to program

			for p in reduced_points_theta:
    			#-------Waypoints----
				pmap_i, pmap_j = self.map2world(p[0],p[1])
				self.robot.state_des.add_destination(x=pmap_i,y=pmap_j,theta=p[2]) #theta is right

		if all_path_found==True:
			for p in points_all:															# check if straightline from the two nodes hits any obstacle
				self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0)) 				# Export pathpoints to program
			self.path.save_path(file_name="Log\prm_path.csv")	

		if path_found==False:
			print("NO PATH TO GOAL FOUND")							# CHECK OUTPUT DURING TESTING
		
		print("--- %s seconds ---" % (time.time() - start_time))
 
		#------------Main PController Code-------------------------------------------------------------------



# bresenham algorithm for line generation on grid map
# from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
def bresenham(x1, y1, x2, y2):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions

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

def reconstruct_path(node_edge, start, goal,reverse):
    current = goal															# Start the path reconstruction from the goal location
    path = [current]														# add frist path point as goal loaction
    while current != start:													# Index through the node_edge list untill u get to the start location
        current = node_edge[current]										# get the stored  [grid -1]  location that was saved in the node_edge dictiionary
        path.append(current)												# add all these paths to the path array
    if reverse==True:
        path.reverse()															# reverse the array so that u start from the starting location instead of the goal location
    return path


def heuristic(goal,next):
    (x1, y1) = goal
    (x2, y2) = next
    manhattan= abs(x1 - x2) + abs(y1 - y2)									# generate the manhattan distance value from goal to current position
    return manhattan

def brushfire(grid,gridspace,unoccupied_space_max_value):
	distance = 1 
	row_length = len(grid)                              					# Length of the rows
	col_length = len(grid[0])    
	bfs=np.zeros((row_length,col_length), dtype=int)             # bfs map ---- THIS MATRIX REPRESENTS THE DISTANCE MAP   0 = occupied / # from occupied space
	bfs_current_queue =[]                                        			# Take the list of Islands as the first bfs  queue
	bfs_current_queue.append(gridspace)                                        # Take the list of Islands as the first bfs  queue
	direction = ((-1, 0), (0, -1), (1, 0), (0, 1))                              # Left, bottom, Right, Right
                                                            # Node Value 

	while bfs_current_queue:                   	# loop as long as a list of neighbors to solve for exists
				bfs_next_queue = []						# clear the new found neighbors list
				while bfs_current_queue :                  # FIND ALL THEN NEIGHBORS AND  SAVE THEM TO THE bfs NEXT QUEUE>>> ALSO RECORD THE CURRENT NEIGHBOR DISTANCE
					(r,c) = bfs_current_queue.pop()        # get the next neighbor position  from the current neighbor list                 
					for i ,j in direction:                    	# INDEX TO ALL NEIGHBORS AROUND THE SELECTED CURRENT POSITION 
						current_r=  r + i
						current_c=  c + j
						if 0 <= current_r < row_length and 0 <= current_c < col_length and grid[current_r][current_c] <= unoccupied_space_max_value-300 and bfs[current_r][current_c] == 0:                       #requirements for neighbor selection
							bfs[current_r][current_c] = round(distance, 1)
							bfs_next_queue.append((current_r, current_c))                         # Add neighbor  to next  loop queue	
				distance = distance+1                  # index the distance value for the next round of neighbors
				bfs_current_queue  = bfs_next_queue             # Save the list of new neighbors to the QUEUE and rerun bfs untill all neighbors have been given a distance value 
			
	return bfs

def find_theta(dx,dy):
	#Set singularities
	if dx == 0 and dy > 0:  
		angle = 0	
	elif dx < 0 and dy == 0:
		angle = math.pi/2
	elif dx == 0 and dy < 0:  
		angle = math.pi
	elif dx > 0 and dy == 0:  
		angle = -math.pi/2
		#angle = 1.5*math.pi

		
	#Set singularities
	elif dx > 0 and dy > 0:
		angle = -math.pi/2+math.atan(dy/dx)  	#GOOD
		#angle =  1.5*math.pi+math.atan(dy/dx)  	#GOOD
	elif dx < 0 and dy > 0:  
		angle = math.pi/2+math.atan(dy/dx)		#good
#		angle = math.pi/2+math.atan(dy/dx)		#good
	elif dx < 0 and dy < 0:  
		angle = math.pi/2+math.atan(dy/dx)		#GOOD
#		angle = math.pi/2+math.atan(dy/dx)		#GOOD
	elif dx > 0 and dy < 0:  
		angle = -math.pi/2+math.atan(dy/dx)		#Good
		#angle = 1.5*math.pi+math.atan(dy/dx)		#Good

	else:
		angle = 999 # generate the manhattan distance value from goal to current position
	return angle

def manhattan(p1,p2):
    (x1, y1) = p1
    (x2, y2) = p2
    manhattan= abs(x1 - x2) + abs(y1 - y2)									# generate the manhattan distance value from goal to current position
    return manhattan

def collision(point1,point2,costmap):
	x1,y1=point1
	x2,y2 =point2
	path = bresenham(x1, y1, x2, y2)			# Generate pint line new node and goal node
	hit_obstacle = False											# Reset Hit obstacle variable
	collision_free = False											# Reset Hit obstacle variable
	for p in path:												# check if straightline from the two nodes hits any obstacle
		if(costmap[p[0]][p[1]]) >= 800-50:				# Path is not clear
			hit_obstacle = True
			break
	if(hit_obstacle==False):								# path is clear .. path to goal found
		collision_free=True										# Variable to be used to determain what is done after loop... Either generate path if found or inform user no path possible
	return collision_free

