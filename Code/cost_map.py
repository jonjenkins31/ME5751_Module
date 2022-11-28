import cv2
import numpy as np
import math
from PIL import Image, ImageTk
from queue import Queue

class cost_map:
	def __init__(self,graphics):
		self.graphics = graphics
		self.inflation_radius = 18 # radius of our robot is 18 pixel or cm
		self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2
		self.map_width = int(self.graphics.environment.width*self.graphics.scale)
		self.map_height = int(self.graphics.environment.height*self.graphics.scale)
		try:
			self.load_map(map = "maps/testmap_maze.png") #load map
		except:
			self.graphics.show_map_button.configure(state="disabled")
			print ("no map loaded") #if fail to find the map png
			return
		self.show_map()
		self.compute_costmap()

		# self.show_costmap()
		self.save_vis_map(map = "maps/testcostmap.png")

	#load occupancy grid into self.map
	#self.map is a numpy 2d array
	#initialize self.costmap, a numpy 2d array, same as self.map
	def load_map(self,map="maps/testmap_maze.png"):
		self.map_img = Image.open(map).convert('L')
		self.map_img = self.map_img.resize((int(self.map_width),int(self.map_height)),Image.ANTIALIAS)
		# self.graphics.draw_map(map_img=self.map_img)
		self.map = cv2.imread(map,cv2.IMREAD_GRAYSCALE)
		print (self.map.dtype)
		print ("Loaded map dimension: %d x %d pixel"%(self.map.shape[0],self.map.shape[1]))
		self.map = cv2.resize(self.map, dsize=(int(self.map_width),int(self.map_height)), interpolation=cv2.INTER_CUBIC)
		self.vis_map=np.copy(self.map) #map for visualization
		self.distmap=np.copy(self.map).astype(np.float)
		self.costmap=np.copy(self.map).astype(np.float)

	#save your costmap into a grayscale image
	def save_vis_map(self,map="maps/costmap.png"):
		save_img = Image.fromarray(self.vis_map)
		save_img.save(map)

	def show_vis_map(self):
		self.get_vis_map()
		self.vis_map_img=Image.frombytes('L', (self.vis_map.shape[1],self.vis_map.shape[0]), self.vis_map.astype('b').tostring())
		self.graphics.draw_map(map_img=self.vis_map_img)

	#display costmap on the dialogue window
	def show_costmap(self):
		self.costmap_img=Image.frombytes('L', (self.costmap.shape[1],self.costmap.shape[0]), self.costmap.astype('b').tostring())
		self.graphics.draw_map(map_img=self.costmap_img)

	#display binary occupancy grid on the dialogue window 
	def show_map(self):
		self.graphics.draw_map(map_img=self.map_img)


	#This is the function you really should update!

	def compute_costmap(self):
		#The following is an example how you manipulate the value in self.costmap
		#It has nothing to do with brushfire, replace it
		#A whilte pixel has value of 255, a black one is 0
		#However, some block value may be dark gray
		# self.costmap[200:250][0:-1]=0  		#Set costmap pizel to a specific value Black
		#self.costmap[300:350][0:-1]=250   #Set costmap pizel to a specific value white
		
		#SET 1 
		# Store costmap as grid 
		# Grey Scale  0 = black 255 = white
		# Convert greyscale to object present/ nonpresent 1/0 
		# filter the grid and make islands a value of 1 and water a value of 0 
		grid=np.copy(self.costmap)				# Copy costmap to grip matrix  ---- THIS MATRIX REPRESENTS THE OCCUPIED/UNOCCUPIED MAP
		greyscale_tollerance = 50 				# pixels might be grey not black--- Specify tollerance to make grey pixels regestered as black (occupied)
		grid[grid < greyscale_tollerance] = 1	# convert grid  greyscale value to  1 If space if black (occupied)
		grid[grid > greyscale_tollerance] = 0	# convert grid  greyscale value to  0 If space if not black (unoccupied)
		#self.costmap=grid						# CHECK OUTPUT DURING TESTING
		#print(grid)							# CHECK OUTPUT DURING TESTING
		# #np.savetxt('Log/test.csv',grid, fmt='%-1d', delimiter=',')    	# CHECK OUTPUT DURING TESTING

		#SET 2 A
		# Using code created for hw2 . Run bushfire algorithum on grid to generate a distance value grid (bushfire grid).
		# Make an array for Occupied and Unoccupied spaces
		# # Initializing a array (queue)
		self.land = []                                      # list of land (occupied spaces = 1)
		self.water = []                                     # list of water  (unoccupied spaces = 0)
        # Identify Grid size
		# A(i,j) = A(r,c)
		self.row_length = len(grid)                              # Length of the rows
		self.col_length = len(grid[0])                           # Length of the columns
		 # Scan GRID Top Down -- LEFT to RIGHT
		for r in range(self.row_length):
			for c in range(self.col_length):
				if grid[r,c] == 1:        
					self.land.append((r,c))                     # IF GRID CONTAINTS A 1 THEN IT IS LAND. ADD TO LAND QUEUE
				else:                      
					self.water.append((r,c))                     # IF NOT LAND THEN WATER. ADD TO WATER QUEUE
		 # Check to see if the grid contains atleast 1 land or 1 water ... If not then return -1 
		if  not self.land or not self.water:
			if  not self.land:                                      # No Land Detected
				Explination = "All cells are empty"
				print(Explination)							# CHECK OUTPUT DURING TESTING
			if  not self.water:                                     # No Water Detected
				Explination = "All cells are occupied"
				print(Explination)							# CHECK OUTPUT DURING TESTING
	
		#SET 2 B
		# Bushfire with 4 connection method. 
		#BUSHFIRE : INFLATION METHOD.... INDEX ONE NEIGHBOR AT A TIME AROUND THE ENTIRE LAND .. LAST NEIGHBOR IS THE FARTHERST
		Bushfire=np.zeros((self.row_length,self.col_length), dtype=int)             # Bushfire map ---- THIS MATRIX REPRESENTS THE DISTANCE MAP   0 = occupied / # from occupied space
		direction = ((-1, 0), (1, 0), (0, -1), (0, 1))                              # Left, Top, Right, Bottom
		distance = 1                                                                # Starting Distance
		bushfire_current_queue =self.land                                           # Take the list of Islands as the first bushfire  queue
		while bushfire_current_queue:                   	# loop as long as a list of neighbors to solve for exists
			bushfire_next_queue = []						# clear the new found neighbors list
			while bushfire_current_queue :                  # FIND ALL THEN NEIGHBORS AND  SAVE THEM TO THE BUSHFIRE NEXT QUEUE>>> ALSO RECORD THE CURRENT NEIGHBOR DISTANCE
				(r,c) = bushfire_current_queue.pop()        # get the next neighbor position  from the current neighbor list                 
				for i ,j in direction:                    	# INDEX TO ALL NEIGHBORS AROUND THE SELECTED CURRENT POSITION 
					current_r=  r + i
					current_c=  c + j
					if 0 <= current_r < self.row_length and 0 <= current_c < self.col_length and grid[current_r][current_c] == 0 and Bushfire[current_r][current_c] == 0:                       #requirements for neighbor selection
						#Bushfire[current_r][current_c] = distance                             # record neighbor grid distance to Bushfire Map
						Bushfire[current_r][current_c] = round(distance, 1)
						bushfire_next_queue.append((current_r, current_c))                         # Add neighbor  to next  loop queue	
			distance = distance+1                  # index the distance value for the next round of neighbors
			bushfire_current_queue  = bushfire_next_queue             # Save the list of new neighbors to the QUEUE and rerun bushfire untill all neighbors have been given a distance value 
		#self.costmap=Bushfire						# CHECK OUTPUT DURING TESTING
		#print(Bushfire)							# CHECK OUTPUT DURING TESTING
		#np.savetxt('Log/test.csv',Bushfire, fmt='%-1d', delimiter=',')    	# CHECK OUTPUT DURING TESTING



		#SET 3
		# Convert the Bushfire map to a potential map / Cost map
		# SPACES CLOSER TO AN OBJECT IS AT A HIGHER COST/VALUE
		bushfire_costmap=Bushfire        # cost map ---- THIS MATRIX REPRESENTS THE COST MAP --- closer to object the higher the cost
		max_cost = 1000									# Specify the desired "cost" for an occupied space
		max_cost_saftey_zone = 800						# Specify the desired starting "cost" for any open space within the saftey zone of a occipied space
		min_cost_saftey_zone = 500 						# Specify the desired MINIMUM "cost" for any open space within the saftey zone of a occipied space
		saftey_zone_distance = 15						# Specify the desired saftey zone distance from known occupied space
		cost_rate_safty=(min_cost_saftey_zone-max_cost_saftey_zone)/(saftey_zone_distance) 			# Specify the rate of cost decrease for each space in the saftey zone 
		cost_rate = 15																				# Specify the rate of cost decrease for each space after the saftey zone --- cost will incrementaly decrease till cost is 0
		# Scan costmap Top Down -- LEFT to RIGHT
		for r in range(self.row_length):
			for c in range(self.col_length):
				if bushfire_costmap[r,c] == 0:        
					bushfire_costmap[r][c] = max_cost                           											 # If bushfire distance is 0 then this zone is occupied and has the highest cost value
				elif bushfire_costmap[r,c] < saftey_zone_distance :        
					bushfire_costmap[r][c] = max_cost_saftey_zone+(cost_rate_safty)*bushfire_costmap[r,c]                    # assign high cost value for spaces in saftey zone (inflation zone to stay out of)
				else:                      
					bushfire_costmap[r][c] = min_cost_saftey_zone+(-cost_rate)*bushfire_costmap[r,c]                         # assign  progresivly lower cost value for spaces farther away from occupied zone 
					if bushfire_costmap[r][c]  < 0 :        
						bushfire_costmap[r][c] = 0                           												# LOWEST COST VALUE IS ZERO..
                  
		# COSTMAP COMPLETED EXPORT RESULTS
		self.costmap=bushfire_costmap			# Set the costmap pixel to be equal to our bushfire costmap
		# self.costmap[200:250][0:-1]=0  		#Set costmap pizel to a specific value Black
		# self.costmap[300:350][0:-1]=250   #Set costmap pizel to a specific value white
		# print(bushfire_costmap)				# CHECK OUTPUT DURING TESTING
		# np.savetxt('Log/test.csv',bushfire_costmap, fmt='%-1d', delimiter=',')    	# CHECK OUTPUT DURING TESTING


		pass

	#scale costmap to 0 - 255 for visualization
	def get_vis_map(self):
		#self.vis_map = np.uint8(self.costmap)
		self.vis_map = np.uint8(255-self.costmap/4.0)
		np.savetxt("Log/vismap.txt",self.vis_map)

