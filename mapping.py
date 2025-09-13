import py_trees
import numpy as np
from scipy import signal
from matplotlib import pyplot as plt
from map_helpers import world2map, map2world

class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.hasrun = False
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        # get devices
        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        self.display = self.robot.getDevice('display')
                
    def initialise(self):
        self.map = np.zeros((200,300))
        self.angles = np.linspace(2.095, -2.095, 667)
        
    def update(self):
        self.hasrun = True
        # Get GPS
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]   
        # Read compass values
        robot_angle = np.arctan2(self.compass.getValues()[0],self.compass.getValues()[1])
        # Read lidar data
        ranges = np.array(self.lidar.getRangeImage())
        # Ignore edge rays because of the cover
        edge_cut = 40  # number of rays to ignore on each side
        ranges = ranges[edge_cut:-edge_cut]
        angles = self.angles[edge_cut:-edge_cut]
        ranges[ranges == np.inf] = 100
        # Transform matrix 
        w_T_r = np.array([[np.cos(robot_angle), -np.sin(robot_angle), xw], 
                        [np.sin(robot_angle), np.cos(robot_angle), yw],
                        [0, 0, 1]])        
        # Add lidar offset from robot center (0.202 m)
        lidar_offset = np.array([[1, 0,  0.202],
                                    [0, 1,  0],
                                    [0, 0,  1]])               
        X_i = np.array([ranges*np.cos(angles), ranges*np.sin(angles), np.ones((len(ranges),))])
        D = w_T_r @ (lidar_offset @ X_i)
        # Update obstacle probability in the map
        for i in range(len(D[0])):
            [xd, yd] = world2map(D[0][i], D[1][i])
            if self.map[xd,yd] < 1:
                self.map[xd,yd] = min(self.map[xd,yd] + 0.01, 1)
            # Print the pixel
            gradient = int(self.map[xd, yd] * 255)
            color = (gradient*256**2 + gradient*256 + gradient)
            self.display.setColor(color)
            self.display.drawPixel(xd, yd)
        # Print trajectory on display
        [xd, yd] = world2map(xw, yw)
        self.display.setColor(0xFF0000)
        self.display.drawPixel(xd, yd)
        
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        if self.hasrun:
            kernel= np.ones((24,24))
            cmap = signal.convolve2d(self.map,kernel,mode='same')
            # Use 90% threshold 
            cspace = cmap > 0.9
            np.save('map',cspace)
            #plt.imshow(cspace)
            #plt.show()