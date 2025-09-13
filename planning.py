import py_trees
import numpy as np
from heapq import heapify, heappush, heappop
from collections import defaultdict
from map_helpers import world2map, map2world

def getNeighbors(inMap, point, goal):
    (i, j) = point
    neighbors = []
    for delta in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)):
        candidat = (i + delta[0], j + delta[1])
        is_in_bound = candidat[0] >= 0 and candidat[0] < len(inMap) and candidat[1] >= 0 and candidat[1] < len(inMap[0])
        is_navigable = False
        if is_in_bound:
            is_navigable = inMap[candidat[0]][candidat[1]] <= 0.3
        if is_navigable:
            cost = np.sqrt((goal[0] - candidat[0])**2 + (goal[1] - candidat[1])**2)
            neighbors.append((cost, candidat))
    return neighbors

def BFS(inMap, start, goal):
    queue = [(0, start)]
    heapify(queue)
    visited = set()
    parent = {}
    path = []
    distances = defaultdict(lambda:float("inf"))
    distances[start] = 0
    
    count = 0
    while len(queue) > 0:
        count+=1
        (curCost, current) = heappop(queue)
        for (neighborCost, neighbor) in getNeighbors(inMap, current, goal):
            if not  neighbor in visited:
                newCost = distances[current] + neighborCost
                if newCost < distances[neighbor]:
                    distances[neighbor] = newCost
                    heappush(queue, (newCost, neighbor))
                    visited.add(neighbor)
                    parent[neighbor] = current
        if current == goal:
            path.append(current)
            while current in parent.keys() and current != start:
                path.insert(0, parent[current])
                current = parent[current]
            return path
    return path

class Planning(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, target):
        super(Planning, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        px, py = world2map(target[0], target[1])
        self.target = (px, py)
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.display = self.robot.getDevice('display')
                
    def initialise(self):
        self.map = np.load('map.npy')
        self.display.setColor(0xFFFFF)
        for (row, rowVal) in enumerate(self.map):
            for (col, colVal) in enumerate(rowVal):
                if (colVal):
                    self.display.drawPixel(row+1, col+1)
        self.display.setColor(0xFF0000)
        self.display.drawPixel(self.target[0], self.target[1])
        
    def update(self):
        #print("Update Planning")
        # Get GPS
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        [xd, yd] = world2map(xw, yw)
        path = BFS(self.map, (xd, yd), self.target)
        WP = []
        for i, (x, y) in enumerate(path):
            if i % 7 == 0 or i == len(path) - 1:
                xm, ym = map2world(x, y)
                WP.append((xm, ym))
            
        self.blackboard.write('waypoints', WP)
                
        return py_trees.common.Status.SUCCESS