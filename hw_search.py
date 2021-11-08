import numpy as np
import copy
from operator import attrgetter
from scipy.spatial.distance import cdist

# Problem map
map = [[0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 1, 0],
       [0, 0, 0, 0, 0, 0, 1],
       [0, 0, 1, 0, 0, 0, 1]]

distances, dimensions = [], []                      # Global variables related to problem definition
toVisit, visited = [], []                           # Global variables to keep track of visited nodes
solution = []                                       # Global variable to record the current solution

target = None                                       # Global variable to keep discovered target

## Class representing one traversal node
class Node:
    def __init__(self, previousNode, coords, g, h):
        self.previousNode = previousNode
        self.coords = coords
        self.g = g
        self.h = h
        self.score = g + h

    # Override equality method
    def __eq__(self, other): 
        if not isinstance(other, Node):
            return NotImplemented
        return self.coords == other.coords


## Function to calculate the manhattan distance between a node and the nearest target
def manhattanDistance(indices, position, i, j):
    return abs(indices[position][0] - i) + abs(indices[position][1] - j)

## Function to calculate the distance matrix, where [x,y] represents the distance from this position to the nearest target
def calcDistances():
    for i in range (0, dimensions[0]):
        for j in range (0, dimensions[1]):
            x, y = np.where(np.array(map) > 0)
            indices = zip(x, y)
            position = np.argmin(cdist(np.array([[i,j]]), indices))

            distances[i][j] = manhattanDistance(indices, position, i, j)

## Function to append new node to lists
def addNode(previousNode, coords):
    global target

    newNode = Node(previousNode, coords, previousNode.g + 1, distances[coords[0]][coords[1]])

    if (map[newNode.coords[0]][newNode.coords[1]]):
        target = newNode

    # If there is a more effective route to the current position already found do not add the new node
    if newNode in toVisit:
        index = toVisit.index(newNode)
        if toVisit[index].score > newNode.score:
            toVisit.append(newNode)
    elif newNode in visited:
        index = visited.index(newNode)
        if visited[index].score > newNode.score:
            toVisit.append(newNode)
    else:
        toVisit.append(newNode)

## Function to generate new unvisited nodes
def generateNodes(previousNode):
    # Down
    if (previousNode.coords[0] + 1 < dimensions[0]):
        addNode(previousNode, [previousNode.coords[0] + 1, previousNode.coords[1]])
    # Right
    if (previousNode.coords[1] + 1 < dimensions[1]):
        addNode(previousNode, [previousNode.coords[0], previousNode.coords[1] + 1])
    # Up
    if (previousNode.coords[0] - 1 >= 0):
        addNode(previousNode, [previousNode.coords[0] - 1, previousNode.coords[1]])
    # Left
    if (previousNode.coords[1] - 1 >= 0):
        addNode(previousNode, [previousNode.coords[0], previousNode.coords[1] - 1])

## Function to append partial solution to global solution
def appendSolution(currentNode):
    partialSolution = []
    while (currentNode.previousNode != None):
        partialSolution.append(currentNode.coords)
        currentNode = currentNode.previousNode

    partialSolution.reverse()
    solution.extend(partialSolution)
    
## A* algorithm main loop
def aStar():
    global toVisit, visited, target

    while toVisit:
        best = min(toVisit, key=attrgetter('score'))
        currentNode = toVisit.pop(toVisit.index(best))

        generateNodes(currentNode)
        visited.append(currentNode)

        if target != None:
            map[target.coords[0]][target.coords[1]] = 0
            appendSolution(target)

            # Return new starting position
            return target.coords


## MAIN
if __name__ == '__main__':
    # Calculate the problem dimensions
    dimensions = np.shape(map)
    
    # Calculate the distance matrix to the nearest target
    distances = copy.deepcopy(map)
    calcDistances()
    
    coords = [0, 0]
    solution.append(coords)

    # Run a star algorithm until all targets are found
    while 1 in np.array(map):
        # Initialize the to visit list
        start = Node(None, coords, 0, 0)
        toVisit.append(start)
        coords = aStar()

        # Restart the search
        toVisit, visited = [], []
        target = None

    print(solution)
