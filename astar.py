from algorithms.base import SearchAlgorithmBase

class astar(SearchAlgorithmBase):
    pathTracking = {}
    cost = {}
    def __init__(self) -> None:
        super().__init__()
    
    def reset(self, grid, start, goal) -> None:
        self._cost = 0
        self._grid = grid
        self._start = start
        self._goal = goal
        self._frontier.clear()
        self._explored.clear()
        self.pathTracking.clear()
        self.cost.clear()
        self._path.clear()
        self._frontier.append(start)
        self.pathTracking[start] = -1
        self.cost[start] = 0
        self.cost[-1] = 0

    def step(self) -> None:
        if (len(self._frontier) != 0): #with this part method checks if the frontier is empty to determine whether or not to continue exploring. Cause if frontier is empty then it will stop continuing.
            newExplored = self.removeNodeFromFrontier() #If its not empty, method will find the least cost one to continue.
            if(self._grid[newExplored] == 3):# ıf the final selected node is the one that desired, algorithm sets "True" and then marks the final way
                self._done = True
                self.pathFinding()
            else:
                self.addNewNodesToFrontier(newExplored) #ıf ıt is not the final node that desired than algorithm continues to the exploring process.

    def removeNodeFromFrontier(self) -> list:
        minCost = abs(self._frontier[0][0] - self._goal[0]) + abs(self._frontier[0][1] - self._goal[1]) + self.cost[self.pathTracking[self._frontier[0]]] #frontier list first element's manhattan distance(| x 1 − x 2 | + | y 1 − y 2 |) and total cost so far is added to the cost which is called minCost.
        minIndex = 0  #every node at the frontier, we measure the manhattan distance, if that measurement is less than it updates as minIndex.  
        for i in range(len(self._frontier)):
            if (abs(self._frontier[i][0] - self._goal[0]) + abs(self._frontier[i][1] - self._goal[1]) + self.cost[self.pathTracking[self._frontier[i]]] < minCost):
               minIndex = i
               minCost = abs(self._frontier[i][0] - self._goal[0]) + abs(self._frontier[i][1] - self._goal[1]) + self.cost[self.pathTracking[self._frontier[i]]]
        newExplored = self._frontier.pop(minIndex)  #after the loop completes, least cost node gets out of the frontier with the usage of pop method and added to newExplored.
        self.cost[newExplored] = minCost  #newExplored is updated as the MinCost lib.
        self._explored.append(newExplored)  #It added to the explored list
        return newExplored
    
    def addNewNodesToFrontier(self, newExplored) -> None:
        x,y = newExplored[0], newExplored[1]
        size_x = len(self._grid)   #get the x and y coords of new explored node
        size_y = len(self._grid[0])
        if(x - 1 >= 0):
            w = (x - 1, y)
            if (not w in self._explored and not w in self._frontier and self._grid[w] != 1):  #for every 4 side of a node west east south and north algorithm checks that it is not reached to the walls of maze
                self._frontier.append(w)                                                      # If an adjacent has not been discovered, it added to the frontier list
                self.pathTracking[w] = newExplored
        if(x + 1 < size_x): 
            e = (x + 1, y)
            if (not e in self._explored and not e in self._frontier and self._grid[e] != 1):
                self._frontier.append(e)
                self.pathTracking[e] = newExplored #pathtracking is a way of checking where this node is coming from
        if(y - 1 >= 0): 
            s = (x, y - 1)
            if (not s in self._explored and not s in self._frontier and self._grid[s] != 1):
                self._frontier.append(s)
                self.pathTracking[s] = newExplored
        if(y + 1 < size_y): 
            n = (x, y + 1)
            if (not n in self._explored and not n in self._frontier and self._grid[n] != 1):
                self._frontier.append(n)
                self.pathTracking[n] = newExplored

    def pathFinding(self) -> None:
        currentNode = self._goal #end point
        self._cost += 1 #keeping track of the number of steps or the path length
        while(self.pathTracking[currentNode] != -1):
            self._cost += 1  #cost increasing one for every node itself
            self._path.append(currentNode)
            currentNode = self.pathTracking[currentNode] #after adding the current it completes the track
        self._path.append(self._start)