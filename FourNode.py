import copy
import utils


class DStarLite:

    def __init__(self):
        self.nodeStart = "A"
        self.nodePrev = self.nodeStart
        self.nodeGoal = "G"
        self.path = [self.nodeStart]

        self.km = 0

        self.pq = utils.PriorityQueue()
        self.origEdgeCost = { "AB": 1, "BC": 1, "BD": 1, "CD": 1, "CG": 1, "DG": 10 }
        self.edgeCost = { "AB": 1, "BC": 1, "BD": 1, "CD": 1, "CG": 1, "DG": 10 }
        self.prevEdgeCost = { "AB": 1, "BC": 1, "BD": 1, "CD": 1, "CG": 1, "DG": 10 }
        self.gValue = { "A": float('inf'), "B": float('inf'), "C": float('inf'), "D": float('inf'), "G": float('inf') }
        self.rhs = { "A": float('inf'), "B": float('inf'), "C": float('inf'), "D": float('inf'), "G": 0 }
        self.heuristic = self.updateHeuristics() # To store the heuristic with respect to the current robotLoc(AKA nodeStart). ALSO need to update this whenever robot moves!

        self.pq.push( (self.nodeGoal, (self.heuristic[self.nodeGoal], 0)) )
        self.visitedNodes = []

    def getNeighbours(self, node):
        if (node == "A"):
            return ["B"]
        elif (node == "B"):
            return ["A", "C", "D"]
        elif (node == "C"):
            return ["B", "D", "G"]
        elif (node == "D"):
            return ["B", "C", "G"]
        elif (node == "G"):
            return ["C", "D"]
        else:
            return None

    # Return heuristic of all nodes from start node / robot node
    def updateHeuristics(self):
        if (self.nodeStart == "A"):
            return { "A": 0, "B": 1, "C": 2, "D": 2, "G": 3 }
        elif(self.nodeStart == "B"):
            return { "A": 1, "B": 0, "C": 1, "D": 1, "G": 2 }
        elif(self.nodeStart == "C"):
            return { "A": 2, "B": 1, "C": 0, "D": 1, "G": 1 }
        elif(self.nodeStart == "D"):
            return { "A": 2, "B": 1, "C": 1, "D": 0, "G": 1 }
        elif(self.nodeStart == "G"):
            return { "A": 3, "B": 2, "C": 1, "D": 1, "G": 0 }
        else:
            return None

    def updateObstacleEdges(self, nodeName):
        # Save prev Edge Cost
        self.prevEdgeCost = copy.deepcopy(self.edgeCost)
        # Find new Edge Cost (ie. reset all edges and change only those affected by latest obstacle position)
        changedEdgeNodes = self.getNeighbours(nodeName)
        self.edgeCost = copy.deepcopy(self.origEdgeCost)

        if(nodeName == "X"): # If no obstacles
            return []

        for nd in changedEdgeNodes:
            alphaEdge = utils.getAlphaOrder(nd, nodeName)
            self.edgeCost[alphaEdge] = float('inf')
        return changedEdgeNodes

    def calculateKeys(self, nodeName):
        k1 = min(self.gValue[nodeName], self.rhs[nodeName]) + self.heuristic[nodeName] + self.km
        k2 = min(self.gValue[nodeName], self.rhs[nodeName])
        return ( k1, k2 )

    def updateVertex(self, nodeName):

        if ( (self.gValue[nodeName] != self.rhs[nodeName]) and (nodeName in self.pq.getNames()) ):
            self.pq.update( (nodeName, self.calculateKeys(nodeName)) )

        elif ( (self.gValue[nodeName] != self.rhs[nodeName]) and (nodeName not in self.pq.getNames()) ):
            self.pq.push( (nodeName, self.calculateKeys(nodeName)) )

        elif ( (self.gValue[nodeName] == self.rhs[nodeName]) and (nodeName in self.pq.getNames()) ):
            #self.visitedNodes.append(nodeName)
            self.pq.remove( (nodeName, self.calculateKeys(nodeName)) )

    def computeShortestPath(self):
        # While queue has elements in it AND the top element in queue is LT key or startNode/robotNode 
        # OR rhs_startNode > gValue_startNode
        #self.visitedNodes = [] # Include this to prevent loops?
        while( ( (self.pq.queue) and self.pq.topKey() <= self.calculateKeys(self.nodeStart) ) \
                or self.rhs[self.nodeStart] > self.gValue[self.nodeStart] ):
            tmpNode = self.pq.top()
            u = tmpNode[0]
            kOld = tmpNode[1]
            kNew = self.calculateKeys(u)

            if(kOld < kNew):
                self.pq.update( (u, (kNew[0], knew[1])) )
            elif (self.gValue[u] > self.rhs[u]):
                self.gValue[u] = self.rhs[u]
                self.pq.remove(tmpNode)

                # get predecessors
                for pred in self.getNeighbours(u):
                    if(pred != "G"):
                        alphaEdge = utils.getAlphaOrder(u, pred)
                        self.rhs[pred] = min( self.rhs[pred], self.edgeCost[alphaEdge] + self.gValue[u] )
                        self.updateVertex(pred)
            else:
                gOld = self.gValue[u]
                self.gValue[u] = float('inf')

                # get predecessors and current node itself
                predecessors = self.getNeighbours(u)
                predecessors.append(u)
                for pred in predecessors: # represented as s in paper
                    alphaEdge = utils.getAlphaOrder(u, pred)
                     # Skip RHS update value and go on to queueUpdate step if dealing with predecessor = node u itself
                    #if ( (pred != u) and (pred not in self.visitedNodes) and (self.rhs[pred] == self.edgeCost[alphaEdge] + gOld) ):
                    if ( (pred != u) and (self.rhs[pred] == self.edgeCost[alphaEdge] + gOld) ):
                        if(pred != "G"):
                            rhsList = []
                            # update rhs to be min of all successors
                            for succ in self.getNeighbours(pred): # represented as s' in paper
                                alphaEdge = utils.getAlphaOrder(pred, succ)
                                rhsList.append(self.edgeCost[alphaEdge] + self.gValue[succ])
                            self.rhs[pred] = min(rhsList)
                    self.updateVertex(pred) # include this statement in the not in self.visitedNodes??


if __name__ == "__main__":
    dlite = DStarLite()
    #dlite.pq.display()

    print("RobotLoc: ", dlite.nodeStart)
    print("gValues: ", dlite.gValue)
    print("rhs: ", dlite.rhs)
    print("km: ", dlite.km)
    print("queue: ", dlite.pq.queue)
    print("Goal: ", dlite.nodeGoal)
    print("-----")
    dlite.computeShortestPath()

    while (dlite.nodeStart != dlite.nodeGoal):
        nextState  = "X"
        nextStateList = dlite.getNeighbours(dlite.nodeStart)
        nextStateCost = []
        for succ in nextStateList:
            alphaEdge = utils.getAlphaOrder(dlite.nodeStart, succ)
            nextStateCost.append(dlite.edgeCost[alphaEdge] + dlite.gValue[succ])
        idx = nextStateCost.index(min(nextStateCost))
        dlite.nodeStart = nextStateList[idx]
        print("RobotLoc: ", dlite.nodeStart)
        dlite.path.append(dlite.nodeStart)
        dlite.heuristic = dlite.updateHeuristics()


        # Scan the graph for changed edge costs
        obstNode = raw_input("Enter obstacle node:" )

        # Now update edgeCosts corsp to latest movement of obstacle
        changedEdgeNodes = dlite.updateObstacleEdges(obstNode)

        if (dlite.edgeCost != dlite.prevEdgeCost):
            
            # Update km
            dlite.km = dlite.km + dlite.heuristic[dlite.nodePrev]
            dlite.nodePrev = dlite.nodeStart

            # Update source nodes of all changed edges (includes the obstacle node if bidirectional)
            # NOTE: Implementing the RHS value update for all source nodes of changed edges differently from what paper gives
            # Following a more logical way of approaching it as given in the MIT video when obstacle appears at C
            changedEdgeNodes.insert(0, obstNode) # Insert rather than append used so that we start with the trivial obstacle node itself.
            for node in changedEdgeNodes:
                if (node != dlite.nodeGoal):
                    # Update RHS
                    rhsList = []
                    for neigh in dlite.getNeighbours(node):
                        alphaEdge = utils.getAlphaOrder(neigh, node)
                        rhsList.append( dlite.edgeCost[alphaEdge] + dlite.gValue[neigh] )
                    dlite.rhs[node] = min(rhsList)

                    # Update keys
                    dlite.updateVertex(node)

            # Perform best-first searach again as per priority queue
            dlite.computeShortestPath()

        #print("RobotLoc: ", dlite.nodeStart)
        print("gValues: ", dlite.gValue)
        print("rhs: ", dlite.rhs)
        print("km: ", dlite.km)
        print("queue: ", dlite.pq.queue)
        print("Goal: ", dlite.nodeGoal)
        print("-----")

    print("Path Followed by Robot: ", dlite.path)


# -----------------------------------------------------------------------------------------------------------

    #utils.testPriorityQueue()

# Error in slides, First iteration, C's keys were wrong. They should be 3,1 was given as 3,2 in slides
# In first loop, should A get dequed? In mine, it does not, so make while condition <= in the OR case?
# Inlcude visited Nodes list in computeShortPath() to prevent loops? ... I think this is already taken care of in if-else condition of updateVertex()


#utils.testPriorityQueue()