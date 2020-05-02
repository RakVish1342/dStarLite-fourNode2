# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    pathCostDict = {}
    prevSeen = []  # do not push nodes found in this list. "Discovered" nodes
    Fringe = util.PriorityQueue()
    startState = problem.getStartState()
    Fringe.push(startState, heuristic(startState, problem))
    pathCostDict[startState] = [[], 0]
    while not Fringe.isEmpty():
        # choose a child node from the fringe
        currentState = Fringe.pop()
        prevSeen.append(currentState)
        # if the child is a goal state return a solution
        if problem.isGoalState(currentState):
            #print(">>>", pathCostDict[currentState][0])
            return pathCostDict[currentState][0]
        else:
            # expand the node
            # Note: getSuccessor might return even the wall states. Since registerInitialState() and observe()
            # functions are being used to only keep track of walls that have been encountered by the bot.
            children = problem.getSuccessors(currentState)
            # for each successor add to fringe iff it is not already expanded or in the fringe.

            for child in children:
                if child[0] not in prevSeen:
                    # for the child state append the action set to the current state
                    # with the action to the child.
                    h_val = heuristic(child[0], problem)
                    g_val = pathCostDict[currentState][1] + child[2]
                    f_val = h_val + g_val
                    if child[0] not in pathCostDict or g_val < pathCostDict[child[0]][1]:
                        # Currently storing the Manhattan path (ie. ignoring walls) to the given node due to local observability
                        # In pacman A*, where full awareness of the map was present, each step of this algo would store the true
                        #  path to that node (with wall considerations), and  nt the manhattan path.
                        pathCostDict[child[0]] = [pathCostDict[currentState][0]+[child[1]], g_val]

                    Fringe.update(child[0], f_val)
                    # For graph search to avoid loops add the child to the "discovered" node list
    return []


def dStarLite(problem, heuristic=nullHeuristic):
    
    ### Defining some functions
    def updateObstacleEdges(nodeName):

        # make copy old edge costs
        
        # nodeName = obst location
        # if nodeName in state.getWalls
        # set inf
        
        # Save prev Edge Cost
        problem.prevEdgeCost = copy.deepcopy(problem.edgeCost)
        # Find new Edge Cost (ie. reset all edges and change only those affected by latest obstacle position)
        changedEdgeNodes = problem.getSuccessors(nodeName)
        problem.edgeCost = copy.deepcopy(problem.origEdgeCost)

        if(nodeName == "X"): # If no obstacles
            return []

        for nd in changedEdgeNodes:
            edgeStr = util.setStr(nd, nodeName)
            problem.edgeCost[edgeStr] = float('inf')
        return changedEdgeNodes

    def calculateKeys(nodeName):
        k1 = min(problem.gValue[nodeName], problem.rhs[nodeName]) + problem.heuristic[nodeName] + problem.km
        k2 = min(problem.gValue[nodeName], problem.rhs[nodeName])
        return ( k1, k2 )

    def updateVertex(nodeName):

        if ( (problem.gValue[nodeName] != problem.rhs[nodeName]) and (nodeName in problem.pq.getNames()) ):
            problem.pq.update( (nodeName, calculateKeys(nodeName)) )

        elif ( (problem.gValue[nodeName] != problem.rhs[nodeName]) and (nodeName not in problem.pq.getNames()) ):
            problem.pq.push( (nodeName, calculateKeys(nodeName)) )

        elif ( (problem.gValue[nodeName] == problem.rhs[nodeName]) and (nodeName in problem.pq.getNames()) ):
            #problem.visitedNodes.append(nodeName)
            problem.pq.remove( (nodeName, calculateKeys(nodeName)) )

    def computeShortestPath(problem):
    # While queue has elements in it AND the top element in queue is LT key or startNode/robotNode 
    # OR rhs_startNode > gValue_startNode
    #problem.visitedNodes = [] # Include this to prevent loops?
        while( ( bool(problem.pq.queue) and problem.pq.topKey() <= calculateKeys(problem.startState) ) \
                or problem.rhs[problem.startState] > problem.gValue[problem.startState] ):
            tmpNode = problem.pq.top()
            u = tmpNode[0]
            kOld = tmpNode[1]
            kNew = calculateKeys(u)

            if(kOld < kNew):
                problem.pq.update( (u, (kNew[0], knew[1])) )
            elif (problem.gValue[u] > problem.rhs[u]):
                problem.gValue[u] = problem.rhs[u]
                problem.pq.remove(tmpNode)

                # get predecessors
                for pred,_action,_cost in problem.getSuccessors(u):      # ((x1,y1), (x2, y2))
                    if(pred != problem.goal):
                        edgeStr = util.setStr(u, pred)
                        problem.rhs[pred] = min( problem.rhs[pred], problem.edgeCost[edgeStr] + problem.gValue[u] )
                        updateVertex(pred)
            else:
                gOld = problem.gValue[u]
                problem.gValue[u] = float('inf')

                # get predecessors and current node itself
                predecessors = problem.getSuccessors(u)
                predecessors.append((u,None,None))
                for pred,_action,_cost in predecessors: # represented as s in paper
                    edgeStr = util.setStr(u, pred)
                        # Skip RHS update value and go on to queueUpdate step if dealing with predecessor = node u itself
                    #if ( (pred != u) and (pred not in problem.visitedNodes) and (problem.rhs[pred] == problem.edgeCost[edgeStr] + gOld) ):
                    if ( (pred != u) and (problem.rhs[pred] == problem.edgeCost[edgeStr] + gOld) ):
                        if(pred != problem.goal):
                            rhsList = []
                            # update rhs to be min of all successors
                            for succ,_action,_cost in problem.getSuccessors(pred): # represented as s' in paper
                                edgeStr = util.setStr(pred, succ)
                                rhsList.append(problem.edgeCost[edgeStr] + problem.gValue[succ])
                            problem.rhs[pred] = min(rhsList)
                    updateVertex(pred) # include this statement in the not in problem.visitedNodes??
    
    ### Actual code of the function starts here
    # This step is to be performed once at the beginning only, right after initialization of all param
    if(problem.firstLoop):
        computeShortestPath(problem)
        problem.firstLoop = False

    # Take the next step
    nextStateList = problem.getSuccessors(problem.startState)
    nextStateCost = []
    for succ,_action,_cost in nextStateList:
        edgeStr = util.setStr(problem.startState, succ)
        nextStateCost.append(problem.edgeCost[edgeStr] + problem.gValue[succ])
    idx = nextStateCost.index(min(nextStateCost))
    problem.startState, action, _cost = nextStateList[idx]
    #print("RobotLoc: ", problem.startState)
    problem.pathSequence.append(problem.startState)
    problem.actionSequence.append(action)

    # Update heuristic since robot has moved now
    problem.heuristic = problem.updateHeuristics(problem.startState)

    return [action] # Converting to list, since in dStarLite only one action will be calculated per time step












# Abbreviations
astar = aStarSearch
dlite = dStarLite
