# searchAgents.py
# ---------------
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
This file contains all of the agents that can be selected to control Pacman.  To
select an agent, use the '-p' option when running pacman.py.  Arguments can be
passed to your agent using '-a'.  For example, to load a SearchAgent that uses
depth first search (dfs), run the following command:

> python pacman.py -p SearchAgent -a fn=depthFirstSearch

Commands to invoke other search strategies can be found in the project
description.

Please only change the parts of the file you are asked to.  Look for the lines
that say

"*** YOUR CODE HERE ***"

The parts you fill in start about 3/4 of the way down.  Follow the project
description for details.

Good luck and happy searching!
"""

from game import Directions
from game import Agent
from game import Actions
import util
import time
import search

#######################################################
# This portion is written for you, but will only work #
#       after you fill in parts of search.py          #
#######################################################

class SearchAgent(Agent):
    """
    This very general search agent finds a path using a supplied search
    algorithm for a supplied search problem, then returns actions to follow that
    path.

    As a default, this agent runs DFS on a PositionSearchProblem to find
    location (1,1)

    Options for fn include:
      depthFirstSearch or dfs
      breadthFirstSearch or bfs


    Note: You should NOT change any code in SearchAgent
    """

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):
        # Warning: some advanced Python magic is employed below to find the right functions and problems

        # Get the search function from the name and heuristic
        if fn not in dir(search):
            raise AttributeError, fn + ' is not a search function in search.py.'
        func = getattr(search, fn)
        if 'heuristic' not in func.func_code.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError, heuristic + ' is not a function in searchAgents.py or search.py.'
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))
            # Note: this bit of Python trickery combines the search algorithm and the heuristic
            self.searchFunction = lambda x: func(x, heuristic=heur)
            self.searchAlgoFlag = fn # Later used to identify if agent is dStarLite, which requries additional init conditions as compared to aStar

        # Get the search problem type from the name
        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError, prob + ' is not a search problem type in SearchAgents.py.'
        self.searchType = globals()[prob]
        print('[SearchAgent] using problem type ' + prob)
        self.walls = list()


    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        def getAllInternalStates(gameState):
            xStart = 1
            xStop = gameState.getMazeWidth() - 1 # For tinyMaze, mazeWidth = mazeHeight = 7
            yStart = 1
            yStop = gameState.getMazeHeight() - 1
            allStates = []
            for x in range(xStart, xStop): # For tiny maze, this gives a list from 1 to 5 for x and y
                for y in range(yStart, yStop):
                    allStates.append((x,y))
            return allStates

        # This function is a copy of getSuccessors(), written without the last portions that uses
        # self._expanded and other stuff, which is defined only in the search.py scope.
        def getSuccessorsTmp(node):
            """
            Returns successor states, the actions they require, and a cost of 1.

             As noted in search.py:
                 For a given state, this should return a list of triples,
             (successor, action, stepCost), where 'successor' is a
             successor to the current state, 'action' is the action
             required to get there, and 'stepCost' is the incremental
             cost of expanding to that successor
            """
            successors = []
            for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
                x,y = node
                dx, dy = Actions.directionToVector(action)
                nextx, nexty = int(x + dx), int(y + dy)
                if not self.walls[nextx][nexty]:
                    nextState = (nextx, nexty)
                    successors.append(nextState)
            return successors

        # Here, knowledge of all the walls is erased, since for our pacman only local walls are known
        # Thus, in A* Star, when neighbours/children of a node are found using the getSuccessor() function,
        # The function will return even the wal states, assuming the wall does not exist...UNLESS ofc, the robot
        # has previously gone near a particular wall and has updated the presence of that wall using the observe() function
        mazeWidth = state.getMazeWidth()
        mazeHeight = state.getMazeHeight()
        # Set all knowledge of walls to false.
        self.walls = [[False for i in xrange(mazeHeight)] for j in xrange(mazeWidth)]
        # Set boundary condition walls to True
        for x in xrange(mazeWidth):
            self.walls[x][0] = True
            self.walls[x][mazeHeight - 1] = True
        for y in xrange(mazeHeight):
            self.walls[0][y] = True
            self.walls[mazeWidth - 1][y] = True

        if self.searchFunction == None: raise Exception, "No search function provided for SearchAgent"
        starttime = time.time()

        # Additional variables required if dStarAlgorithm
        if(self.searchAlgoFlag == 'dlite'):
            self.firstLoop = True
            self.km = 0
            self.origEdgeCost = {}
            self.edgeCost = {}
            self.prevEdgeCost = {}
            self.gValue = {}
            self.rhs = {}
            self.pq = util.PriorityQueueDStarLite() # PrioorityQueue is carried forward through each iterations
            ## util.testPriorityQueueDStarLite()
            self.internalStates = getAllInternalStates(state)

            # Set Edge Costs
            for state1 in self.internalStates: # initially all costs are 1 (including wall positions)
                for state2 in getSuccessorsTmp(state1):
                        self.origEdgeCost[ util.setStr(state1, state2) ] = 1
                        self.edgeCost[ util.setStr(state1, state2) ] = 1
                        self.prevEdgeCost[ util.setStr(state1, state2) ] = 1
            ## Now go through self.walls and set edge cost to inf ?
            ## OR just exclude outermost walls always as part of general algorithm?

            # Set g and rhs values
            for tmpState in self.internalStates:
                self.gValue[tmpState] = float('inf')
                self.rhs[tmpState] = float('inf')
            #####self.rhs[problem.goal] = 0  # rhs of goal state is 0
            # Set heurisitc
            #####self.heuristic = self.updateHeuristics(problem.startState) # To store the heuristic with respect to the current robotLoc(AKA nodeStart). ALSO need to update this whenever robot moves!
            self.heuristic = {}
            # Update queue
            ####self.pq.push( (problem.goal, (self.heuristic[problem.goal], 0)) )
            self.pathSequence = []
            self.actionSequence = []
            #self.visitedNodes = [] Needed??

        problem = self.searchType(
            state, 
            self.walls, 
            self.searchAlgoFlag, 
            self.firstLoop,
            self.km,
            self.origEdgeCost,
            self.edgeCost,
            self.prevEdgeCost,
            self.pq,
            self.internalStates,
            self.gValue,
            self.rhs,
            self.heuristic,
            self.pathSequence,
            self.actionSequence) 
        self.rhs[problem.goal] = 0  # rhs of goal state is 0
        problem.rhs[problem.goal] = 0
        self.heuristic = problem.updateHeuristics(problem.startState) # To store the heuristic with respect to the current robotLoc(AKA nodeStart). ALSO need to update this whenever robot moves!
        problem.heuristic = problem.updateHeuristics(problem.startState)
        self.pq.push( (problem.goal, (self.heuristic[problem.goal], 0)) )
        #problem.pq.push( (problem.goal, (self.heuristic[problem.goal], 0)) ) # DO NOT ADD THIS, MAKES A DOUBLE COPY
        
        # Makes a new search problem
        self.actions  = self.searchFunction(problem) # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        pacPos = state.data.agentStates[0].getPosition()
        newProblem = self.searchType(state, 
                                    self.walls, 
                                    self.searchAlgoFlag, 
                                    self.firstLoop,
                                    self.km,
                                    self.origEdgeCost,
                                    self.edgeCost,
                                    self.prevEdgeCost,
                                    self.pq,
                                    self.internalStates,
                                    self.gValue,
                                    self.rhs,
                                    self.heuristic,
                                    self.pathSequence,
                                    self.actionSequence)
        self.rhs[newProblem.goal] = 0  # rhs of goal state is 0
        newProblem.rhs[newProblem.goal] = 0
        self.heuristic = newProblem.updateHeuristics(newProblem.startState) # To store the heuristic with respect to the current robotLoc(AKA nodeStart). ALSO need to update this whenever robot moves!
        newProblem.heuristic = newProblem.updateHeuristics(newProblem.startState)
        self.pq.push( (newProblem.goal, (self.heuristic[newProblem.goal], 0)) )
        #newProblem.pq.push( (newProblem.goal, (self.heuristic[newProblem.goal], 0)) ) # DO NOT ADD THIS, MAKES A DOUBLE COPY

        self.actions = self.searchFunction(newProblem)

        # Originally, would send all actions to be executed directly, since
        # agent knew the entire world and world was static
        #if 'actionIndex' not in dir(self): self.actionIndex = 0
        #i = self.actionIndex
        #self.actionIndex += 1
        #if i < len(self.actions):
        #    return self.actions[i]
        #else:
        #    return Directions.STOP

        # Now, need to send one action at a time, to allow for observation step by the agent
        if len(self.actions) > 0:
            return self.actions[0]
        else:
            return Directions.STOP

    def observationFunction(self, observation):
        pos = observation.data.agentStates[0].getPosition()

        for x in xrange(pos[0] - 1, pos[0] + 2):
            for y in xrange(pos[1] - 1, pos[1] + 2):
                self.walls[x][y] = observation.data.layout.isWall((x, y))

        return observation


class PositionSearchProblem(search.SearchProblem):
    """
    A search problem defines the state space, start state, goal test, successor
    function and cost function.  This search problem can be used to find paths
    to a particular point on the pacman board.

    The state space consists of (x,y) positions in a pacman game.

    Note: this search problem is fully specified; you should NOT change it.
    """

    def __init__(self, 
                gameState, 
                knownWalls,
                searchAlgoFlag, 
                firstLoop,
                km,
                origEdgeCost,
                edgeCost,
                prevEdgeCost,
                pq,
                internalStates,
                gValue,
                rhs,
                heuristic,
                pathSequence,
                actionSequence,
                costFn = lambda x: 1, goal=(1,1), 
                start=None, 
                warn=True, 
                visualize=True):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.walls = knownWalls
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn

        if(searchAlgoFlag=='dlite'):
            self.searchAlgoFlag = 'dlite'
            self.firstLoop = firstLoop
            self.km = km
            self.origEdgeCost = origEdgeCost
            self.edgeCost = edgeCost
            self.prevEdgeCost = prevEdgeCost
            self.gValue = gValue
            self.rhs = rhs
            self.pq = pq
            self.internalStates = internalStates
            self.heuristic = heuristic
            self.pathSequence = pathSequence
            self.actionSequence = actionSequence

        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print 'Warning: this does not look like a regular search maze'

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # For display purposes only
        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
                    __main__._display.drawExpandedCells(self._visitedlist) #@UndefinedVariable

        return isGoal

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """
        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1 # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)
        return successors

    # Function for dStarLite
    def updateHeuristics(self, nodeStart):
        dict = {}
        for state in self.internalStates:
            dict[state] = util.manhattanDistance(state, nodeStart)
        return dict
            
    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions. If those actions
        include an illegal move, return 999999.
        """
        if actions == None: return 999999
        x,y= self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost
        


def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

#####################################################
# This portion is incomplete.  Time to write code!  #
#####################################################

#class FoodSearchProblem:
#    """
#    A search problem associated with finding the a path that collects all of the
#    food (dots) in a Pacman game.

#    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
#      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
#      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
#    """
#    def __init__(self, startingGameState):
#        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
#        self.walls = [[False for i in xrange(startingGameState.getMazeWidth())] for j in xrange(startingGameState.getMazeHeight())]
#        self._hiddenWalls = startingGameState.getWalls()
#        self.startingGameState = startingGameState
#        self._expanded = 0 # DO NOT CHANGE
#        self.heuristicInfo = {} # A dictionary for the heuristic to store information

#    def getStartState(self):
#        return self.start

#    def isGoalState(self, state):
#        return state[1].count() == 0

#    def getSuccessors(self, state):
#        "Returns successor states, the actions they require, and a cost of 1."
#        successors = []
#        self._expanded += 1 # DO NOT CHANGE
#        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
#            x,y = state[0]
#            dx, dy = Actions.directionToVector(direction)
#            nextx, nexty = int(x + dx), int(y + dy)
#            if not self.walls[nextx][nexty]:
#                nextFood = state[1].copy()
#                nextFood[nextx][nexty] = False
#                successors.append( ( ((nextx, nexty), nextFood), direction, 1) )
#        return successors

#    def getCostOfActions(self, actions):
#        """Returns the cost of a particular sequence of actions.  If those actions
#        include an illegal move, return 999999"""
#        x,y= self.getStartState()[0]
#        cost = 0
#        for action in actions:
#            # figure out the next state and see whether it's legal
#            dx, dy = Actions.directionToVector(action)
#            x, y = int(x + dx), int(y + dy)
#            if self.walls[x][y]:
#                return 999999
#            cost += 1
#        return cost

#class AStarFoodSearchAgent(SearchAgent):
#    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
#    def __init__(self):
#        self.searchFunction = lambda prob: search.aStarSearch(prob, foodHeuristic)
#        self.searchType = FoodSearchProblem

def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    position, foodGrid = state
    "*** YOUR CODE HERE ***"
    # Trying Dijkstra graph
    # Relax the problem by using distance costs that ignore walls.
    # Food locations and starting position are nodes on the graph,
    # edges are the manhattan distance from one node to another (again, ignoring walls)
    # When Dijkstra is done, the longest distance in the set will
    # represent the total heuristic.

    unvisited = util.PriorityQueue()
    dist = {position: 0}
    parent = {}
    graph = [position] + foodGrid.asList()
    for node in graph:
        if not node == position:
            dist[node] = 99999
            parent[node] = ()
        unvisited.push(node, dist[node])

    while not unvisited.isEmpty():
        aNode = unvisited.pop()
        for bNode in graph:
            tempdist = dist[aNode] + abs(bNode[0] - aNode[0]) + abs(bNode[1] - aNode[1])
            if tempdist < dist[bNode]:
                dist[bNode] = tempdist
                parent[bNode] = aNode
                unvisited.update(bNode, dist[bNode])

    hval = 0
    for pathLen in dist:
        hval = max(hval, dist[pathLen])

    return hval

    return len(foodGrid.asList())

class ClosestDotSearchAgent(SearchAgent):
    "Search for all food using a sequence of searches"
    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while(currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState) # The missing piece
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception, 'findPathToClosestDot returned an illegal move: %s!\n%s' % t
                currentState = currentState.generateSuccessor(0, action)
        self.actionIndex = 0
        print 'Path found with cost %d.' % len(self.actions)

    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition()
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState)

        "*** YOUR CODE HERE ***"
        return search.aStarSearch(problem)

        util.raiseNotDefined()

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state

        "*** YOUR CODE HERE ***"
        if self.food[x][y]:
            return True
        else:
            return False

        util.raiseNotDefined()

def mazeDistance(point1, point2, gameState):
    """
    Returns the maze distance between any two points, using the search functions
    you have already built. The gameState can be any game state -- Pacman's
    position in that state is ignored.

    Example usage: mazeDistance( (2,4), (5,6), gameState)

    This might be a useful helper function for your ApproximateSearchAgent.
    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.bfs(prob))



##?? Might need to init all known walls to False...even the border walls, so that true sense of not knowing wall boundaries exists, AND
# edge costs initialized will be accurate.
##?? Exclude outermost walls as part of general algorithm? OR set that to inf in the graph? Maybe exclude in algo so that weird node resets don't happen