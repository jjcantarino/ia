# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
    Returns the start state for the search problem
    """
    util.raiseNotDefined()
    
def isGoalState(self, state):
    """
    state: Search state
    
    Returns True if and only if the state is a valid goal state
    """
    util.raiseNotDefined()

def getSuccessors(self, state):
    """
    state: Search state
     
    For a given state, this should return a list of triples,
    (successor, action, stepCost), where 'successor' is a
    successor to the current state, 'action' is the action
    required to get there, and 'stepCost' is the incremental
    cost of expanding to that successor
    """
    util.raiseNotDefined()

def getCostOfActions(self, actions):
    """
    actions: A list of actions to take
 
    This method returns the total cost of a particular sequence of actions.  The sequence must
    be composed of legal moves
    """
    util.raiseNotDefined()

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


# THIS IS A NEW CODE BY OLEKSII & HYUNGJOON
def getSolution(goalVertex, parentNodes, parentActions):
    '''
    This is a helper function to retrieve a solution path,
    because in the current implementation we are saving
    used memory by storing only one parent array
    - this is a hint from ACM ICPC experience
    '''

    resPath = []
    v = goalVertex

    # Just going up by parents
    while v in parentNodes:
        resPath.append(parentActions[v])
        v = parentNodes[v]

    # Now we need to reverse the path,
    # because we started from the goal
    resPath.reverse()

    return resPath

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first [p 85].

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    # THIS IS A NEW CODE BY OLEKSII & HYUNGJOON
    # TODO: Check wether there are some evidence that DFS is faster in connectivity detection

    exploredNodes = []
    # These dict's serves to retrieve the path
    trackParents = {}
    trackActions = {}

    # Get the beginning node of the given problem
    initialNode = problem.getStartState()
    if problem.isGoalState(initialNode):
        # If the first node is already a goal
        # then return nothing (just stay there)
        return []

    # Create the instance from Stack class to store nodes
    # Push the first node to explore its successors
    dfsStack = util.Stack()
    dfsStack.push(initialNode)

    # Do iteration until the stack is empty
    while not dfsStack.isEmpty():
        # Pop the node in a stack
        currentNode = dfsStack.pop()
        # Add the node to the exploredNodes list
        exploredNodes.append(currentNode)
        for (nextState, action, cost) in problem.getSuccessors(currentNode):
            # Only if this successor is not in the exploredNodes list
            if nextState not in exploredNodes:
                # Add this successor to the exploredNodes list
                exploredNodes.append(nextState)
                # Save tracking information -
                # from what parent and with what action we came
                trackParents[nextState] = currentNode
                trackActions[nextState] = action
                if problem.isGoalState(nextState):
                    # If a goal is found - return this solution
                    return getSolution(nextState, trackParents, trackActions)
                # Otherwise push a successor into the stack
                dfsStack.push(nextState)

    # We didn't find a solution, such assert
    # calls generalErrorDefined() and exit
    util.generalErrorDefined()

def breadthFirstSearch(problem):
    "Search the shallowest nodes in the search tree first. [p 81]"

    # THIS IS A NEW CODE BY OLEKSII & HYUNGJOON
    # BFS implementation is pretty much the same with DFS except using a queue instead of a stack
    # Again e apply iGoalTest while generating successors, that is explained in the textbook,
    # pages 81 - 82

    exploredNodes = []
    # These dict's serves to retrieve the path
    trackParents = {}
    trackActions = {}

    # Get the beginning node of the given problem
    initialNode = problem.getStartState()
    if problem.isGoalState(initialNode):
        # If the first node is already a goal
        # then return nothing (just stay there)
        return []

    # Create the instance from simple Queue class to store nodes
    # Push the first node to explore its successors
    bfsQueue = util.Queue()
    bfsQueue.push(initialNode)

    # Do iteration until the queue is empty
    while not bfsQueue.isEmpty():
        # Dequeue the node in a queue
        currentNode = bfsQueue.pop()
        # Add the node to the exploredNodes list
        exploredNodes.append(currentNode)
        for (nextState, action, cost) in problem.getSuccessors(currentNode):
            # Only if this successor is not in the exploredNodes list
            if nextState not in exploredNodes:
                # Add this successor to the exploredNodes list
                exploredNodes.append(nextState)
                # Save tracking information -
                # from what parent and with what action we came
                trackParents[nextState] = currentNode
                trackActions[nextState] = action
                if problem.isGoalState(nextState):
                    # If a goal is found - return this solution
                    return getSolution(nextState, trackParents, trackActions)
                 # Otherwise enqueue a successor into the queue
                bfsQueue.push(nextState)

    # We didn't find a solution, such assert
    # calls generalErrorDefined() and exit
    util.generalErrorDefined()


def uniformCostSearch(problem):
    "Search the node of least total cost first. "

    # THIS IS A NEW CODE BY OLEKSII & HYUNGJOON
    # ATTENTION: in UCS check for isGoal must be conducted
    # during analyzing the particular vertex retrieved from
    # the priority queue

    '''
    In fact, this is an implementation of Dijkstra's algorithm
    using priority queue. Graph complexity is O(MlogN)
    '''

    exploredNodes = []
    # These dict's serves to retrieve the path
    trackParents = {}
    trackActions = {}

    # We need to track the best currently known distance (cost)
    # to yet unexplored vertices
    currentDistance = {}

    # Get the beginning node of the given problem
    initialNode = problem.getStartState()
    if problem.isGoalState(initialNode):
        # If the first node is already a goal
        # then return nothing (just stay there)
        return []

    # ATTENTION: we have changed the code in util.py
    # to get priority back as well when pop an item!
    uscQueue = util.PriorityQueue()

    # Initialize the queue, each tuple contain vertex
    # and distance (cost) as a priority
    uscQueue.push(initialNode, 0)
    # Initialize the first known distance
    currentDistance[initialNode] = 0

    # Do iteration until the queue is empty
    while not uscQueue.isEmpty():
        # We retrieve a node with the best priority (cost or distance)
        (priority, node) = uscQueue.pop()
        '''
        This implementation uses the following trick -
        we cannot change priority of an element inside priority queue,
        but we need to do it during the relaxation phase.
        So indeed during relaxation we just add to the queue new values,
        not removing for a vertex its old priority in the queue.
        So we need to check for the following fictive nodes.
        More details can be found in Russian at e-maxx.ru :)
        '''
        # Fictive nodes will be omitted
        if priority > currentDistance[node]:
            # meaning an old one  - let's pop the next
            continue
        if problem.isGoalState(node):
            # If a goal is found - return this solution
            # Note, we should check it here! not while generating children
            # as in previous algorithms
            return getSolution(node, trackParents, trackActions)
        exploredNodes.append(node)
        for (nextState, action, cost) in problem.getSuccessors(node):
            if nextState in exploredNodes:
                # not interesting for us anymore
                continue
            # THE RELAXATION PHASE follows
            if nextState not in currentDistance or priority + cost < currentDistance[nextState]:
                uscQueue.push(nextState, priority + cost)
                currentDistance[nextState] = priority + cost
                # Update tracking information -
                # from what parent and with what action we came
                trackParents[nextState] = node
                trackActions[nextState] = action

    # We didn't find a solution, such assert
    # calls generalErrorDefined() and exit
    util.generalErrorDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."

    # THIS IS A NEW CODE BY OLEKSII & HYUNGJOON
    # aStarSearch is pretty much the same with uniformCostSearch
    # except using the lowest combined cost plus heuristic

    exploredNodes = []
    # These dict's serves to retrieve the path
    trackParents = {}
    trackActions = {}

    # We need to track the best currently known distance (cost)
    # to yet unexplored vertices
    currentDistance = {}

    # Get the beginning node of the given problem
    initialNode = problem.getStartState()
    if problem.isGoalState(initialNode):
        # If the first node is already a goal
        # then return nothing (just stay there)
        return []

    # ATTENTION: we have changed the code in util.py
    # to get priority back as well when pop an item!
    astarQueue = util.PriorityQueue()

    # Initialize the queue, each tuple contain vertex
    # and distance (cost) as a priority
    astarQueue.push(initialNode, 0)
    # Initialize the first known distance
    currentDistance[initialNode] = 0

    # Do iteration until the queue is empty
    while not astarQueue.isEmpty():
        # We retrieve a node with the best priority (cost or distance)
        (priority, node) = astarQueue.pop()
        # Fictive nodes will be omitted
        if priority > currentDistance[node]:
            # meaning an old one  - let's pop the next
            continue
        if problem.isGoalState(node):
            # If a goal is found - return this solution
            # Note, we should check it here! not while generating children
            # as in previous algorithms
            return getSolution(node, trackParents, trackActions)
        exploredNodes.append(node)
        for (nextState, action, cost) in problem.getSuccessors(node):
            if nextState in exploredNodes:
                # not interesting for us anymore
                continue
            # THE RELAXATION PHASE follows
            if nextState not in currentDistance or priority + cost + heuristic(nextState, problem) < currentDistance[nextState]:
                astarQueue.push(nextState, priority + cost + heuristic(nextState, problem))
                currentDistance[nextState] = priority + cost + heuristic(nextState, problem)
                # Update tracking information -
                # from what parent and with what action we came
                trackParents[nextState] = node
                trackActions[nextState] = action

    # We didn't find a solution, such assert
    # calls generalErrorDefined() and exit
    util.generalErrorDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
