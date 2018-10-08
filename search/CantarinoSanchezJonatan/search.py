# -*- coding: utf-8 -*-
#
# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The e projects and autograders were primarily created by John DeNero 
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

def depthFirstSearch(problem):
    stack = util.Stack()
    return searchPath(problem, stack)

def breadthFirstSearch(problem):
    queue = util.Queue()
    return searchPath(problem, queue)

#function that searches the goal through data structures
#the function will be used in dfs and bfs, the only implementation
#that changes is the datastructure itself 
#(stack for dfs and queue for bfs)
def searchPath(problem, dstructure):
    # as the state (node) components neded can differ on the problem
    #received as parameter, we will append more or less attributes
    dstructure.push((problem.getStartState(),[],0))   
    #visited list, used to ensure we don't mistake with infinite loops
    visited = []
    while not dstructure.isEmpty():
        current, path, cost = dstructure.pop()
        #if state == goal return path 
        if problem.isGoalState(current):
            return path
        #append current node to visited list
        if current not in visited:
            visited.append(current)
            #for every successor, push it to the datastructure keep the iteration
            for succ, dire, sCost in problem.getSuccessors(current):
                dstructure.push((succ, dire, sCost))

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    pQueue = util.PriorityQueue()
    pQueue.push((problem.getStartState(),[],0)0)
    visited=[]
    while not pQueue.isEmpty():
        current, path, cost = pQueue.pop()
        if problem.isGoalState(current):
            return path
        if current not in visited:
            visited.append(current)
            for succ, dire, sCost in problem.getSuccessors(current):
                pQueue.push((succ, path+[dire], sCost + cost + heuristic(succ, problem)))
                
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
    