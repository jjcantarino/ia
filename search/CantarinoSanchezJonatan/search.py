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
    frontier = util.Stack()
    return searchGoal(problem, frontier)

#function that searches the goal through data structures
#the function will be used in dfs and bfs, the only implementation
#that changes is the datastructure itself frontier 
#(stack for dfs and queue for bfs)

def searchGoal(problem, frontier):
    # as the state (node) components neded can differ on the problem
    #received as parameter, we will append more or less attributes
    frontier.push((problem.getStartState(),False,0))   
    #visited list, used to ensure we don't mistake with infinite loops
    visited = []
    #parents dictionary, given a child key returns a father value
    #so we can reconstruct backwards our path till the solution
    parents_dict = dict()
    while not frontier.isEmpty():
        current = frontier.pop()
        visited.append(current)
        #if state == goal find our path 
        if problem.isGoalState(current):
            path = []
            aux = current
            while parents_dict.has_key(aux):    
                path.append(aux[1])
                aux = parents_dict[aux]
            path.reverse()
            return path
        #for every child that node has, push it to frontier
        #and link the relationship
        for successor in problem.getSuccessors(current):
            if successor not in visited:
                parents_dict[successor] = current
                frontier.push(successor)
    return []

def breadthFirstSearch(problem):
    frontier = util.Queue()
    return searchGoal(problem, frontier)

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
    from collections import defaultdict
    #default value of infinite in every key of the dictionary
    #so we can update it easily
    cost = defaultdict(lambda x : 9999999)
    startNode = problem.getStartState()
    cost[startNode] = 0
    #we will use a priority queue that differs on the cost of the path through an heuristic
    queue = util.PriorityQueueWithFunction(lambda x : cost[x]+heuristic(x,problem))
    queue.push(startNode)
    visited = []
    parents_dict = dict()
    while not queue.isEmpty():
        current = queue.pop()
        visited.append(current)
        if problem.isGoalState(current):
            path = []
            aux = current
            while parents_dict.has_key(aux):
                path.append(aux[1])
                aux = parents_dict[aux]
            path.reverse()
            return path
        #in the case of astar, we need to check actual cost of successor and update it
        #if we found a better way into it, similar to dijkstras algorythm
        for successor in problem.getSuccessors(current):       
            if successor not in visited or cost[successor] > cost[current] + successor[2] : 
                visited.append(successor)
                parents_dict[successor] = current
                cost[successor] = cost[current] + successor[2]
                queue.push(successor)    
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
    