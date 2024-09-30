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
from game import Directions
from typing import List

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

#==================================================================================

def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

#===================================================================================================

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """
    - Search the deepest nodes in the search tree first.
    - Your search algorithm needs to return a list of actions that reaches goal. 
    - Make sure to implement a graph search algorithm.
    """
    #print("Start:", problem.getStartState())
    #print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    #print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    def recursiveDFS(state, actions, explored):
        if problem.isGoalState(state):                     
            return actions
        explored.add(state)                                        # Mark as Explored
        for next_state, action, _ in problem.getSuccessors(state): # Explore Successor
            if next_state not in explored:
                result = recursiveDFS(next_state, actions + [action], explored)
                if result:                            
                    return result
        return None                                        

    startState = problem.getStartState()                           # Start state
    exploredNodes = set()                                          # Tracking Explored Paths
    return recursiveDFS(startState, [], exploredNodes) or []   

    # TESTS:
    # python pacman.py -l tinyMaze -p SearchAgent
    # python pacman.py -l mediumMaze -p SearchAgent
    # python pacman.py -l bigMaze -z .5 -p SearchAgent

    """    NON RECURSIVE VERSION SCORED BETTER IN MEDIUM MAZE hm    """



#===================================================================================================

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""

    startState = problem.getStartState()
    queue = util.Queue()
    queue.push((startState, [])) 
    
    exploredNodes = set()
    
    while not queue.isEmpty():
        current, actions = queue.pop() 
        
        if problem.isGoalState(current):
            return actions
        
        if current not in exploredNodes:
            exploredNodes.add(current)
            
            for next, action, _ in problem.getSuccessors(current): # next state, action, _ ignore param
                if next not in exploredNodes and next not in (node[0] for node in queue.list):
                    newAction = actions + [action]
                    queue.push((next, newAction))
    
    return []

#==================================================================================

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    startState = problem.getStartState()
    pqueue = util.PriorityQueue()
    pqueue.push((startState, [], 0), 0) 
    
    exploredNodes = set()
    
    while not pqueue.isEmpty():
        current, actions, cost = pqueue.pop()
        
        if problem.isGoalState(current):
            return actions
        
        if current not in exploredNodes:
            exploredNodes.add(current)
            
            for next, action, stepCost in problem.getSuccessors(current):
                if next not in exploredNodes:
                    newActions = actions + [action]
                    newCost = cost + stepCost
                    pqueue.update((next, newActions, newCost), newCost)
    
    return []

#==================================================================================

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

#==================================================================================

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    startState = problem.getStartState()
    pqueue = util.PriorityQueue()
    heuristic0 = heuristic(startState, problem) # heuristic value for start state
    pqueue.push((startState, [], 0), heuristic0)  # param (state, actions, gCost), fCost

    exploredNodes = {}  

    while not pqueue.isEmpty():
        current, actions, gCost = pqueue.pop()

        if problem.isGoalState(current):
            return actions

        if current not in exploredNodes or gCost < exploredNodes[current]:
            exploredNodes[current] = gCost

            for next, action, stepCost in problem.getSuccessors(current):
                newActions = actions + [action]
                newGcost = gCost + stepCost
                newFcost = newGcost + heuristic(next, problem)
                pqueue.update((next, newActions, newGcost), newFcost)

    return []

#==================================================================================

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
