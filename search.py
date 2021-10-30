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
import math
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
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"

    # Locals
    startState = problem.getStartState()
    pathStack = util.Stack()
    visited = []

    def dfsHelper(state):
        visited.append(state)
        # Goal State Found
        if problem.isGoalState(state):
            return True
        # Goal State Not Found
        elif pathStack.isEmpty() and state != problem.getStartState():
            pathStack.push("NONE")
            return False
        # Still Searching
        else:
            # Expand Node
            successors = problem.getSuccessors(state)
            for s in successors:
                candidateState = s[0]
                # Unvisited State
                if candidateState not in visited:
                    pathStack.push(s)
                    if dfsHelper(candidateState):
                        return True
            # Dead-End Reached
            pathStack.pop()
            return False

    # Find Directions & Parse from Stack
    if dfsHelper(startState) == False:
        return []
    else:
        directionsList = []
        while not pathStack.isEmpty():
            directionsList.append(pathStack.pop())
        directionsList.reverse()
    return list(map(lambda x : x[1], directionsList))


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    visited = {}
    frontier = util.Queue()
    path = []
    goalFound = False

    def traceBack(lastAction, parent):
        path.append(lastAction)
        state = parent
        while (visited[state][0] is not None):
            path.append(visited[state][1])
            state = visited[state][0]
        path.reverse()
        return path 

    state = problem.getStartState()
    if (problem.isGoalState(state)):
        return path
    else:
        frontier.push(state) 
        visited[state] = (None, None, 0)

        while (not frontier.isEmpty()):
            state = frontier.pop()

            if (state is not None):
                children = problem.getSuccessors(state)
                for child in children:
                    child_state = child[0]
                    child_action = child[1]
                    child_cost = child[2]

                    if (child_state not in visited) and not goalFound:
                        if (problem.isGoalState(child_state)):
                            goalFound = True
                            traceBack(child_action, state)
                        else:
                            frontier.push(child_state)
                            visited[child_state] = (state, child_action, child_cost)

    return path


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"


    def traceBack(child_action, parent):
        solution = []

        # append last action to Goal to solution
        solution.append(child_action)
        while parent is not start:
            # append current action to solution
            parent_action = explored[parent][0]
            solution.append(parent_action)
            # update parent
            parent = explored[parent][1]

        solution.reverse()
        return solution

    frontier = util.PriorityQueue()
    explored = {}
    visited = {}
    goalFound = False
    start = problem.getStartState()

    if problem.isGoalState(start):
        traceBack(start, None)
    else:
        # pushing priority value 0 for start position
        frontier.push(start, 0)
        # key: state_position, value: [f, par, act]
        visited[start] = (0, None, None)

        while (not frontier.isEmpty()) and (not goalFound):
            # get the current state
            state = frontier.pop()

            # Got to the goal
            if (problem.isGoalState(state)):
                goalFound = True
                print("goal is found")
                path = traceBack(visited[state][2], visited[state][1])

            else:
                # stats of the current state
                g = visited[state][0]
                parent = visited[state][1]
                action = visited[state][2]

                # add the current node to explored
                explored[state] = (action, parent)
                
                children = problem.getSuccessors(state)
                for child in children:
                    child_state = child[0]
                    child_action = child[1]
                    child_cost = child[2]

                    # How far it actually took to get to child_g                    
                    f = g + child_cost

                    # new node found
                    if (child_state not in explored) and (child_state not in visited):
                        frontier.push(child_state, f)
                        visited[child_state] = (f, state, child_action)
                    
                    # it's something that we already put in pq
                    # but we got cheaper route to goal 
                    elif (child_state in visited) and (visited[child_state][0] > f):
                        visited[child_state] = (f, state, child_action) 
                        frontier.update(child_state, f)   
    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    def traceBack(child_action, parent):
        solution = []

        # append last action to Goal to solution
        solution.append(child_action)
        while parent is not start:
            # append current action to solution
            parent_action = explored[parent][0]
            solution.append(parent_action)
            # update parent
            parent = explored[parent][1]

        solution.reverse()
        return solution

    frontier = util.PriorityQueue()
    explored = {}
    visited = {}
    goalFound = False
    start = problem.getStartState()

    if problem.isGoalState(start):
        traceBack(start, None)
    else:
        # pushing priority value 0 for start position
        frontier.push(start, 0)

        # key: state_position, value: [g, f, par, act]
        visited[start] = (0, 0, None, None)

        while (not frontier.isEmpty()) and (not goalFound):
            # get the current state
            state = frontier.pop()

            # Got to the goal
            if (problem.isGoalState(state)):
                goalFound = True
                path = traceBack(visited[state][3], visited[state][2])

            else:
                # stats of the current state
                g = visited[state][0]
                parent = visited[state][2]
                action = visited[state][3]

                # add the current node to explored
                explored[state] = (action, parent)
                
                children = problem.getSuccessors(state)
                for child in children:
                    child_state = child[0]
                    child_action = child[1]
                    child_cost = child[2]

                    # Calculate heuristic
                    h = heuristic(child_state, problem)

                    # How far it actually took to get to child_g
                    child_g = g + child_cost

                    # Total cost so far + heuristic estimate
                    f = child_g + h

                    # new node found
                    if (child_state not in explored) and (child_state not in visited):
                        frontier.push(child_state, f)
                        visited[child_state] = (child_g, f, state, child_action)
                    
                    # it's something that we already put in pq
                    # but we got cheaper route to goal 
                    elif (child_state in visited) and (visited[child_state][1] > f):
                        visited[child_state] = (child_g, f, state, child_action) 
                        frontier.update(child_state, f)     
    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
