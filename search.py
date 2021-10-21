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
    return [s, s, w, s, w, w, s, w]


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
    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()

    # Already there!
    if problem.isGoalState(start):
        print "Start is goal."
        return start

    parent_dict = dict()
    actions_dict = dict()
    cost_so_far_dict = dict()
    visited = set()
    pq = util.PriorityQueue()

    def make_solution(final_node):
        solution = list()
        this_node = final_node
        while parent_dict[this_node]:
            solution.insert(0, actions_dict[this_node])
            this_node = parent_dict[this_node]

        return solution

    # Initialize the children of start
    parent_dict[start] = None  # Start has no parent. :(
    children = problem.getSuccessors(start)

    if children:
        for child in children:
            parent_dict[child[0]] = start
            actions_dict[child[0]] = child[1]
            cost_so_far_dict[child[0]] = child[2]  # Start has no cost. So to get to a child, use its own cost.
            pq.push(child, child[2])
    # ----------------------------------

    while not pq.isEmpty():
        parent_node = pq.pop()
        children = problem.getSuccessors(parent_node[0])

        if children:
            for child in children:
                # If a child is this node's parent, or a child has already been visited, skip it
                if child[0] == parent_dict[parent_node[0]] or child[0] in visited:
                    continue

                # Mark visited nodes so we don't get stuck in infinite loops
                visited.add(child[0])
                parent_cost = cost_so_far_dict[parent_node[0]]
                child_cost = child[2]  # Cost to explore this child. Value is a single integer
                total_cost = parent_cost + child_cost

                # Only update the parents and actions if it's an improvement
                if child[0] not in cost_so_far_dict or total_cost < cost_so_far_dict[child[0]]:
                    parent_dict[child[0]] = parent_node[0]
                    actions_dict[child[0]] = child[1]
                    cost_so_far_dict[child[0]] = total_cost

                # Found a solution
                if problem.isGoalState(child[0]):
                    return make_solution(child[0])

                # If item already in priority queue with higher priority, update its priority and rebuild the heap.
                # If item already in priority queue with equal or lower priority, do nothing.
                # If item not in priority queue, do the same thing as pq.push.
                pq.update(child, total_cost)

    print "Could not find a solution :(\n"
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
