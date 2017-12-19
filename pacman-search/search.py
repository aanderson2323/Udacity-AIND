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
wall = 999999
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
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]

  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:

  """
  from game import Directions
  from util import Stack
  def recursive_dls (frontier=None, explored=set()):
      if frontier == None:
          frontier = Stack()
          frontier.push((problem.getStartState(),[]))
      state, route = frontier.pop()
      if problem.isGoalState(state):
          #print "Goal!"
          #print route
          return route
      explored.add(state)
      #loop through posible actions
      for successor in problem.getSuccessors(state):
          successor_state, successor_direction, successor_cost = successor
          if successor_state not in explored:
              frontier.push((successor_state, route + [successor_direction]))
              solution = recursive_dls(frontier, explored)
              if solution:
                return solution
  return recursive_dls()

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  from util import Queue
  state = problem.getStartState()
  if problem.isGoalState(state):
      return route
  frontier = Queue()
  frontier.push((state,[]))
  explored = set()

  while frontier:
      state, action = frontier.pop()
      for next_node in problem.getSuccessors(state):
          next_state, next_direction, next_cost = next_node
          if next_state not in explored and next_state not in frontier.list:
              if problem.isGoalState(next_state):
                  return action + [next_direction]
              frontier.push((next_state, action+[next_direction]))
          explored.add(state)

def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  from util import PriorityQueue
  state = problem.getStartState()
  if problem.isGoalState(state):
      return route
  frontier = PriorityQueue()
  frontier.push((state,[]),0)
  explored = set()

  while frontier:
      #frontier.list.sort(key=lambda tup: tup[2])
      state, action = frontier.pop()
      for next_node in problem.getSuccessors(state):
          next_state, next_direction, next_cost = next_node
          if next_state not in explored and next_state not in frontier.heap:
              if problem.isGoalState(next_state):
                  return action + [next_direction]
              frontier.push((next_state, action+[next_direction]),problem.getCostOfActions(action+[next_direction]))
          explored.add(state)

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  from util import PriorityQueue
  state = problem.getStartState()
  if problem.isGoalState(state):
      return route
  frontier = PriorityQueue()
  frontier.push((state,[]),0)
  explored = set()

  while frontier:
      state, action = frontier.pop()
      for next_node in problem.getSuccessors(state):
          next_state, next_direction, next_cost = next_node
          if next_state not in explored and next_state not in frontier.heap:
              if problem.isGoalState(next_state):
                  return action + [next_direction]
              cost = heuristic(next_state,problem)
              frontier.push((next_state, action+[next_direction]),cost)
          explored.add(state)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
