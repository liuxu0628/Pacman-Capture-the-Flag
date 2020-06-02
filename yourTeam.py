# myTeam.py
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


from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util, sys
from game import Directions
import game
from game import Actions
from util import nearestPoint

import math

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'ReflexAgent', second = 'ReflexAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class IntelligentAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    self.midWidth = int(gameState.data.layout.width / 2)
    self.height = int(gameState.data.layout.height)
    self.start = gameState.getAgentPosition(self.index)
    self.walls = gameState.getWalls().asList()
    self.food = self.getFood(gameState).asList()
    print(self.getFoodYouAreDefending(gameState).asList())
    self.slipProb = 0.8
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''

  def manhattanDistance(position, goal):
    (x, y) = position
    (gx, gy) = goal
    return abs(gx - x) + abs(gy - y)

  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  def boundaryPosition(self, gameState):
    ''''
    return a list of positions of boundary
    '''
    myState = gameState.getAgentState(self.index)
    myPosition = myState.getPosition()
    boundaries = []
    if self.red:
      i = self.midWidth - 1
    else:
      i = self.midWidth + 1
    boudaries = [(i,j) for j in  range(self.height)]
    validPositions = []
    for i in boudaries:
      if not gameState.hasWall(i[0],i[1]):
        validPositions.append(i)
    return validPositions


class ReflexAgent(IntelligentAgent):
  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)
    myPos = successor.getAgentState(self.index).getPosition()

    # actions to eat foods
    foodList = self.getFood(successor).asList()
    action, actionValue = self.waStarSearch(gameState, foodList, 2, True)
    foodAction, features['foodScore'] = action, actionValue

    # actions to eat invaders
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    action, actionValue = self.waStarSearch(gameState, invaders, 0, False)
    invadersAction, features['invadersScore'] = (action, actionValue)

    # actions to return border
    borders = self.boundaryPosition(gameState)
    action, actionValue = self.waStarSearch(gameState, borders, len(borders)-1, False)
    bordersAction, features['bordersScore'] = action, actionValue

    # actions to eat capsules
    capsules = self.getCapsules(gameState)
    action, actionValue = self.waStarSearch(gameState, capsules, len(capsules)-1, False)
    capsuleAction, features['capsuleScore'] = action, actionValue
    # actions to dodge ghosts
    ghosts = [a for a in enemies if not a.isPacman and a.getPosition() != None]
    # action, actionValue = self.waStarSearch(gameState, ghosts, heuristic, False)
    # features['ghostScore'] = (action, actionValue)
    features['ghostScore'] = 9999

    weights = self.getWeights(gameState, action)
    return features

  def getWeights(self, gameState, action):
    return {'foodScore': -1, 'invadersScore': -1, 'bordersScore': -1, 'capsuleScore': -1.5, 'ghostScore': -1}

  def chooseAction(self, gameState):

    x, y = self.getCurrentObservation().getAgentPosition(self.index)
    if x < self.midWidth - 1:
      boarder_goal = self.getBoarderGoal(gameState)
      action = self.aStarSearch(gameState, boarder_goal, self.simpleHeuristic)
      return action

    Cp = 0.5
    gamma = 0.9
    limit = 100
    time = 0
    actions = [Directions.EAST, Directions.WEST, Directions.SOUTH, Directions.NORTH]
    MCT = {}
    UCT = {}
    for action in actions:
      slipActions = self.allSlipOver(action)
      dict = {}
      for slipAction in slipActions:
        dict[slipAction] = [0,0]
      MCT[action] = dict
      UCT[action] = [0,0]
    legalAction = gameState.getLegalActions(self.index)
    while time < limit:
      highest = 0
      for a in UCT.keys():
        value, times = UCT[a]
        if times == 0:
          action = a
          break
        else :
          if not time < 3:
            pi = value + 2 *Cp* math.sqrt(2*math.log(time)/times)
            if pi >= highest:
              highest = pi
              action = a

      slipAction = self.slipOver(action)
      value, times = MCT[action][slipAction]
      sumValue = times * value

      if action in legalAction:
        newGameState = self.getSuccessor(gameState, action)
      else:
        newGameState = gameState

      # if slipAction in legalAction:
      #   newGameState = self.getSuccessor(gameState, slipAction)
      # else:
      #   newGameState = gameState
      times += 1
      sumValue += self.simulation(gamma, newGameState)

      value = sumValue / times
      MCT[action][slipAction] = [value, times]
      V = 0
      for a in MCT[action].keys():
        if a == action:
          value, times = MCT[action][a]
          V += self.slipProb * value
        else :
          V += (1-self.slipProb)/2 * value
      v, t = UCT[action]
      UCT[action] = [V, t+1]
      time +=1
    highest = -99999
    for a in UCT.keys():
      v, t = UCT[a]
      if highest <= v:
        highest = v
        bestAct = a
    bestAct = self.slipOver(bestAct)
    if bestAct in legalAction:
     return bestAct
    else:
      return Directions.STOP

  def boarderPosition(self, gameState):
    """
    return a list of positions of boundary
    """
    if self.red:
      i = self.midWidth - 1
    else:
      i = self.midWidth + 1
    boudaries = [(i, j) for j in range(self.height)]
    validPositions = []
    for i in boudaries:
      if not gameState.hasWall(i[0], i[1]):
        validPositions.append(i)
    return validPositions

  def getBoarderGoal(self, gameState):
        boards = self.boarderPosition(gameState)
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if len(boards) > 0:
            dis = 9999
            nearestBoarder = boards[0]
            for b in boards:
                temp = self.getMazeDistance(myPos, b)
                if temp < dis:
                    dis = temp
                    nearestBoarder = b
            return nearestBoarder
        else:
            return None

  def simpleHeuristic(self, gameSate, currentPos):
    goal = self.getBoarderGoal(gameSate)
    return self.getMazeDistance(currentPos, goal)

  def aStarSearch(self, gameState, goal, heuristic=simpleHeuristic):
    openList = util.PriorityQueue()
    closedList = []
    initialState = self.getCurrentObservation().getAgentState(self.index).getPosition()
    openList.push((initialState, []), 0)
    closedList.append([])

    while not openList.isEmpty():
      state, trace = openList.pop()
      if state == goal:
        if len(trace) == 0:
          return 'Stop'
        return trace[0]

      if state not in closedList:
        closedList.append(state)
        successors = self.getSuccessors(state)
        for successor in successors:
          action = successor[0]
          nextPos = successor[1]
          cost = len(trace + [nextPos])
          nextHeuristic = cost + heuristic(gameState, nextPos)
          if successor not in closedList:
            openList.push((nextPos, trace + [action]), nextHeuristic)

    return 'Stop'

  def getSuccessors(self, currentPosition):
    successors = []
    walls = self.walls
    for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
      x, y = currentPosition
      dx, dy = Actions.directionToVector(action)
      nextx, nexty = int(x + dx), int(y + dy)
      if (nextx, nexty) not in walls:
        nextPosition = (nextx, nexty)
        successors.append((action, nextPosition))
    return successors

  def simulation(self, gamma, gameState):
    limit= 20
    time = 0
    enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]

    ghosts = [a for a in enemies if not a.isPacman and a.getPosition() != None]
    enemiesPosition = [ghost.getPosition() for ghost in ghosts]

    scoreStart = gameState.getScore()
    foodStart = len(self.getFood(gameState).asList())

    preAction = None

    while time < limit:
        time += 1

        gameState, preAction = self.randomMove(preAction, gameState)
        position = gameState.getAgentPosition(self.index)
        if not enemiesPosition == None and position in enemiesPosition:
          foodFinish = len(gameState.getFood.asList())
          score = (-1 -1 * foodStart - foodFinish)*(math.pow(gamma,time))
          if score < 0:
            print(score)
          return score

        if len(self.getFood(gameState).asList()) <= 2:
          scoreGet = gameState.getScore() - scoreStart
          foodCarry = gameState.getAgentState(self.index).numCarrying
          if foodCarry == 0:
            score =(100 + 1 * scoreGet) * (math.pow(gamma,time))
            if score < 0:
              print(score)
            return score
    border = self.boundaryPosition(gameState)
    minDis = 99999
    # for (x,y) in border:
    #   dis = self.getMazeDistance(gameState.getAgentState(self.index).getPosition(), (x,y))
    #   if dis < minDis:
    #     minDis =dis
    foodFinish = len(self.getFood(gameState).asList())
    foodEaten = foodFinish - foodStart
    scoreGet = gameState.getScore() - scoreStart
    foodCarry = self.getCurrentObservation().getAgentState(self.index).numCarrying
    score = (foodCarry * 1 + scoreGet * 5) * (math.pow(gamma,time))
    if score<0:
      print(score)
    return score

  def randomMove(self, preAction, gameState):
    actions = gameState.getLegalActions(self.index)
    actions.remove(Directions.STOP)
    if not preAction == None and len(actions) > 1 and Directions.REVERSE[preAction] in actions:
      preAction = Directions.REVERSE[preAction]
      actions.remove(preAction)

    action = random.choice(actions)
    gameState = self.getSuccessor(gameState, action)
    # print (gameState)
    return gameState, action

  def slipOver(self, action):
    succ = self.slipProb * 100
    choice = random.randint(0,100)
    if choice <= succ:
        return action
    elif choice <= (100+succ)/2:
        if action == Directions.EAST:
            return Directions.SOUTH
        elif action == Directions.WEST:
            return Directions.NORTH
        elif action == Directions.NORTH:
            return Directions.EAST
        elif action == Directions.SOUTH:
            return Directions.WEST
    else:
        if action == Directions.EAST:
            return Directions.NORTH
        elif action == Directions.WEST:
            return Directions.SOUTH
        elif action == Directions.NORTH:
            return Directions.WEST
        elif action == Directions.SOUTH:
            return Directions.EAST
    return action

  def allSlipOver(self, action):
    actions = [Directions.EAST, Directions.WEST, Directions.SOUTH, Directions.NORTH]
    if action == Directions.EAST:
      actions.remove(Directions.WEST)
      return actions
    elif action == Directions.WEST:
      actions.remove(Directions.EAST)
      return actions
    elif action == Directions.NORTH:
      actions.remove(Directions.SOUTH)
      return actions
    elif action == Directions.SOUTH:
      actions.remove(Directions.NORTH)
      return actions