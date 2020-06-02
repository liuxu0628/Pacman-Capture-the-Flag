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
import util
from game import Directions
from game import Actions
from util import nearestPoint
import math
import random

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
    first='Attacker', second='RetardAgent'):
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
        self.boarders = self.boarderPosition(gameState)
        self.opponentBoarders = self.opponentBoarderPosition(gameState)
        self.midWidth = int(gameState.data.layout.width / 2)
        self.midHeight = int(gameState.data.layout.height / 2)
        self.height = int(gameState.data.layout.height)
        self.Width = int(gameState.data.layout.width)
        self.start = gameState.getAgentPosition(self.index)
        self.walls = gameState.getWalls().asList()
        # self.foods = self.getFood(gameState).asList()
        # print(self.foods)
        self.scanmap = ScanMap(gameState, self)
        self.deadWidth = self.scanmap.deadRoadsWithDepth()
        self.isRed = gameState.isOnRedTeam(self.index)
        self.dangerousFood = []
        self.safeFood = []
        CaptureAgent.registerInitialState(self, gameState)

        '''
        Your initialization code goes here, if you need any.
        '''

    def getMiddleGoal(self, gameState, posit):
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if gameState.isOnRedTeam(self.index):
            midPoint = (self.midWidth - 1, self.midHeight)
        else:
            midPoint = (self.midWidth, self.midHeight)

        if midPoint in self.walls:
            adjacent = self.scanmap.adjacentValidPoints(self.midWidth, self.midHeight)
            goal = random.choice(adjacent)
            dis = self.getMazeDistance(myPos, goal)
            return random.choice(adjacent), dis
        else:
            dis = self.getMazeDistance(myPos, midPoint)
            return midPoint, dis

    def getFoodGoal(self, gameState):
        """
        return the distance between current agent and the nearest food
        :param gameState:
        :return:the position of nearest food
        """
        food = self.getFood(gameState).asList()
        # print(food)
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if len(food) > 0:
            dis = 9999
            nearestFood = food[0]
            for a in food:
                temp = self.getMazeDistance(myPos, a)
                if temp < dis:
                    dis = temp
                    nearestFood = a
            return nearestFood
        else:
            return None

    def getDangerousFoodGoal(self, gameState):
        """
        return the distance between current agent and the nearest food
        :param gameState:
        :return:the position of nearest food
        """
        food = self.dangerousFood
        # print(food)
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if len(food) > 0:
            dis = 9999
            nearestFood = food[0]
            for a in food:
                temp = self.getMazeDistance(myPos, a)
                if temp < dis:
                    dis = temp
                    nearestFood = a
            return nearestFood
        else:
            return None

    def getSafeFoodGoal(self, gameState):
        """
        return the distance between current agent and the nearest food
        :param gameState:
        :return:the position of nearest food
        """
        food = self.safeFood
        # print(food)
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if len(food) > 0:
            dis = 9999
            nearestFood = food[0]
            for a in food:
                temp = self.getMazeDistance(myPos, a)
                if temp < dis:
                    dis = temp
                    nearestFood = a
            return nearestFood
        else:
            return None

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

    def disToBoarder(self, gameState):
        boarder = self.getBoarderGoal(gameState)
        myPos = self.getCurrentObservation().getAgentPosition(self.index)
        return self.getMazeDistance(myPos, boarder)

    def getPacmanGoal(self, gameState):
        """
        This method measure the distance between current agent and the nearest Pac man
        :param gameState:
        :return:the position of nearest pacman
        """
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        pac_man = [a for a in enemies if a.isPacman and a.getPosition() != None]
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if len(pac_man) > 0:
            dis = 9999
            nearestPacman = pac_man[0]
            for p in pac_man:
                temp = self.getMazeDistance(myPos, p.getPosition())
                if temp < dis:
                    dis = temp
                    nearestPacman = p
            return nearestPacman.getPosition(), dis
        else:
            return None, None

    def getCapsuleGoal(self, gameState):
        """
       This method measure the distance between current agent and the nearest Ghost
       :param gameState:
       :return:
        """
        capsules = self.getCapsules(gameState)
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if len(capsules) > 0:
            dis = 9999
            nearestCapsule = capsules[0]
            for c in capsules:
                temp = self.getMazeDistance(myPos, c)
                if temp < dis:
                    dis = temp
                    nearestCapsule = c
            return nearestCapsule
        else:
            return None

    def numOfFood(self, gameState):
        food = self.getFood(gameState).asList()
        return len(food)

    def disToCasule(self, gameState):
        capsule = self.getCapsuleGoal(gameState)
        if capsule == None:
            return None
        myPos = self.getCurrentObservation().getAgentPosition(self.index)
        return self.getMazeDistance(myPos, capsule)

    def disToGhost(self, gameState):
        goal = self.getGhostGoal(gameState)
        if goal == None:
            return None
        myPos = self.getCurrentObservation().getAgentPosition(self.index)
        return self.getMazeDistance(myPos, goal)

    def getGhostGoal(self, gameState):
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        ghost = [a for a in enemies if not a.isPacman and a.getPosition() != None]
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if len(ghost) > 0:
            dis = 9999
            nearestPacman = ghost[0]
            for p in ghost:
                temp = self.getMazeDistance(myPos, p.getPosition())
                if temp < dis:
                    dis = temp
                    nearestPacman = p
            return nearestPacman.getPosition()
        else:
            return None

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

    def disToDangerousFood(self,gameState):
        myPos = self.getCurrentObservation().getAgentPosition(self.index)
        foodPos = self.getDangerousFoodGoal(gameState)
        if foodPos == None:
            return None
        else:
            return self.getMazeDistance(myPos, foodPos)

    def disToSafeFood(self, gameState):
        myPos = self.getCurrentObservation().getAgentPosition(self.index)
        foodPos = self.getSafeFoodGoal(gameState)
        if foodPos == None:
            return None
        else:
            return self.getMazeDistance(myPos, foodPos)

    def boarderPosition(self, gameState):
        """
        return a list of positions of boundary
        """
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

    def opponentBoarderPosition(self, gameState):
        if self.red:
          i = self.midWidth + 1
        else:
          i = self.midWidth - 1
        boudaries = [(i,j) for j in  range(self.height)]
        validPositions = []
        for i in boudaries:
          if not gameState.hasWall(i[0],i[1]):
            validPositions.append(i)
        return validPositions

    def nullHeuristic(self, gameState, currentPos=None):
        return 0

    def simpleHeuristic(self, gameState, currentPos):
        goal = self.getBoarderGoal(gameState)
        return self.getMazeDistance(currentPos, goal)

    def middleHeuristic(self, gameState, currentPos):
        goal = self.getMiddleGoal(gameState)
        return self.getMazeDistance(currentPos, goal)

    def aStarSearch(self, gameState, goal, heuristic=nullHeuristic):
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

    def GeneralHeuristic(self, gameState, state):

        """
        This heuristic is used for to avoid ghoost, we give the
        position which close to ghost a higher heuristic to avoid
        colission with ghost

        """
        heuristic = 0
        if self.getGhosts(gameState) != None:
            enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
            # pacmans = [a for a in enemies if a.isPacman]
            ghosts = [a for a in enemies if not a.isPacman and a.scaredTimer < 2 and a.getPosition() is not None]
            if ghosts != None and len(ghosts) > 0:
                ghostpositions = [ghost.getPosition() for ghost in ghosts]
                # pacmanPositions = [pacman.getPosition() for pacman in pacmans]
                ghostDists = [self.getMazeDistance(state, ghostposition) for ghostposition in ghostpositions]
                ghostDist = min(ghostDists)
                if ghostDist < 2:
                    # print ghostDist
                    heuristic = pow((5 - ghostDist), 5)
        return heuristic

    def getGhosts(self, gameState):
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        ghosts = [a for a in enemies if not a.isPacman and a.getPosition() != None]
        if len(ghosts) == 0:
            return None
        else:
            return ghosts

    def isZero(self, num):
        if num == 0 or num is None:
            return 1
        else:
            return num

    def normalize(self, dict):
        Max = max(v for k, v in dict.items() if v != 0)
        Min = min(v for k, v in dict.items() if v != 0)
        for k,v in dict.items():
            v = (v - Min) / (Max - Min)
            temp = {k: v}
            dict.update(temp)
        return dict

    def DefenderHeuristic(self, gameState, state):
        goal = self.getPacmanGoal(gameState)
        return self.getMazeDistance(state, goal)

    def opponentscaredTime(self, gameState):
        opponents = self.getOpponents(gameState)
        for opponent in opponents:
            if gameState.getAgentState(opponent).scaredTimer > 1:
                return gameState.getAgentState(opponent).scaredTimer
        return None

class ScanMap:

    def __init__(self, gameState, agent):
        # Store the food for later reference
        self.food = agent.getFood(gameState).asList()
        # Store info for the PositionSearchProblem (no need to change this)
        self.isRed = gameState.isOnRedTeam(agent.index)
        self.walls = gameState.getWalls().asList()
        self.boarder = agent.boarderPosition(gameState)
        self.height = gameState.data.layout.height
        self.width = gameState.data.layout.width
        self.midWidth = int(gameState.data.layout.width / 2)


    def adjacentValidPoints(self, x, y):
        points = []
        point = (x + 1, y)
        if point not in self.walls:
            points.append(point)
        point = (x, y - 1)
        if point not in self.walls:
            points.append(point)
        point = (x - 1, y)
        if point not in self.walls:
            points.append(point)
        point = (x, y + 1)
        if point not in self.walls:
            points.append(point)
        return points
    def oneEntryPoints(self):
        points = []
        if self.isRed:
            for x in range(self.midWidth, self.width):
                for y in range(0, self.height):
                    if (x, y) not in self.walls:
                        adjacent = self.adjacentValidPoints(x, y)
                        if len(adjacent) == 1:
                            points.append((x, y))
        else:
            for x in range(0, self.midWidth-1):
                for y in range(0, self.height):
                    if(x, y) not in self.walls:
                        adjacent = self.adjacentValidPoints(x, y)
                        if len(adjacent) == 1:
                            points.append((x, y))
        return points

    def deadRoads(self):
        dangerousPoints = self.oneEntryPoints()
        deadRoads = dangerousPoints.copy()
        deadEntry = []
        for (x, y) in dangerousPoints:
            adjacent = self.adjacentValidPoints(x, y)
            if adjacent in deadRoads:
                continue
            else:
                while len(adjacent) == 1:
                    (a, b) = adjacent[0]
                    adjacent = self.adjacentValidPoints(a, b)
                    adjacent = [i for i in adjacent if i not in deadRoads]
                    if len(adjacent) > 1:
                        deadEntry.append((a, b))
                    else:
                        deadRoads.append((a, b))
        return self.duplicates(deadEntry), self.duplicates(deadRoads)

    def deadRoadsWithDepth(self):
        from util import Queue
        deadEntry, deadRoads = self.deadRoads()
        withDepth = {}
        done = []
        for i in deadEntry:
            explored = Queue()
            depth = 0
            withDepth[i] = depth
            explored.push(i)
            done.append(i)
            while not explored.isEmpty():
                j = explored.pop()
                (x, y) = j
                adjacentPoints = self.adjacentValidPoints(x, y)
                adjacentPoints = [j for j in adjacentPoints if j in deadRoads and j not in done and j not in deadEntry]
                if len(adjacentPoints) != 0:
                    depth += 1
                for k in adjacentPoints:
                    explored.push(k)
                    done.append(k)
                    withDepth[k] = depth
        return withDepth


    def duplicates(self, x):
        return list(dict.fromkeys(x))


class Attacker(IntelligentAgent):

    def chooseAction(self, gameState):
        x, y = self.getCurrentObservation().getAgentPosition(self.index)
        if not self.isRed:
            x = self.Width - x
        if x < self.midWidth - 1:
            boarder_goal = self.getBoarderGoal(gameState)
            action = self.aStarSearch(gameState, boarder_goal, self.simpleHeuristic)
            return action
        if x == self.midWidth - 1:
            food_goal = self.getFoodGoal(gameState)
            action = self.aStarSearch(gameState, food_goal, self.GeneralHeuristic)
            return action
        else:
            self.dangerousFood = [i for i in self.deadWidth.keys() if i in self.getFood(gameState).asList()]
            self.safeFood = [i for i in self.getFood(gameState).asList() if i not in self.dangerousFood]
            # this list stores all the actions can be taken according to different goals
            actions = {}

            # this list stores the scores of each goal, which indicates the tendency of choose that goal.
            scores = {}

            # find actions to return border
            boarder_goal = self.getBoarderGoal(gameState)
            action = self.aStarSearch(gameState, boarder_goal, self.GeneralHeuristic)
            actions["boarder"] = action


            # actions to eat capsules
            capsule_goal = self.getCapsuleGoal(gameState)
            action = self.aStarSearch(gameState, capsule_goal, self.GeneralHeuristic)
            actions["capsule"] = action

            # actions to eat safe food
            safe_food_goal = self.getSafeFoodGoal(gameState)
            action = self.aStarSearch(gameState, safe_food_goal, self.GeneralHeuristic)
            actions["safeFood"] = action

            # actions to eat dangerous food
            dangerous_food_goal = self.getDangerousFoodGoal(gameState)
            action = self.aStarSearch(gameState, dangerous_food_goal, self.GeneralHeuristic)
            actions["dangerousFood"] = action

            # calculate the scores for each action

            # starts with foodScore
            ghostDis = self.disToGhost(gameState)
            foodNum = self.numOfFood(gameState)
            safeDis = self.disToSafeFood(gameState)
            safeDis = self.isZero(safeDis)
            dangerousDis = self.disToDangerousFood(gameState)
            dangerousDis = self.isZero(dangerousDis)
            carried = self.getCurrentObservation().getAgentState(self.index).numCarrying
            carried = self.isZero(carried)
            if foodNum < 3:
                scores["safeFood"] = -9999
                scores["dangerousFood"] = -9999
            else:
                if safe_food_goal != None:
                    scores["safeFood"] = 50 / (carried * safeDis)
                else:
                    scores["safeFood"] = -9999
            if dangerousDis is not None:
                if ghostDis is not None and self.opponentscaredTime(gameState) is None:
                    if dangerousDis + self.deadWidth[dangerous_food_goal] * 2 + 1 < ghostDis:
                        scores["dangerousFood"] = 30 / (carried * dangerousDis)
                    else:
                        scores["dangerousFood"] = 8 / (carried * dangerousDis)
                else:
                    scores["dangerousFood"] = 20 / (carried * dangerousDis)
            # calculate score for back to boarder
            boarderDis = self.disToBoarder(gameState)
            boarderDis = self.isZero(boarderDis)
            scores["boarder"] = carried / math.sqrt(boarderDis)

            # calculate score for eating capsule
            capsuleDis = self.disToCasule(gameState)
            if capsuleDis == None:
                scores["capsule"] = -9999
            else:
                capsuleDis = self.isZero(capsuleDis)
                if safe_food_goal is None:
                    scores["capsule"] = 100 / capsuleDis
                else:
                    scores["capsule"] = 5 / capsuleDis


            scores = self.normalize(scores)
            max = -99999
            key = 0
            for k, v in scores.items():
                if v != 0:
                    if v > max:
                        max = v
                        key = k

            # key = max(k for k, v in scores.items() if v != 0)
            return actions[key]


class Defender(IntelligentAgent):

    def manhattanDis(self, pos1, pos2):
        (x1, y1) = pos1
        (x2, y2) = pos2
        return abs(x1-x2) + abs(y1-y2)

    def SimpleHeuristic(self, gameState, myPos, goal):
        return self.getMazeDistance(myPos, goal)

    def getSuccessors(self, currentPosition):
        successors = []
        walls = self.walls
        boarders = self.opponentBoarders
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
          x, y = currentPosition
          dx, dy = Actions.directionToVector(action)
          nextx, nexty = int(x + dx), int(y + dy)
          if (nextx, nexty) not in walls and (nextx, nexty) not in boarders:
            nextPosition = (nextx, nexty)
            successors.append((action, nextPosition))
        return successors

    def aStarSearch(self, gameState, goal, heuristic=SimpleHeuristic):
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
                    nextHeuristic = cost + heuristic(gameState, state, goal)
                    if successor not in closedList:
                        openList.push((nextPos, trace + [action]), nextHeuristic)

        return 'Stop'

    def defenderGoal(self, PacmanPos, myPos):
        (x, y) = PacmanPos
        temp = self.scanmap.adjacentValidPoints(x, y)
        targets = []
        for i in temp:
            (x, y) = i
            targets += self.scanmap.adjacentValidPoints(x, y)
        targets = list(dict.fromkeys(targets))
        targets.remove(PacmanPos)

        minDis = self.getMazeDistance(myPos, targets[0])
        nearestDefender = targets[0]
        for j in targets:
            dis = self.getMazeDistance(myPos, j)
            if dis < minDis:
                minDis = dis
                nearestDefender = j
        return nearestDefender

    def defendBoarderGoal(self, gameState, myPos):
        goal, dis = self.getGhostGoal(gameState)
        if goal is not None:
            if self.manhattanDis(myPos, goal) <= 5:
                minDis = 999
                nearestBoarder = self.boarders[0]
                for i in self.boarders:
                    if self.getMazeDistance(i, goal) < minDis:
                        minDis = self.getMazeDistance(i, goal)
                        nearestBoarder = goal
        return nearestBoarder

    def chooseAction(self, gameState):

        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        invader_goal = self.getPacmanGoal(gameState)
        if invader_goal is None:
            middle_boarder_goal = self.getMiddleGoal(gameState)
            action = self.aStarSearch(gameState, middle_boarder_goal, self.middleHeuristic)
        else:
            if gameState.getAgentState(self.index).scaredTimer > 1:
                defendGoal = self.defenderGoal(invader_goal, myPos)
                action = self.aStarSearch(gameState, defendGoal, self.DefenderHeuristic)
            else:
                action = self.aStarSearch(gameState, invader_goal, self.DefenderHeuristic)
        return action


class RetardAgent(CaptureAgent):
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
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''



  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    return Directions.STOP