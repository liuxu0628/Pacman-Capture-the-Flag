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

import queue
from captureAgents import CaptureAgent
import util
from game import Directions
from game import Actions
from util import nearestPoint
import math
import random
import datetime

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
    first='Attacker', second='Defender'):
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
        self.timer = 0
        self.missingPacmen = []
        self.foodGone = None
        self.foodToDefence = ()
        self.toBorder = True
        self.defence = False
        self.start = gameState.getAgentPosition(self.index)

        self.food = self.getFood(gameState).asList()
        self.midWidth = int(gameState.data.layout.width / 2)
        self.midHeight = int(gameState.data.layout.height / 2)
        self.height = int(gameState.data.layout.height)
        self.Width = int(gameState.data.layout.width)
        self.start = gameState.getAgentPosition(self.index)
        self.walls = gameState.getWalls().asList()
        self.blocked = False
        self.scanmap = ScanMap(gameState, self)
        self.isRed = gameState.isOnRedTeam(self.index)
        self.dangerousFood = []
        self.safeFood = []
        self.boarders = self.boarderPosition(gameState)
        self.enemyBoundaries = self.enemyBorder(gameState)
        self.previousActions = queue.Queue(maxsize=6)
        CaptureAgent.registerInitialState(self, gameState)
        self.width = gameState.data.layout.width
        self.deadPoints = self.searchDeadEnd()
        self.entryEnd = self.entryPair()
        self.endEntry = {}
        for i in self.entryEnd.keys():
            for j in self.entryEnd[i]:
                self.endEntry[j] = i
        self.deadWidth = self.deadRoadsWithDepth(self.endEntry)

        self.ourdeadPoints = []
        for (x,y) in self.deadPoints:
            self.ourdeadPoints.append((self.width-x-1, self.height-y-1))

        self.ourendEntry = {}
        for (x,y) in self.endEntry.keys():
            (i,j) = self.endEntry[(x,y)]
            self.ourendEntry[(self.width-x-1, self.height-y-1)]=((self.width-i-1, self.height-j-1))


        # self.isRed = not self.isRed
        # self.scanmap.isRed = not self.scanmap.isRed
        # self.ourdeadPoints = self.searchDeadEnd()
        # self.ourentryEnd = self.entryPair()
        # self.ourendEntry = {}
        # for i in self.ourentryEnd.keys():
        #     for j in self.ourentryEnd[i]:
        #         self.ourendEntry[j] = i
        # self.scanmap.isRed = not self.scanmap.isRed
        # self.isRed = not self.isRed
        '''
        Your initialization code goes here, if you need any.
        '''
    def enemyBorder(self, gameState):
        if self.red:
            i = self.midWidth - 1
        else:
            i = self.midWidth
        boudaries = [(i, j) for j in range(self.height)]
        enemyBoundaries = []
        for i in boudaries:
            if not (i[0], i[1]) in self.walls:
                enemyBoundaries.append(i)
        return enemyBoundaries
    def getDangerousFoodGoal(self, gameState):
        """
        return the distance between current agent and the nearest food
        :param gameState:
        :return:the position of nearest food, and its distance to myPosition
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
            return nearestFood, dis
        else:
            return None, None

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
            return nearestFood, dis
        else:
            return None, None

    def getFoodGoal(self, gameState):
        food = self.getFood(gameState).asList()
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if len(food) > 0:
            dis = 9999
            nearestFood = food[0]
            for a in food:
                temp = self.getMazeDistance(myPos, a)
                if temp < dis:
                    dis = temp
                    nearestFood = a
            return nearestFood, dis
        else:
            return None, None

    def getBoarderGoal(self, gameState):
        boards = self.boarders
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        if len(boards) > 0:
            dis = 9999
            nearestBoarder = boards[0]
            for b in boards:
                temp = self.getMazeDistance(myPos, b)
                if temp < dis:
                    dis = temp
                    nearestBoarder = b
            return nearestBoarder, dis
        else:
            return None, None

    def entryPair(self):

        exitPair = {}
        deadEnd = self.searchDeadEnd()
        deadPoints = []
        deadRoads = deadEnd.copy()
        for (x,y) in deadEnd:
             # if (x,y) not in deadPoints:
                adjacent = self.scanmap.adjacentValidPoints(x, y)
                BFS = util.Queue()
                visited = []
                BFS.push((x,y))
                while not BFS.isEmpty():
                    (i,j) = BFS.pop()
                    visited.append((i, j))
                    adjacents = self.scanmap.adjacentValidPoints(i,j)
                    for position in adjacents:
                        if not position in deadRoads:
                            exitPoint = position
                        if position in deadRoads and not position in visited:
                            BFS.push(position)
                if exitPoint in exitPair.keys():
                    exitPair[exitPoint] = list(set(visited).union(set(exitPair[exitPoint])))
                else:
                    exitPair[exitPoint] = visited
                deadPoints = list(set(deadEnd).difference(set(visited)))

        return exitPair

    def searchDeadEnd(self):
        boundaries = []
        if not self.red:
            i = self.midWidth - 1
        else:
            i = self.midWidth + 1
        boudaries = [(i, j) for j in range(self.height)]
        validPositions = []
        for i in boudaries:
            if not (i[0], i[1]) in self.walls:
                validPositions.append(i)

        dangerPos = []

        toExpand = self.scanmap.twoEntryPoints()
        for (x,y) in toExpand:
            adjacent = self.scanmap.adjacentValidPoints(x, y)
            if not (x,y) in dangerPos:
                for (u, w) in adjacent:
                    visited = []
                    visited.append((x, y))
                    safe = False
                    danger = False
                    DFS = util.Stack()
                    DFS.push((u,w))
                    while not safe and not danger:
                        (i,j) = DFS.pop()
                        visited.append((i,j))
                        adjacents = self.scanmap.adjacentValidPoints(i,j)
                        for position in adjacents:
                            if not position in visited:
                                DFS.push(position)
                        if DFS.isEmpty():
                            danger = True
                            dangerPos = list(set(dangerPos) | set(visited))

                        if (i,j) in validPositions:
                            safe = True
        oneEntry = self.scanmap.oneEntryPoints()
        dangerPos = list(set(oneEntry).union(set(dangerPos)))
        dangerPos.sort()
        return dangerPos

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
            return nearestCapsule, dis
        else:
            return None, None

    def numOfFood(self, gameState):
        food = self.getFood(gameState).asList()
        return len(food)

    def getAnotherBoarderGoal(self, gameState, current_goal):
        myPos = self.getCurrentObservation().getAgentState(self.index).getPosition()
        goal = current_goal
        max_dis = 0
        for i in self.boarders:
            temp = self.getMazeDistance(i, current_goal)
            if temp > max_dis:
                max_dis = temp
                goal = i
        max_dis = self.getMazeDistance(myPos, goal)
        return goal, max_dis

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
            return nearestPacman.getPosition(), dis
        else:
            return None, None

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

    def boarderPosition(self, gameState):
        """
        return a list of positions of boundary
        """
        if gameState.isOnRedTeam(self.index):
          i = self.midWidth - 1
        else:
          i = self.midWidth + 1
        boudaries = [(i,j) for j in range(self.height)]
        validPositions = []
        for i in boudaries:
          if not gameState.hasWall(i[0],i[1]):
            validPositions.append(i)
        return validPositions

    def nullHeuristic(self, gameState, currentPos=None):
        return 0

    def simpleHeuristic(self, gameState, currentPos):
        goal, disToBoarder = self.getBoarderGoal(gameState)
        return disToBoarder

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
        ghost, disToGhost = self.getGhostGoal(gameState)
        if ghost is not None:
            enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
            ghosts = [a for a in enemies if not a.isPacman and a.scaredTimer < 2 and a.getPosition() is not None]
            if ghosts != None and len(ghosts) > 0:
                ghostpositions = [ghost.getPosition() for ghost in ghosts]
                ghostDists = [self.getMazeDistance(state, ghostposition) for ghostposition in ghostpositions]
                ghostDist = min(ghostDists)
                if ghostDist < 2:
                    heuristic = pow((5 - ghostDist), 5)
        return heuristic

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

    def opponentscaredTime(self, gameState):
        opponents = self.getOpponents(gameState)
        for opponent in opponents:
            if gameState.getAgentState(opponent).scaredTimer > 1:
                return gameState.getAgentState(opponent).scaredTimer
        return None

    def deadRoadsWithDepth(self, endEntry):
        withDepth = {}
        for point, entry in endEntry.items():
            withDepth[point] = self.getMazeDistance(point, entry)
        return withDepth

    def randomMove(self, preAction, gameState):
        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        if not preAction == None and len(actions) > 1 and Directions.REVERSE[preAction] in actions:
            preAction = Directions.REVERSE[preAction]
            actions.remove(preAction)

        action = random.choice(actions)
        gameState = gameState.generateSuccessor(self.index, action)
        # print (gameState)
        return gameState, action

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
        self.agent = agent

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

    def duplicates(self, x):
        return list(dict.fromkeys(x))

    def twoEntryPoints(self):
        points = []
        if self.isRed:
            for x in range(self.midWidth, self.width):
                for y in range(0, self.height):
                    if (x, y) not in self.walls:
                        adjacent = self.adjacentValidPoints(x, y)
                        if len(adjacent) == 2:
                            points.append((x, y))
        else:
            for x in range(0, self.midWidth-1):
                for y in range(0, self.height):
                    if(x, y) not in self.walls:
                        adjacent = self.adjacentValidPoints(x, y)
                        if len(adjacent) == 2:
                            points.append((x, y))
        return points

class Attacker(IntelligentAgent):

    def chooseAction(self, gameState):
        if self.getPreviousObservation() is not None:
            lastCapsule = self.getCapsules(self.getPreviousObservation())
        else:
            lastCapsule = None
        x, y = self.getCurrentObservation().getAgentPosition(self.index)
        if (x, y) == (24, 5):
            print(1)
        self.dangerousFood = [i for i in self.deadWidth.keys() if i in self.getFood(gameState).asList()]
        self.safeFood = [i for i in self.getFood(gameState).asList() if i not in self.dangerousFood]
        # this list stores all the actions can be taken according to different goals
        actions = {}

        # this list stores the scores of each goal, which indicates the tendency of choose that goal.
        scores = {}
        # find actions to go to boarder
        boarder_goal, disToBoarder = self.getBoarderGoal(gameState)
        action = self.aStarSearch(gameState, boarder_goal, self.simpleHeuristic)
        actions["go_to_boarder"] = action

        # # find the actions to another boarder
        # another_boarder_goal, disToBoarder = self.getAnotherBoarderGoal(gameState, boarder_goal)
        # action = self.aStarSearch(gameState, another_boarder_goal, self.nullHeuristic)
        # actions["another_boarder"] = action

        # find actions to return border
        boarder_goal, disToBoarder = self.getBoarderGoal(gameState)
        action = self.aStarSearch(gameState, boarder_goal, self.GeneralHeuristic)
        actions["return_boarder"] = action

        # actions to eat capsules
        capsule_goal, disToCapsule = self.getCapsuleGoal(gameState)
        currentCapsule = self.getCapsules(gameState)
        if currentCapsule is None and lastCapsule is not None:
            self.timer = 20
        elif lastCapsule is not None and currentCapsule is not None:
            if len(lastCapsule) - len(currentCapsule) == 1:
                self.timer = 20
        action = self.aStarSearch(gameState, capsule_goal, self.GeneralHeuristic)
        actions["capsule"] = action

        # actions to eat safe food
        safe_food_goal, disToSafeFood = self.getSafeFoodGoal(gameState)
        # if (x, y) == (22, 8):
        #     print(1)
        capsule_goal, disToCapsule = self.getCapsuleGoal(gameState)
        self.walls.append(capsule_goal)
        action = self.aStarSearch(gameState, safe_food_goal, self.GeneralHeuristic)
        actions["safeFood"] = action

        # actions to eat dangerous food
        dangerous_food_goal, disToDangerousFood = self.getDangerousFoodGoal(gameState)
        action = self.aStarSearch(gameState, dangerous_food_goal, self.GeneralHeuristic)
        self.walls.remove(capsule_goal)
        actions["dangerousFood"] = action

        # calculate the scores for each action

        ghost_goal, ghostDis = self.getGhostGoal(gameState)
        foodNum = self.numOfFood(gameState)
        safeDis = disToSafeFood
        safeDis = self.isZero(safeDis)
        dangerousDis = disToDangerousFood
        dangerousDis = self.isZero(dangerousDis)
        carried = self.getCurrentObservation().getAgentState(self.index).numCarrying
        disToBoarder = self.isZero(disToBoarder)

        # choose actions
        if not self.isRed:
            x = self.Width - x

        if x < self.midWidth - 1:
            if self.blocked:
                action = actions["another_boarder"]
                if self.previousActions.full():
                    self.previousActions.get()
                self.previousActions.put(action)
                return action
            else:
                if foodNum < 3:
                    return actions["return_boarder"]
                else:
                    if safe_food_goal is not None:
                        scores["return_boarder"] = carried / math.sqrt(disToBoarder)
                        scores["safeFood"] = 30 / ((carried + 1) * safeDis)
                        if disToCapsule == None:
                            scores["capsule"] = 0
                        else:
                            capsuleDis = self.isZero(disToCapsule)
                            if self.timer > 0:
                                self.timer -= 1
                                scores["capsule"] = 0
                            else:
                                scores["capsule"] = 50 / capsuleDis
                        max = -99999
                        key = 0
                        for k, v in scores.items():
                            if v != 0:
                                if v > max:
                                    max = v
                                    key = k
                        return actions[key]
                    else:
                        scores["return_boarder"] = carried / math.sqrt(disToBoarder)

                        if disToCapsule == None:
                            scores["capsule"] = 0
                        else:
                            capsuleDis = self.isZero(disToCapsule)
                            if self.timer > 0:
                                self.timer -= 1
                                scores["capsule"] = 0

                            else:
                                scores["capsule"] = 500 / capsuleDis
                        if dangerousDis is not None:
                            if ghostDis is not None:
                                scaredTimer = self.opponentscaredTime(gameState)
                                if scaredTimer is None:
                                    if ghostDis > 5:
                                        ghostDis = 5
                                    if dangerousDis + self.deadWidth[dangerous_food_goal] + 1 < ghostDis:
                                        scores["dangerousFood"] = 30 / ((carried + 1) * dangerousDis)
                                    else:
                                        scores["dangerousFood"] = 3 / ((carried + 1) * dangerousDis)
                                else:
                                    if dangerousDis + self.deadWidth[dangerous_food_goal] + 1 < scaredTimer:
                                        scores["dangerousFood"] = 30 / ((carried + 1) * dangerousDis)
                                    else:
                                        scores["dangerousFood"] = 8 / ((carried + 1) * dangerousDis)
                            else:
                                scores["dangerousFood"] = 20 / ((carried + 1) * dangerousDis)
                        max = -99999
                        key = 0
                        for k, v in scores.items():
                            if v != 0:
                                if v > max:
                                    max = v
                                    key = k
                        return actions[key]
        else:
            if foodNum < 3:
                return actions["return_boarder"]
            else:
                if safe_food_goal is not None:
                    scores["return_boarder"] = carried / math.sqrt(disToBoarder)
                    scores["safeFood"] = 30 / ((carried + 1) * safeDis)
                    if disToCapsule == None:
                        scores["capsule"] = 0
                    else:
                        capsuleDis = self.isZero(disToCapsule)
                        if self.timer > 0:
                            self.timer -= 1
                            scores["capsule"] = 0
                            scores["return_boarder"] = carried / disToBoarder
                        else:
                            scores["capsule"] = 50 / capsuleDis
                    max = -99999
                    key = 0
                    for k, v in scores.items():
                        if v != 0:
                            if v > max:
                                max = v
                                key = k
                    return actions[key]
                else:
                    scores["return_boarder"] = carried / math.sqrt(disToBoarder)
                    if disToCapsule == None:
                        scores["capsule"] = 0
                    else:
                        capsuleDis = self.isZero(disToCapsule)
                        if self.timer > 0:
                            self.timer -= 1
                            scores["capsule"] = 0
                            scores["return_boarder"] = carried / disToBoarder
                        else:
                            scores["capsule"] = 500 / capsuleDis
                    if dangerousDis is not None:
                        if ghostDis is not None:
                            scaredTimer = self.opponentscaredTime(gameState)
                            if scaredTimer is None:
                                if ghostDis > 5:
                                    ghostDis = 5
                                if dangerousDis + self.deadWidth[dangerous_food_goal] + 1 < ghostDis:
                                    scores["dangerousFood"] = 30 / ((carried + 1) * dangerousDis)
                                else:
                                    scores["dangerousFood"] = 3 / ((carried + 1) * dangerousDis)
                            else:
                                if dangerousDis + self.deadWidth[dangerous_food_goal] + 1 < scaredTimer:
                                    scores["dangerousFood"] = 30 / ((carried + 1) * dangerousDis)
                                else:
                                    scores["dangerousFood"] = 8 / ((carried + 1) * dangerousDis)
                        else:
                            scores["dangerousFood"] = 20 / ((carried + 1) * dangerousDis)
                    max = -99999
                    key = 0
                    for k, v in scores.items():
                        if v != 0:
                            if v > max:
                                max = v
                                key = k
                    return actions[key]


class Defender(IntelligentAgent):

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
        boudaries = [(i, j) for j in range(self.height)]
        validPositions = []
        for i in boudaries:
            if not gameState.hasWall(i[0], i[1]):
                validPositions.append(i)
        return validPositions

    def chooseAction(self, gameState):
        startTime = datetime.datetime.now()
        x, y = self.getCurrentObservation().getAgentPosition(self.index)
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        ghosts = [a for a in enemies if not a.isPacman and a.getPosition() != None]
        pacmen = [a for a in enemies if a.isPacman and a.getPosition() != None]
        if self.start == (x, y):
            self.toBorder = True
        if len(pacmen) > 0:
            self.toBorder = False
        if self.toBorder:
            if x < self.midWidth - 1 and self.red:
                boarder_goal, boarder_dis = self.getBoarderGoal(gameState)
                action = self.aStarSearch(gameState, boarder_goal, self.simpleHeuristic)
                # print(datetime.datetime.now() - startTime)
                return action
            elif x > self.midWidth +1 and not self.red:
                boarder_goal, boarder_dis = self.getBoarderGoal(gameState)
                action = self.aStarSearch(gameState, boarder_goal, self.simpleHeuristic)
                # print(datetime.datetime.now() - startTime)
                return action
            else:
                self.toBorder = False
        Cp = 0.5
        gamma = 0.9
        limit = 25
        time = 0
        actions = gameState.getLegalActions(self.index)
        # actions.remove(Directions.STOP)
        UCT = {}
        position = gameState.getAgentPosition(self.index)
        lastGameState = self.getPreviousObservation()

        lastEnemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        lastPacmen = [a for a in lastEnemies if a.isPacman and a.getPosition() != None]
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        pacmen = [a for a in enemies if a.isPacman and a.getPosition() != None]
        ghosts = [a for a in enemies if not a.isPacman and a.getPosition() != None]
        if len(pacmen) < len(lastPacmen):
            for p in lastPacmen:
                if (self.getMazeDistance(position, p)) > 2:
                    self.missingPacmen = list(set(lastPacmen) - set(pacmen))
                else:
                    self.defence = False
        elif len(pacmen) > len(lastPacmen):
            self.missingPacmen = []

        currentFood = self.getFoodYouAreDefending(gameState).asList()
        if lastGameState != None:
            lastFood = self.getFoodYouAreDefending(lastGameState).asList()
            if len(currentFood) == len(self.food) or len(currentFood) > len(lastFood):
                self.defence = False
                self.foodGone = None
                self.foodToDefence = None
            elif len(currentFood) < len(lastFood):
                self.defence = True
                self.foodGone = list(set(lastFood) - set(currentFood))[0]
            elif len(currentFood) > len(lastFood):
                self.foodGone = None
                self.foodToDefence = ()

        minDis = 99999
        if self.foodGone != None:
            for food in currentFood:
                Dis = self.getMazeDistance(self.foodGone, food)
                if minDis > Dis:
                    minDis = Dis
                    self.foodToDefence = food



        for action in actions:
            UCT[action] = [0, 0]
            time = 0
            sumValue = 0
            # while time < limit and (datetime.datetime.now()-startTime).total_seconds()<=0.9:
            while time < limit:
                smValue = self.simulation(gamma, gameState, action, enemies, ghosts, pacmen)
                sumValue += smValue
                time += 1
            UCT[action] = [sumValue / time, time]




        highest = -99999
        for a in UCT.keys():
            v, t = UCT[a]
            if highest <= v:
                highest = v
                bestAct = a
        # print(datetime.datetime.now()-startTime)
        return bestAct

    def simulation(self, gamma, gameState, preAction, enemies, ghosts, pacmen):
        limit = 50
        time = 0


        gameState = gameState.generateSuccessor(self.index, preAction)
        score = 0
        while time < limit:
            time += 1
            position = gameState.getAgentPosition(self.index)
            if not ghosts == None:
                for g in ghosts:
                    p = g.getPosition()
                    if self.getMazeDistance(position, p) <= 1:
                        if g.scaredTimer <= 2:
                            score = -(100) * (math.pow(gamma, time))
                            return score
                    if position == self.start:
                        score = -(100) * (math.pow(gamma, time))
                        return score


            if len(pacmen) > 0:
                for p in pacmen:
                    posit = p.getPosition()
                    if gameState.getAgentState(self.index).scaredTimer == 0:
                        if posit in self.ourdeadPoints and position == self.ourendEntry[posit]:
                            score = (10000) * (math.pow(gamma, time))
                            return score
                        if position == posit:
                            score = (100) * (math.pow(gamma, time))
                            return score
                    else:
                        if self.getMazeDistance(position, posit) == 2:
                            score = (100) * (math.pow(gamma, time))
                            return score
                        elif self.getMazeDistance(position, posit) < 2:
                            score = -(100) * (math.pow(gamma, time))
                            return score

                        if posit in self.ourdeadPoints and position == self.ourendEntry[posit]:
                            score = (10000) * (math.pow(gamma, time))
                            return score

            if len(pacmen) == 0:
                if True:
                    # gameState.getAgentState(self.index).scaredTimer == 0:
                    if len(self.missingPacmen) != 0:
                        for posit in self.missingPacmen:
                            if posit in self.ourdeadPoints and position == self.ourendEntry[self.foodGone]:
                                score = (10000) * (math.pow(gamma, time))
                                return score
                            elif position in self.missingPacmen:
                                score = (10) * (math.pow(gamma, time))
                                return score

                    if self.foodGone in self.ourdeadPoints and position == self.ourendEntry[self.foodGone]:
                        score = (10000) * (math.pow(gamma, time))
                        return score
                if self.defence and position == self.foodToDefence:
                    score += (100) * (math.pow(gamma, time))
                    return score
                if position in self.boundaryPosition(gameState) and not self.defence:
                    (x, y) = position
                    dis = abs(y - self.height / 2)
                    score += (1 / (dis+1)) * (math.pow(gamma, time))
                    return score
            if position in self.enemyBoundaries:
                    score -= (1) * (math.pow(gamma, time))
                    return score
            gameState, preAction = self.randomMove(preAction, gameState)

        return score

    def randomMove(self, preAction, gameState):
        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        if not preAction == None and len(actions) > 1 and Directions.REVERSE[preAction] in actions:
            preAction = Directions.REVERSE[preAction]
            actions.remove(preAction)

        action = random.choice(actions)
        gameState = gameState.generateSuccessor(self.index, action)
        # print (gameState)
        return gameState, action



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