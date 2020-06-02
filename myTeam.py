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
import math
import random

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
        self.opponentBoarders = self.opponentBoarderPosition(gameState)

    def opponentBoarderPosition(self, gameState):
        """
        Find the valid opponent boarder points.
        :param gameState: The current game information.
        :return: A list of opponent boarder points.
        """
        if self.red:
          i = self.midWidth
        else:
          i = self.midWidth - 1
        boudaries = [(i,j) for j in  range(self.height)]
        validPositions = []
        for i in boudaries:
          if not gameState.hasWall(i[0],i[1]):
            validPositions.append(i)
        return validPositions

    def getDangerousFoodGoal(self, gameState):
        """
        Find the nearest dangerous food.
        :param gameState: The current game information.
        :return:It there are dangerous food left, return the position of nearest dangerous food, and its distance to myPosition.
                Else, return None, None
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
        Find the nearest safe food.
        :param gameState: The current game information.
        :return:It there are safe food left, return the position of nearest safe food, and its distance to myPosition.
                Else, return None, None
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

    def getBoarderGoal(self, gameState):
        """
        Find the nearest boarder point.
        :param gameState: The current game information.
        :return: return the position of nearest boarder, and its distance to myPosition.
        """
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
        """
        Match the identified dead roads with their entries.
        :return: A dictionary of <Dead Road Point, Its Entry>.
        """
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
        """
        Identify all the dead roads using two-way DFS on all two-entry points.
        :return: A list of all points in the dead roads.
        """
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
       Find the nearest capsule.
       :param gameState: The current game information.
       :return: It there are capsules left, return the position of nearest capsule, and its distance to myPosition.
                Else, return None, None
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
        """
        :param gameState: gameState: The current game information.
        :return: The number of food left
        """
        food = self.getFood(gameState).asList()
        return len(food)

    def getGhostGoal(self, gameState):
        """
        Find the nearest ghost.
        :param gameState: The current game information.
        :return: It there are ghost in the agent's view, return the position of nearest ghost, and its distance to myPosition.
                Else, return None, None
        """
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
        """
        Find all the valid adjacent points of current postion.
        :param currentPosition: (x, y)
        :return: A list of valid adjacent points of current postion.
        """
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
        Find the valid our boarder points.
        :param gameState: The current game information.
        :return: A list of our boarder points.
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
        """
        :param gameState: The current game information.
        :param currentPos: The current position of our agent.
        :return: 0
        """
        return 0

    def simpleHeuristic(self, gameState, currentPos):
        """
        Measures the distance between current position and nearest boarder.
        :param gameState: The current game information.
        :param currentPos: The current position of our agent.
        :return: Maze distance between current position and nearest boarder.
        """
        goal, disToBoarder = self.getBoarderGoal(gameState)
        return disToBoarder

    def aStarSearch(self, gameState, goal, heuristic=nullHeuristic):
        """
        The implementation of a_star search algorithm for finding the actions to approach goals.
        :param gameState: The current game information.
        :param goal: The position of goal.
        :param heuristic: The heuristic functions.
        :return: The first action of the possible actions to approach goals, and if there are no ways to the goal, it returns None.
        """
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
        Ghost aware heuristic which measuring the distance to nearest ghost. As the distance became smaller,
        the heuristic became larger.
        :param gameState: The current game information.
        :param state: My position
        :return: Heuristic value.
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
        """
        Change the zero number to one.
        :param num: Original value
        :return: Changed value
        """
        if num == 0 or num is None:
            return 1
        else:
            return num

    def opponentscaredTime(self, gameState):
        """
        Get the scared time of the opponent agents
        :param gameState: The current game information.
        :return: The scared time left of the opponent agents if they are in the view of our agents.
                Else, return None
        """
        opponents = self.getOpponents(gameState)
        for opponent in opponents:
            if gameState.getAgentState(opponent).scaredTimer > 1:
                return gameState.getAgentState(opponent).scaredTimer
        return None

    def deadRoadsWithDepth(self, endEntry):
        """
        Mark all points in the dead roads with its depth from entry.
        :param endEntry: A dictionary of dead road points with its entry.
        :return: A dictionary of dead road points with its distance to entry.
        """
        withDepth = {}
        for point, entry in endEntry.items():
            withDepth[point] = self.getMazeDistance(point, entry)
        return withDepth


class ScanMap:
    """
    This class is responsible to scan the map before the game starts for finding the dead roads.
    """
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
        """
        Find all the valid adjacent points surround (x, y).
        :param x: X value of my position
        :param y: Y value of my position
        :return: A list of valid adjacent points.
        """
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
        """
        Find all the one entry points on the enemy side.
        :return: A list of one entry points.
        """
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
        """
        For every one entry points, use DFS to find the entry of them.
        :return: A dictionary of dead roads and their entry.
        """
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
        """
        remove the duplication of the input dictionary.
        :param x: Input dictionary.
        :return: A dictionary of input dictionary removing deuplicatons.
        """
        return list(dict.fromkeys(x))

    def twoEntryPoints(self):
        """
        Find all the two entry points on the enemy side.
        :return: A list of two entry points.
        """
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
        """
        Return the action chosen according to startegy tree and score system
        :param gameState: The current game information
        :return: The action decided.
        """
        if self.getPreviousObservation() is not None:
            lastCapsule = self.getCapsules(self.getPreviousObservation())
        else:
            lastCapsule = None
        x, y = self.getCurrentObservation().getAgentPosition(self.index)
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
                        scores["safeFood"] = 100 / ((carried + 1) * safeDis)
                        if disToCapsule == None:
                            scores["capsule"] = 0
                        else:
                            capsuleDis = self.isZero(disToCapsule)
                            if self.timer > 0:
                                self.timer -= 1
                                scores["capsule"] = 0
                            else:
                                scores["capsule"] = 30 / capsuleDis
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
                            scores["capsule"] = 10 / capsuleDis
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

    def getSuccessors(self, currentPosition):
        """
        Find all the valid adjacent points of current postion.
        :param currentPosition: (x, y)
        :return: A list of valid adjacent points of current postion.
        """
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

    def SimpleHeuristic(self, gameState, myPos, goal):
        """
        Measures the distance between current position and nearest boarder.
        :param gameState: The current game information.
        :param currentPos: The current position of our agent.
        :return: Maze distance between current position and nearest boarder.
        """
        return self.getMazeDistance(myPos, goal)

    def aStarSearch(self, gameState, goal, heuristic=SimpleHeuristic):
        """
        The implementation of a_star search algorithm for finding the actions to approach goals.
        :param gameState: The current game information.
        :param goal: The position of goal.
        :param heuristic: The heuristic functions.
        :return: The first action of the possible actions to approach goals, and if there are no ways to the goal, it returns None.
        """
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
        return Directions.STOP

    def getMiddleGoal(self, gameState, posit):
        """
        Calculates the middle point of boundary, if has wall choose the closest point.

        :param gameState: a GameState shows current state of the game.
        :param posit: a Tuple shows current position of our agent.
        :return: a Tuple shows the the middle point of the boundary.
        """
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

    def manhattanDis(self, pos1, pos2):
        """
        Calculate the manhattan distance between two points.

        :param pos1: a Tuple shows position of first point.
        :param pos2: a Tuple shows position of second point.
        :return: an Integer shows the manhattan distance between two points
        """
        (x1, y1) = pos1
        (x2, y2) = pos2
        return abs(x1-x2) + abs(y1-y2)

    def defendBoarderGoal(self, gameState, myPos):
        """
        Search for the closest boundary position to enemy ghost. Used when no pacman detected.

        :param gameState: a GameState shows current state of the game.
        :param myPos: a Tuple shows current position of our agent.
        :return: nearestBoarder:  a Tuple shows the closest boundary position to enemy ghost.
        """
        goal, dis = self.getGhostGoal(gameState)
        minDis = 999
        nearestBoarder = self.boarders[0]
        for i in self.boarders:
            if self.getMazeDistance(i, goal) < minDis:
                minDis = self.getMazeDistance(i, goal)
                nearestBoarder = i
        return nearestBoarder


    def defenderGoal(self, point, myPos):
        """
        Find out a point with 2 manhattan distance to enemy agent, used when defender is scared.

        :param point: a Tuple shows current position of the enemy agent.
        :param myPos: a Tuple shows current position of our agent.
        :return: nearestDefender: a Tuple shows a point with 2 manhattan distance to enemy agent.
        """
        (x, y) = point
        temp = self.scanmap.adjacentValidPoints(x, y)
        targets = []
        for i in temp:
            (x, y) = i
            targets += self.scanmap.adjacentValidPoints(x, y)
        targets = list(dict.fromkeys(targets))
        targets.remove(point)

        minDis = self.getMazeDistance(myPos, targets[0])
        nearestDefender = targets[0]
        for j in targets:
            dis = self.getMazeDistance(myPos, j)
            if dis < minDis:
                minDis = dis
                nearestDefender = j
        return nearestDefender

    def boundaryPosition(self, gameState):
        """
        Gives a list of valid boundry positions.

        :param gameState: Current state of the game.
        :return: a list of valid boundry positions.
        """
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
        """
         Choose the action according to the behaviour logic.

        :param gameState: a GameState shows current state of the game.
        :return: bestAct: a Direction chosen based on MCTS.
        """
        # Go to border at the start of game or after dying.
        x, y = self.getCurrentObservation().getAgentPosition(self.index)
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        pacmen = [a for a in enemies if a.isPacman and a.getPosition() != None]
        if self.start == (x, y):
            self.toBorder = True
        if len(pacmen) > 0:
            self.toBorder = False
        if self.toBorder:
            if x < self.midWidth - 1 and self.red:
                boarder_goal, boarder_dis = self.getBoarderGoal(gameState)
                action = self.aStarSearch(gameState, boarder_goal, self.SimpleHeuristic)
                return action
            elif x > self.midWidth +1 and not self.red:
                boarder_goal, boarder_dis = self.getBoarderGoal(gameState)
                action = self.aStarSearch(gameState, boarder_goal, self.SimpleHeuristic)
                return action
            else:
                self.toBorder = False
        actions = gameState.getLegalActions(self.index)
        position = gameState.getAgentPosition(self.index)
        lastGameState = self.getPreviousObservation()

        lastEnemies = [lastGameState.getAgentState(i) for i in self.getOpponents(lastGameState)]
        lastPacmen = [a for a in lastEnemies if a.isPacman and a.getPosition() != None]
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        pacmen = [a for a in enemies if a.isPacman and a.getPosition() != None]
        ghosts = [a for a in enemies if not a.isPacman and a.getPosition() != None]
        if len(pacmen) < len(lastPacmen):
            for p in lastPacmen:
                if (self.getMazeDistance(position, p.getPosition())) > 2:
                    self.missingPacmen = list(set(lastPacmen) - set(pacmen))
                else:
                    self.defence = False
        elif len(pacmen) > len(lastPacmen):
            self.missingPacmen = []
        # Detecting any food missing out of sight.
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

        # When food is gone, defend the closest food.

        if self.foodGone != None:
            for food in currentFood:
                Dis = self.getMazeDistance(self.foodGone, food)
                if minDis > Dis:
                    minDis = Dis
                    self.foodToDefence = food


        if len(pacmen) > 0:
            for p in pacmen:
                posit = p.getPosition()
                # If scared, dodging enemy pacman and keep a distance of 2.
                if self.getMazeDistance(position,posit) <= 2 and gameState.getAgentState(self.index).scaredTimer != 0:
                    bestAct = self.aStarSearch(gameState, self.defenderGoal(posit, position), self.SimpleHeuristic)
                    return bestAct
                # If enemy in dead road, block its entry.
                elif posit in self.ourdeadPoints:
                    bestAct = self.aStarSearch(gameState, self.ourendEntry[posit], self.SimpleHeuristic)
                    return bestAct
                # Otherwise just chase it.
                else:
                    bestAct = self.aStarSearch(gameState, posit, self.SimpleHeuristic)
                    return bestAct

        if len(pacmen) == 0:
            # if enemy pacman missing in the dead road, go block the entry. Otherwise chase it.
            if len(self.missingPacmen) != 0:
                if self.missingPacmen[0].getPosition() in self.ourdeadPoints:
                    bestAct = self.aStarSearch(gameState, self.ourendEntry[self.missingPacmen[0].getPosition()],
                                               self.SimpleHeuristic)
                    return bestAct
                else:
                    bestAct = self.aStarSearch(gameState, self.missingPacmen[0].getPosition(), self.SimpleHeuristic)
                    return bestAct
        # If defending food missing in the dead road, go block the entry. Otherwise defend closed food.
        if self.foodGone in self.ourdeadPoints and position == self.ourendEntry[self.foodGone]:
                bestAct = self.aStarSearch(gameState, self.ourendEntry[self.foodGone], self.SimpleHeuristic)
                return bestAct

        if self.defence:
                bestAct = self.aStarSearch(gameState, self.foodToDefence, self.SimpleHeuristic)
                return bestAct
        else:
            # Preventing a enemy ghost from crossing border.
            if len(ghosts) != 0:
                posit, dis = self.getGhostGoal(gameState)
                if self.manhattanDis(position, posit) <= 5:
                    bestAct = self.aStarSearch(gameState, self.defendBoarderGoal(gameState,position), self.SimpleHeuristic)
                    return bestAct
            goal, dis = self.getMiddleGoal(gameState, position)
            bestAct = self.aStarSearch(gameState, goal, self.SimpleHeuristic)
            return bestAct

        return Directions.STOP