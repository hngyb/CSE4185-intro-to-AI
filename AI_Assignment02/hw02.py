# -*- coding: utf-8 -*- 
import math
from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

## Example Agent
class ReflexAgent(Agent):

  def Action(self, gameState):

    move_candidate = gameState.getLegalActions()

    scores = [self.reflex_agent_evaluationFunc(gameState, action) for action in move_candidate]
    bestScore = max(scores)
    Index = [index for index in range(len(scores)) if scores[index] == bestScore]
    get_index = random.choice(Index)

    return move_candidate[get_index]

  def reflex_agent_evaluationFunc(self, currentGameState, action):

    successorGameState = currentGameState.generatePacmanSuccessor(action)
    newPos = successorGameState.getPacmanPosition()
    oldFood = currentGameState.getFood()
    newGhostStates = successorGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

    return successorGameState.getScore()



def scoreEvalFunc(currentGameState):

  return currentGameState.getScore()

class AdversialSearchAgent(Agent):

  def __init__(self, getFunc ='scoreEvalFunc', depth ='2'):
    self.index = 0
    self.evaluationFunction = util.lookup(getFunc, globals())

    self.depth = int(depth)


class MinimaxAgent(AdversialSearchAgent):
  """
    [문제 01] MiniMax의 Action을 구현하시오. (20점)
    (depth와 evaluation function은 위에서 정의한 self.depth and self.evaluationFunction을 사용할 것.)
  """
  def Action(self, gameState):
    ####################### Write Your Code Here ################################
    move_candidate = gameState.getLegalActions()
    scores = []
    for action in move_candidate:
      successorGameState = gameState.generatePacmanSuccessor(action)
      scores.append(self.minValue(successorGameState, 0, 1))
    
    bestScore = max(scores)
    Index = [index for index in range(len(scores)) if scores[index] == bestScore]
    get_index = random.choice(Index)

    return move_candidate[get_index]

  def isTerminal(self, gameState, currentDepth):
    return True if currentDepth == self.depth or gameState.isWin() or gameState.isLose() else False

  def isPacman(self, agentIndex):
    return True if agentIndex == 0 else False
    
  def maxValue(self, gameState, currentDepth):
    if self.isTerminal(gameState, currentDepth): # Terminal Test
      return self.evaluationFunction(gameState)
    
    move_candidate = gameState.getLegalActions()
    scores = []

    for action in move_candidate:
      successorGameState = gameState.generatePacmanSuccessor(action)
      scores.append(self.minValue(successorGameState, currentDepth, 1))
    
    return max(scores)
  
  def minValue(self, gameState, currentDepth, agentIndex):
    if self.isTerminal(gameState, currentDepth): # Terminal Test
      return self.evaluationFunction(gameState)

    move_candidate = gameState.getLegalActions(agentIndex)
    scores = []
    ghostNum = gameState.getNumAgents() - 1

    for action in move_candidate:
      successorGameState = gameState.generateSuccessor(agentIndex, action)

      if agentIndex == ghostNum: # 팩맨으로 넘어감
        nextDepth = currentDepth + 1
        scores.append(self.maxValue(successorGameState, nextDepth))
      else: # 다음 고스트
        nextGhost = agentIndex + 1
        scores.append(self.minValue(successorGameState, currentDepth, nextGhost))
      
    return min(scores)

    ############################################################################


class AlphaBetaAgent(AdversialSearchAgent):
  """
    [문제 02] AlphaBeta의 Action을 구현하시오. (25점)
    (depth와 evaluation function은 위에서 정의한 self.depth and self.evaluationFunction을 사용할 것.)
  """
  def Action(self, gameState):
    ####################### Write Your Code Here ################################
    move_candidate = gameState.getLegalActions()
    scores = []
    alpha = float('-inf')
    beta = float('inf')
    bestScore = float('-inf')

    for action in move_candidate:
      successorGameState = gameState.generatePacmanSuccessor(action)
      scores.append(self.minValue(successorGameState, 0, 1, alpha, beta))
      
      bestScore = max(scores)
      alpha = max(alpha, bestScore)

    Index = [index for index in range(len(scores)) if scores[index] == bestScore]
    get_index = random.choice(Index)

    return move_candidate[get_index]

  def isTerminal(self, gameState, currentDepth):
    return True if currentDepth == self.depth or gameState.isWin() or gameState.isLose() else False
  
  def isPacman(self, agentIndex):
    return True if agentIndex == 0 else False

  def maxValue(self, gameState, currentDepth, alpha, beta):
    if self.isTerminal(gameState, currentDepth): # Terminal Test
      return self.evaluationFunction(gameState)

    move_candidate = gameState.getLegalActions()
    scores = []
    value = float('-inf')

    for action in move_candidate:
      successorGameState = gameState.generatePacmanSuccessor(action)
      scores.append(self.minValue(successorGameState, currentDepth, 1, alpha, beta))
      
      value = max(scores)
      alpha = max(alpha, value)

      if value > beta: return value # pruning
    
    return value

  def minValue(self, gameState, currentDepth, agentIndex, alpha, beta):
    if self.isTerminal(gameState, currentDepth): # Terminal Test
      return self.evaluationFunction(gameState)
    
    move_candidate = gameState.getLegalActions(agentIndex)
    scores = []
    value = float('inf')
    ghostNum = gameState.getNumAgents() - 1

    for action in move_candidate:
      successorGameState = gameState.generateSuccessor(agentIndex, action)

      if agentIndex == ghostNum:
        nextDepth = currentDepth + 1
        scores.append(self.maxValue(successorGameState, nextDepth, alpha, beta))
      else:
        nextGhost = agentIndex + 1
        scores.append(self.minValue(successorGameState, currentDepth, nextGhost, alpha, beta))
      
      value = min(scores)
      beta = min(beta, value)
      
      if value < alpha: return value # pruning
      
    return value

    ############################################################################



class ExpectimaxAgent(AdversialSearchAgent):
  """
    [문제 03] Expectimax의 Action을 구현하시오. (25점)
    (depth와 evaluation function은 위에서 정의한 self.depth and self.evaluationFunction을 사용할 것.)
  """
  def Action(self, gameState):
    ####################### Write Your Code Here ################################
    move_candidate = gameState.getLegalActions()
    scores = []
    for action in move_candidate:
      successorGameState = gameState.generatePacmanSuccessor(action)
      scores.append(self.minValue(successorGameState, 0, 1))
    
    bestScore = max(scores)
    Index = [index for index in range(len(scores)) if scores[index] == bestScore]
    get_index = random.choice(Index)

    return move_candidate[get_index]

  def isTerminal(self, gameState, currentDepth):
    return True if currentDepth == self.depth or gameState.isWin() or gameState.isLose() else False

  def isPacman(self, agentIndex):
    return True if agentIndex == 0 else False
  
  def maxValue(self, gameState, currentDepth):
    if self.isTerminal(gameState, currentDepth): # Terminal Test
      return self.evaluationFunction(gameState)
    
    move_candidate = gameState.getLegalActions()
    scores = []

    for action in move_candidate:
      successorGameState = gameState.generatePacmanSuccessor(action)
      scores.append(self.minValue(successorGameState, currentDepth, 1))
    
    return max(scores)
  
  def minValue(self, gameState, currentDepth, agentIndex):
    if self.isTerminal(gameState, currentDepth): # Terminal Test
      return self.evaluationFunction(gameState)
    
    move_candidate = gameState.getLegalActions(agentIndex)
    scores = []
    ghostNum = gameState.getNumAgents() - 1

    for action in move_candidate:
      successorGameState = gameState.generateSuccessor(agentIndex, action)

      if agentIndex == ghostNum:
        nextDepth = currentDepth + 1
        scores.append(self.maxValue(successorGameState, nextDepth))
      else:
        nextGhost = agentIndex + 1
        scores.append(self.minValue(successorGameState, currentDepth, nextGhost))
    
    return float(sum(scores)/len(move_candidate))

    ############################################################################
