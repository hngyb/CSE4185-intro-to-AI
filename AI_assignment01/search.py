###### Write Your Library Here ###########
from collections import deque
import heapq
from copy import deepcopy


#########################################


def search(maze, func):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_four_circles": astar_four_circles,
        "astar_many_circles": astar_many_circles
    }.get(func)(maze)


# -------------------- Stage 01: One circle - BFS Algorithm ------------------------ #

def bfs(maze):
    """
    [문제 01] 제시된 stage1의 맵 세가지를 BFS Algorithm을 통해 최단 경로를 return하시오.(20점)
    """
    start_point=maze.startPoint()

    path=[]

    ####################### Write Your Code Here ################################
    # BFS 구현을 위한 frontier(큐) 초기화 (point, path_to_point)
    frontier = deque([(start_point, [start_point])])
    
    # 방문 체크를 위한 변수 초기화
    visited = []

    while frontier:
        current_point, current_path = frontier.popleft()
        cur_x, cur_y = current_point
        visited.append(current_point)
        
        if maze.isObjective(cur_x, cur_y): # GOAL TEST
            return current_path

        for neighbor in maze.neighborPoints(cur_x, cur_y):
            if neighbor not in visited: # 방문 여부 체크
                frontier.append((neighbor, current_path + [neighbor]))
    
    return path

    ############################################################################



class Node:
    def __init__(self,parent,location):
        self.parent=parent
        self.location=location # 현재 노드

        self.obj=[] # 방문하지 않은 목적지

        # F = G+H
        self.f=0
        self.g=0
        self.h=0
    
    def __eq__(self, other):
        return self.location==other.location and str(self.obj)==str(other.obj)

    def __le__(self, other):
        return self.g+self.h<=other.g+other.h

    def __lt__(self, other):
        return self.g+self.h<other.g+other.h

    def __gt__(self, other):
        return self.g+self.h>other.g+other.h

    def __ge__(self, other):
        return self.g+self.h>=other.g+other.h


# -------------------- Stage 01: One circle - A* Algorithm ------------------------ #

def manhatten_dist(p1,p2):
    return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def astar(maze):

    """
    [문제 02] 제시된 stage1의 맵 세가지를 A* Algorithm을 통해 최단경로를 return하시오.(20점)
    (Heuristic Function은 위에서 정의한 manhatten_dist function을 사용할 것.)
    """
    start_point=maze.startPoint()

    end_point=maze.circlePoints()[0]

    path=[]

    ####################### Write Your Code Here ################################

    # start_node 초기화
    start_node = Node(None, start_point)
    start_node.obj = end_point # 방문해야 할 목적지

    # frontier 초기화 (node, path_to_node)
    frontier = []
    heapq.heappush(frontier, \
        (manhatten_dist(start_point, end_point), (start_node, [start_point])))
    
    # 방문 체크를 위한 변수 초기화
    visited = dict()

    while frontier:
        # f값이 가장 적은 node 선택 및 방문 처리
        current_node, current_path = heapq.heappop(frontier)[1]
        cur_x, cur_y = current_node.location
        visited[current_node.location] = current_node.g
        

        if maze.isObjective(cur_x, cur_y): # GOAL TEST
            return current_path

        for neighbor in maze.neighborPoints(cur_x, cur_y):
            if neighbor not in visited: # 방문 여부 체크
                new_node = Node(current_node, neighbor)
                new_node.g = current_node.g + 1
                new_node.h = manhatten_dist(new_node.location, end_point)
                new_node.f = new_node.g + new_node.h
                updated_path = current_path + [neighbor]
                heapq.heappush(frontier, (new_node.f, (new_node, updated_path)))

    return path

    ############################################################################


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #

def stage2_heuristic(node):
    left_end_points = deepcopy(node.obj)
    cur_point = node.location

    if len(left_end_points) == 0:
        return 0
    
    max_distance = 0
    for end_point in left_end_points:
        distance = manhatten_dist(cur_point, end_point)
        if max_distance < distance:
            max_distance = distance
        
    return max_distance


def goal_test(node):
    if len(node.obj) == 0: # 모든 목적지를 방문했다면 True 반환
        return True
    return False


def astar_four_circles(maze):
    """
    [문제 03] 제시된 stage2의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage2_heuristic function을 직접 정의하여 사용해야 한다.)
    """

    end_points=maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################
    # start_node 초기화
    start_point = maze.startPoint()
    start_node = Node(None, start_point)
    start_node.obj = end_points # 방문하지 않은 목적지

    # frontier 초기화 (node, path_to_node)
    frontier = []
    heapq.heappush(frontier, (stage2_heuristic(start_node), (start_node, [start_point])))

    # 방문 체크를 위한 변수 초기화
    visited = dict()
    while frontier:
        # f값이 가장 적은 node 선택 및 방문 처리
        current_node, current_path = heapq.heappop(frontier)[1]
        cur_x, cur_y = current_node.location
        visited[(current_node.location, tuple(current_node.obj))] = current_node.g
        
        if goal_test(current_node): # GOAL TEST
            return current_path

        for neighbor in maze.neighborPoints(cur_x, cur_y):
            if neighbor not in current_node.obj: # 다음 노드가 목적지가 아닐 때
                next_node = Node(current_node, neighbor)
                next_node.obj = current_node.obj
            else: # 다음 노드가 목적지일 때 (방문하지 않은 목적지 수정)
                new_obj = []
                for obj in current_node.obj:
                    if obj != neighbor:
                        new_obj.append(obj)
                new_obj.sort()
                next_node = Node(current_node, neighbor)
                next_node.obj = new_obj

            if (next_node.location, tuple(next_node.obj)) not in visited: # 방문 여부 체크
                next_node.g = current_node.g + 1
                next_node.h = stage2_heuristic(next_node)
                next_node.f = next_node.g + next_node.h
                updated_path = current_path + [neighbor]
                heapq.heappush(frontier, (next_node.f, (next_node, updated_path)))

    return path

    ############################################################################


# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #

def neighborPoints(maze, row, col):
    possibleNeighbors = [
        (row + 1, col),
        (row - 1, col),
        (row, col + 1),
        (row, col - 1)
    ]
    neighbors = []
    for r, c in possibleNeighbors:
        if maze.choose_move(r, c):
            neighbors.append((r,c))

    return neighbors

def astar_distance(maze, start_point, end_point):
    path=[]

    start_node = Node(None, start_point)

    frontier = []
    heapq.heappush(frontier, (manhatten_dist(start_point, end_point), (start_node, [start_point])))
    
    visited = dict()

    while frontier:
        current_node, current_path = heapq.heappop(frontier)[1]
        cur_x, cur_y = current_node.location
        visited[current_node.location] = current_node.g

        if current_node.location == end_point:
            return len(current_path)

        for neighbor in neighborPoints(maze, cur_x, cur_y):

            if neighbor not in visited:
                new_node = Node(current_node, neighbor)
                new_node.g = current_node.g + 1
                new_node.h = manhatten_dist(new_node.location, end_point)
                new_node.f = new_node.g + new_node.h
                updated_path = current_path + [neighbor]
                heapq.heappush(frontier, (new_node.f, (new_node, updated_path)))

    return len(path)

def calculate_distances(maze, end_points, cached_distances):
    vertices = deepcopy(end_points)
    distances = [[ [] for _ in range(len(maze.mazeRaw[0]))]  for _ in range(len(maze.mazeRaw))]
    
    for point in end_points:
        vertices.remove(point)
        for vertex in vertices:
            cached = 0
            for edge in cached_distances[point[0]][point[1]]: # cached_distances에 존재하는지 확인
                if edge[1] == vertex:
                    distances[point[0]][point[1]].append(edge)
                    distances[vertex[0]][vertex[1]].append((edge[0], point))
                    cached = 1
                    break
            if cached == 0:
                distance = astar_distance(maze, point, vertex)
                distances[point[0]][point[1]].append((distance, vertex))
                distances[vertex[0]][vertex[1]].append((distance, point))
                
                # cached_distances 업데이트
                cached_distances[point[0]][point[1]].append((distance, vertex))
                cached_distances[vertex[0]][vertex[1]].append((distance, point))
    
    return distances


def mst(maze, nearest_end_point, left_end_points, cached_distances):
    cost_sum=0
    ####################### Write Your Code Here ################################
    # Prim Algorithm
    cur_point = nearest_end_point

    # 남은 left_end_points들의 거리 계산
    distances = calculate_distances(maze, left_end_points, cached_distances)
    visited = []
    visited.append(cur_point)

    heap = distances[cur_point[0]][cur_point[1]]
    heapq.heapify(heap)

    while heap:
        cost, point = heapq.heappop(heap)
        if point not in visited:
            visited.append(point)
            cost_sum += cost

            for edge in distances[point[0]][point[1]]:
                if edge[1] not in visited:
                    heapq.heappush(heap, edge)

    return cost_sum

    ############################################################################

def stage3_heuristic(maze, node, cached_distances):
    total_cost = 0
    cur_x, cur_y = node.location
    left_end_points = deepcopy(node.obj)

    if len(node.obj) == 0:
        return total_cost
    
    # 현재 위치부터 가장 가까운 end_point까지의 거리 계산
    distances = []
    for point in left_end_points:
        cached = 0
        for edge in cached_distances[cur_x][cur_y]:
            if edge[1] == point:
                distances.append(edge)
                cached = 1
                break
        if cached == 0:
            distance = astar_distance(maze, (cur_x, cur_y), point)
            distances.append((distance, point))
            
            # cached_distances 업데이트
            cached_distances[cur_x][cur_y].append((distance, point))
            cached_distances[point[0]][point[1]].append((distance, (cur_x, cur_y)))
    
    cost, nearest_end_point = min(distances)
    total_cost += cost
    total_cost += mst(maze, nearest_end_point, left_end_points, cached_distances) # 남은 end_point들의 mst

    return total_cost

def astar_many_circles(maze):
    """
    [문제 04] 제시된 stage3의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage3_heuristic function을 직접 정의하여 사용해야 하고, minimum spanning tree
    알고리즘을 활용한 heuristic function이어야 한다.)
    """

    end_points= maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################
    # start_node 초기화
    start_point = maze.startPoint()
    start_node = Node(None, start_point)
    start_node.obj = end_points # 방문하지 않은 목적지

    # point들의 거리를 저장(cached)
    cached_distances = [[ [] for _ in range(len(maze.mazeRaw[0]))] for _ in range(len(maze.mazeRaw))]

    # frontier 초기화 (node, path_to_node)
    frontier = []
    heapq.heappush(frontier, (stage3_heuristic(maze, start_node, cached_distances), (start_node, [start_point])))

    # 방문 체크를 위한 변수 초기화
    visited = dict()

    while frontier:
        # f값이 가장 적은 node 선택 및 방문 처리
        current_node, current_path = heapq.heappop(frontier)[1]
        cur_x, cur_y = current_node.location
        visited[(current_node.location, tuple(current_node.obj))] = current_node.g
        
        if goal_test(current_node): # GOAL TEST
            return current_path
        
        for neighbor in maze.neighborPoints(cur_x, cur_y):
            if neighbor not in current_node.obj: # 다음 노드가 목적지가 아닐 때
                next_node = Node(current_node, neighbor)
                next_node.obj = current_node.obj
            else: # 다음 노드가 목적지일 때 (방문하지 않은 목적지 수정)
                new_obj = []
                for obj in current_node.obj:
                    if obj != neighbor:
                        new_obj.append(obj)
                new_obj.sort()
                next_node = Node(current_node, neighbor)
                next_node.obj = new_obj

            if (next_node.location, tuple(next_node.obj)) not in visited: # 방문 여부 체크
                next_node.g = current_node.g + 1
                next_node.h = stage3_heuristic(maze, next_node, cached_distances)
                next_node.f = next_node.g + next_node.h
                updated_path = current_path + [neighbor]
                heapq.heappush(frontier, (next_node.f, (next_node, updated_path)))

    return path

    ############################################################################
