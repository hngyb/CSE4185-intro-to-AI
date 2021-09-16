###### Write Your Library Here ###########
from collections import deque
import heapq



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
    # BFS 구현을 위한 큐 초기화
    queue = deque([start_point])
    
    # 방문 체크 및 경로 추적을 위한 변수 초기화
    visited = [[0] * len(maze.mazeRaw[0]) for _ in range(len(maze.mazeRaw))]
    visited[start_point[0]][start_point[1]] = (-1, -1)

    while queue:
        x, y = queue.popleft()
        if maze.isObjective(x, y): # 목적지에 도달
            break
        for n in maze.neighborPoints(x, y):
            if visited[n[0]][n[1]] == 0: # 방문한 적이 없을 때 큐에 추가
                queue.append(n)
                visited[n[0]][n[1]] = (x, y) # 경로 추적을 위해 이전 좌표 저장
    
    # 최단 경로 추적
    cur_x, cur_y = x, y
    while (cur_x, cur_y) != (-1, -1):
        path.append((cur_x, cur_y))
        cur_x, cur_y = visited[cur_x][cur_y]

    path = path[::-1]
    return path

    ############################################################################



class Node:
    def __init__(self,parent,location):
        self.parent=parent
        self.location=location #현재 노드

        self.obj=[]

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

    # start_node, end_node 초기화
    start_node = Node(None, start_point)
    end_node = Node(None, end_point)

    # frontier로 heapq 사용 (f값을 기준으로 정렬)
    heap = []
    heapq.heappush(heap, (start_node.f, start_node))
    
    # 방문 체크 및 경로 추적을 위한 변수 초기화
    visited = [[0] * len(maze.mazeRaw[0]) for _ in range(len(maze.mazeRaw))]
    visited[start_node.location[0]][start_node.location[1]] = 1

    while heap:
        # f값이 가장 적은 node 선택 및 방문 처리
        current_node = heapq.heappop(heap)[1]
        cur_x = current_node.location[0]
        cur_y = current_node.location[1]
        visited[cur_x][cur_y] = 1


        if current_node.__eq__(end_node): # 목적지에 도달
            end_node = current_node
            break
        
        for n in maze.neighborPoints(cur_x, cur_y):
            if visited[n[0]][n[1]] == 1: # 이미 방문했을 시 skip
                continue
            
            new_node = Node(current_node, n)
            new_node.g = current_node.g + 1 # cost는 한 칸
            new_node.h = manhatten_dist(end_node.location, new_node.location)
            new_node.f = new_node.g + new_node.h
            
            # heap 안에 node가 이미 존재하면서 new_node보다 g값이 적은 경우엔 skip
            if len([i[1] for i in heap if new_node == i[1] and new_node.g > i[1].g]) > 0:
                continue
            heapq.heappush(heap, (new_node.f, new_node))

    # 최단 경로 추적
    traverse_node = end_node
    while True:
        path.append((traverse_node.location[0], traverse_node.location[1]))
        if traverse_node.__eq__(start_node):
            break
        traverse_node = traverse_node.parent

    path = path[::-1]
    return path

    ############################################################################


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #



def stage2_heuristic(p1, p2):
    # To break tie: manhatten distance * (1 + p)
    # heuristic의 값을 미세하게 증가시킴으로써, 
    # start_point보다 end_point에 가까운 노드를 선택하도록 한다.
    # p < (minimum cost of taking one step) / (expected maximum path length)

    manhatten_dist = abs(p1[0]-p2[0])+abs(p1[1]-p2[1])
    return manhatten_dist * (1 + 0.001)


def find_nearest_end_point(node, end_points): # 가장 가까운 end_point 찾기
    min_h = stage2_heuristic(node.location, end_points[0])
    nearest_end_point = end_points[0]
    nearest_end_point_idx = 0
    for i in range(1, len(end_points)):
        h = stage2_heuristic(node.location, end_points[i])
        if h < min_h:
            min_h = h
            nearest_end_point = end_points[i]
            nearest_end_point_idx = i
    
    return nearest_end_point, nearest_end_point_idx


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
    start_point=maze.startPoint()
    start_node = Node(None, start_point)

    # start_node로부터 가까운 end_point 찾아서 end_node 초기화
    end_point, end_point_idx = find_nearest_end_point(start_node, end_points)
    end_node = Node(None, end_point)

    # frontier로 heapq 사용 (f값을 기준으로 정렬)
    heap = []
    heapq.heappush(heap, (start_node.f, start_node))
    
    # 방문 체크 및 경로 추적을 위한 변수 초기화
    visited = [[0] * len(maze.mazeRaw[0]) for _ in range(len(maze.mazeRaw))]
    visited[start_node.location[0]][start_node.location[1]] = 1

    while end_points:
        while heap:
            # f값이 가장 적은 node 선택 및 방문 처리
            current_node = heapq.heappop(heap)[1]
            cur_x = current_node.location[0]
            cur_y = current_node.location[1]
            visited[cur_x][cur_y] = 1

            # 목적지에 도달
            if current_node.__eq__(end_node):
                end_points.pop(end_point_idx) # end_points에서 제거
                
                # 다음 방문할 end_point 선택
                if len(end_points) > 0:
                    end_point, end_point_idx = find_nearest_end_point(current_node, end_points)
                    end_node = Node(None, end_point)
                    
                    # heap 초기화
                    heap = []
                    heapq.heappush(heap, (current_node.f, current_node))
                    
                    # 방문 정보 초기화
                    visited = [[0] * len(maze.mazeRaw[0]) for _ in range(len(maze.mazeRaw))]
                    visited[current_node.location[0]][current_node.location[1]] = 1
                else:
                    end_node = current_node
                break
            
            for n in maze.neighborPoints(cur_x, cur_y):
                if visited[n[0]][n[1]] == 1: # 이미 방문했을 시 skip
                    continue
                
                new_node = Node(current_node, n)
                new_node.g = current_node.g + 1 # cost는 한 칸
                new_node.h = stage2_heuristic(end_node.location, new_node.location)
                new_node.f = new_node.g + new_node.h
                
                # heap 안에 node가 이미 존재하면서 new_node보다 g값이 적은 경우엔 skip
                if len([i[1] for i in heap if new_node == i[1] and new_node.g > i[1].g]) > 0:
                    continue
                heapq.heappush(heap, (new_node.f, new_node))
    
    # 최단 경로 추적
    traverse_node = end_node
    while True:
        path.append((traverse_node.location[0], traverse_node.location[1]))
        if traverse_node.parent is None:
            break
        traverse_node = traverse_node.parent

    path = path[::-1]
    return path

    ############################################################################



# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #

def mst(edges, visited, start_point):
    # Prim Algorithm
    cost_sum=0
    ####################### Write Your Code Here ################################
    mst = []
    visited[start_point[0]][start_point[1]] = 1
    candidate = edges[start_point[0]][start_point[1]]
    heapq.heapify(candidate)

    while candidate:
        cost, u, v = heapq.heappop(candidate)
        if visited[v[0]][v[1]] == 0:
            visited[v[0]][v[1]] = 1
            mst.append((u, v))
            cost_sum += cost
            candidate = [] # 기존 Prim Algorithm과 다른 점

            for edge in edges[v[0]][v[1]]:
                if visited[edge[2][0]][edge[2][1]] == 0:
                    heapq.heappush(candidate, edge)



    return cost_sum, mst

    ############################################################################


def stage3_heuristic(p1, p2):
    # To break tie: manhatten distance * (1 + p)
    manhatten_dist = abs(p1[0]-p2[0])+abs(p1[1]-p2[1])
    return manhatten_dist * (1 + 0.001)

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
    start_point = maze.startPoint()
    end_points.append(start_point)
    edges = [[ [] for _ in range(len(maze.mazeRaw[0]))]  for _ in range(len(maze.mazeRaw))]
    # 간선 비용 계산
    for u in end_points:
        for v in end_points:
            if u == v: continue
            cost = stage3_heuristic(u, v)
            edges[u[0]][u[1]].append([cost, u, v])

    visited = [[0] * len(maze.mazeRaw[0]) for _ in range(len(maze.mazeRaw))]
    cost_sum, mst_list = mst(edges, visited, start_point)
    

    start_node = Node(None, start_point)
    end_node = Node(None, mst_list[0][1])
    visited = [[0] * len(maze.mazeRaw[0]) for _ in range(len(maze.mazeRaw))]
    heap = []
    heapq.heappush(heap, (start_node.f, start_node))
    while mst_list:
        while heap:
            # f값이 가장 적은 node 선택 및 방문 처리
            current_node = heapq.heappop(heap)[1]
            cur_x = current_node.location[0]
            cur_y = current_node.location[1]
            visited[cur_x][cur_y] = 1

            # 목적지에 도달
            if current_node.__eq__(end_node):
                mst_list.pop(0)
                
                # 다음 방문할 end_point 선택
                if len(mst_list) > 0:
                    end_node = Node(None, mst_list[0][1])
                    
                    # heap 초기화
                    heap = []
                    heapq.heappush(heap, (current_node.f, current_node))
                    
                    # 방문 정보 초기화
                    visited = [[0] * len(maze.mazeRaw[0]) for _ in range(len(maze.mazeRaw))]
                    visited[current_node.location[0]][current_node.location[1]] = 1
                else:
                    end_node = current_node
                break
            
            for n in maze.neighborPoints(cur_x, cur_y):
                if visited[n[0]][n[1]] == 1: # 이미 방문했을 시 skip
                    continue
                
                new_node = Node(current_node, n)
                new_node.g = current_node.g + 1 # cost는 한 칸
                new_node.h = stage3_heuristic(end_node.location, new_node.location)
                new_node.f = new_node.g + new_node.h
                
                # heap 안에 node가 이미 존재하면서 new_node보다 g값이 적은 경우엔 skip
                if len([i[1] for i in heap if new_node == i[1] and new_node.g > i[1].g]) > 0:
                    continue
                heapq.heappush(heap, (new_node.f, new_node))
    
    # 최단 경로 추적
    traverse_node = end_node
    while True:
        path.append((traverse_node.location[0], traverse_node.location[1]))
        if traverse_node.parent is None:
            break
        traverse_node = traverse_node.parent

    path = path[::-1]
    return path

    ############################################################################
