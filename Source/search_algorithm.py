import pygame
import graphUI
import math
from node_color import white, yellow, black, red, blue, purple, orange, green, grey


"""
Feel free print graph, edges to console to get more understand input.
Do not change input parameters
Create new function/file if necessary
"""


def BFS(graph, edges, edge_id, start, goal):
    """
    BFS search
    """
    
    visited = []
    parents = []
    queue = []

    for i in range(0, len(graph)):
        visited.append(False)
        parents.append(-1) 
    
    queue.append(start)
    visited[start] = True
    
    while queue:
        v = queue.pop(0)
        graph[v][3] = yellow
        graph[v][2] = white
        graphUI.updateUI()
        for w in graph[v][1]:
            if visited[w] == False:
                visited[w] = True
                queue.append(w)
                parents[w] = v
                edges[edge_id(v, w)][1] = white
                graph[w][3] = red
                graph[w][2] = white
                graphUI.updateUI()
            if w == goal:
                graph[start][3] = orange
                graph[goal][3] = purple
                goal_temp = goal
                result = []
                while parents[goal_temp] != -1:
                    result.append(parents[goal_temp])
                    goal_temp = parents[goal_temp]
                path = [] 
                path.append(goal) 
                path.extend(result)
                for i in range(len(path) - 1):
                   edges[edge_id(path[i], path[i + 1])][1] = green
                graphUI.updateUI()
                return

        graph[v][3] = blue
        graph[v][2] = white
        graphUI.updateUI()        

    # TODO: your code
    print("Implement BFS algorithm.")
    pass


def DFS(graph, edges, edge_id, start, goal):
    """
    DFS search
    """
    visited = []
    parents = []
    stack = []
    path = []

    for i in range(0, len(graph)):
        visited.append(False)
        parents.append(-1) 
        path.append([])

    stack.append(start)
    visited[start] = True

    while stack:
        v = stack[-1]
        graph[v][3] = yellow
        graph[v][2] = white
        graphUI.updateUI()

        if visited[v] == False:
            visited[v] = True

        if v == goal:
                graph[start][3] = orange
                graph[goal][3] = purple
                goal_temp = goal
                result = []
                while parents[goal_temp] != -1:
                    result.append(parents[goal_temp])
                    goal_temp = parents[goal_temp]
                path = [] 
                path.append(goal) 
                path.extend(result)
                for i in range(len(path) - 1):
                   edges[edge_id(path[i], path[i + 1])][1] = green
                graphUI.updateUI()
                return

        for w in graph[v][1]:
            if visited[w] == False:
                stack.append(w)
                parents[w] = v
                edges[edge_id(v, w)][1] = white
                graph[w][3] = red
                graph[w][2] = white
                graphUI.updateUI()
                break
    
        if stack[-1] == v:
            stack.pop()
        graph[v][3] = blue
        graph[v][2] = white
        graphUI.updateUI()

    # TODO: your code
    print("Implement DFS algorithm.")
    pass


def UCS(graph, edges, edge_id, start, goal):
    """
    Uniform Cost Search search
    """
    visited = []
    parents = []
    costs = []
    queue = []
    path = []

    for i in range(0, len(graph)):
        visited.append(False)
        parents.append(-1)
        path.append([])
        

    queue.append(start)
    costs.append(0)
    path[start] = []
    visited[start] = True
    
    while queue:
        min_cost_index = costs.index(min(costs))
        v = queue.pop(min_cost_index)
        cost = costs.pop(min_cost_index)
        graph[v][3] = yellow
        graph[v][2] = white
        graphUI.updateUI()

        if visited[v] == False:
            visited[v] = True
        
        if v == goal:
            path[v].append(goal)
            graph[start][3] = orange
            graph[goal][3] = purple
            for i in range(len(path[v]) - 1):
               edges[edge_id(path[v][i], path[v][i + 1])][1] = green
            graphUI.updateUI()
            return 


        for w in graph[v][1]:
            
            total_cost = cost + math.sqrt((graph[v][0][0] - graph[w][0][0])**2 + (graph[v][0][1] - graph[w][0][1])**2)
            if visited[w] == False:
                queue.append(w)
                costs.append(total_cost)
                path[w].extend(path[v])
                path[w].append(v)
                visited[w] = True
                parents[w] = v
                edges[edge_id(v, w)][1] = white
                graph[w][3] = red
                graph[w][2] = white
                graphUI.updateUI()
            elif w in queue and total_cost < costs[queue.index(w)]:
                edges[edge_id(parents[w], w)][1] = grey
                costs[queue.index(w)] = total_cost
                parents[w] = v
                path[w] = []
                path[w].extend(path[v])
                path[w].append(v)
                edges[edge_id(parents[w], w)][1] = white
                graphUI.updateUI()

        graph[v][3] = blue
        graph[v][2] = white
        graphUI.updateUI()

    # TODO: your code
    print("Implement Uniform Cost Search algorithm.")
    pass


def AStar(graph, edges, edge_id, start, goal):
    """
    A star search
    """
    # TODO: your code
    visited = []
    parents = []
    costs = []
    f = []
    f_closed = []
    queue = []
    path = []

    for i in range(0, len(graph)):
        visited.append(False)
        parents.append(-1)
        path.append([])
        f_closed.append(-1)

    queue.append(start)
    costs.append(0)
       
    f.append(costs[0] + math.sqrt((graph[start][0][0] - graph[goal][0][0])**2 + (graph[start][0][1] - graph[goal][0][1])**2))
    path[start] = []  
    visited[start] = True
    f_closed[start] = f[0]

    print("eucliden:")
    for i in graph:
        print(i, "   ",math.sqrt((i[0][0] - graph[goal][0][0])**2 + (i[0][1] - graph[goal][0][1])**2))

    while queue:
        min_f_index = f.index(min(f))
        v = queue.pop(min_f_index)
        f_v = f.pop(min_f_index)
        cost_v = costs.pop(min_f_index)
        f_closed[v] = f_v
        graph[v][3] = yellow
        graph[v][2] = white
        graphUI.updateUI()
        

        if visited[v] == False:
            visited[v] = True

        if v == goal:
            path[v].append(goal)
            graph[start][3] = orange
            graph[goal][3] = purple
            for i in range(len(path[v]) - 1):
               edges[edge_id(path[v][i], path[v][i + 1])][1] = green
            graphUI.updateUI()
            return

        for w in graph[v][1]:
            cost_total = cost_v + math.sqrt((graph[v][0][0] - graph[w][0][0])**2 + (graph[v][0][1] - graph[w][0][1])**2)
            print(w, "cost: ", cost_total)
            f_total = cost_total + math.sqrt((graph[goal][0][0] - graph[w][0][0])**2 + (graph[goal][0][1] - graph[w][0][1])**2)
            print(v, " ", w,"   : ", f_total)
            if visited[w] == False:
                queue.append(w)
                costs.append(cost_total)
                f.append(f_total)
                path[w].extend(path[v])
                path[w].append(v)
                visited[w] = True
                parents[w] = v
                edges[edge_id(v, w)][1] = white
                graph[w][3] = red
                graph[w][2] = white
                graphUI.updateUI()
            elif w in queue and f_total < f[queue.index(w)]:
                edges[edge_id(parents[w], w)][1] = grey
                f[queue.index(w)] = f_total
                parents[w] = v
                path[w] = []
                path[w].extend(path[v])
                path[w].append(v)
                edges[edge_id(parents[w], w)][1] = white
                graphUI.updateUI()
            elif visited[w] == True and f_total < f_closed[w]:
                queue.append(w)
                costs.append(cost_total)
                f.append(f_total)
                f_closed[w] = f_total
                parents[w] = v
                path[w] = []
                path[w].extend(path[v])
                path[w].append(v)
                edges[edge_id(parents[w], w)][1] = white
                graphUI.updateUI()

        graph[v][3] = blue
        graph[v][2] = white
        graphUI.updateUI()

    print("Implement A* algorithm.")
    pass

def GBFS(graph, edges, edge_id, start, goal):
    """
    Greedy Best First Search search
    """
    parents = []
    heuristics = []
    queue = []
    path = []

    for i in range(0, len(graph)):
        parents.append(-1)
        path.append([])
        

    queue.append(start)
    heuristics.append(math.sqrt((graph[goal][0][0] - graph[start][0][0])**2 + (graph[goal][0][1] - graph[start][0][1])**2))
    path[start] = []
    
    while queue:
        print("before")
        print(queue)
        print(heuristics)
        min_heuristic_index = heuristics.index(min(heuristics))
        v = queue.pop(min_heuristic_index)
        heuristic = heuristics.pop(min_heuristic_index)
        print("pop")
        print(v)
        print(heuristic)
        graph[v][3] = yellow
        graph[v][2] = white
        graphUI.updateUI()

        queue = []
        heuristics = []
        print("after")
        print(queue)
        print(heuristics)
        
        
        if v == goal:
            path[v].append(goal)
            graph[start][3] = orange
            graph[goal][3] = purple
            for i in range(len(path[v]) - 1):
               edges[edge_id(path[v][i], path[v][i + 1])][1] = green
            graphUI.updateUI()
            return 


        for w in graph[v][1]:
            
            heuristic_temp = math.sqrt((graph[goal][0][0] - graph[w][0][0])**2 + (graph[goal][0][1] - graph[w][0][1])**2)
            print(v,w, ": ", heuristic_temp)

            queue.append(w)
            heuristics.append(heuristic_temp)
            path[w].extend(path[v])
            path[w].append(v)
            parents[w] = v
            edges[edge_id(v, w)][1] = white
            graph[w][3] = red
            graph[w][2] = white
            graphUI.updateUI()

        graph[v][3] = blue
        graph[v][2] = white
        graphUI.updateUI()

    # TODO: your code
    print("Implement Greedy Best First Search algorithm.")
    pass


def example_func(graph, edges, edge_id, start, goal):
    """
    This function is just show some basic feature that you can use your project.
    @param graph: list - contain information of graph (same value as global_graph)
                    list of object:
                     [0] : (x,y) coordinate in UI
                     [1] : adjacent node indexes
                     [2] : node edge color
                     [3] : node fill color
                Ex: graph = [
                                [
                                    (139, 140),             # position of node when draw on UI
                                    [1, 2],                 # list of adjacent node
                                    (100, 100, 100),        # grey - node edged color
                                    (0, 0, 0)               # black - node fill color
                                ],
                                [(312, 224), [0, 4, 2, 3], (100, 100, 100), (0, 0, 0)],
                                ...
                            ]
                It means this graph has Node 0 links to Node 1 and Node 2.
                Node 1 links to Node 0,2,3 and 4.
    @param edges: dict - dictionary of edge_id: [(n1,n2), color]. Ex: edges[edge_id(0,1)] = [(0,1), (0,0,0)] : set color
                    of edge from Node 0 to Node 1 is black.
    @param edge_id: id of each edge between two nodes. Ex: edge_id(0, 1) : id edge of two Node 0 and Node 1
    @param start: int - start vertices/node
    @param goal: int - vertices/node to search
    @return:
    """

    # Ex1: Set all edge from Node 1 to Adjacency node of Node 1 is green edges.
    node_1 = graph[1]
    for adjacency_node in node_1[1]:
        edges[edge_id(1, adjacency_node)][1] = green
    graphUI.updateUI()

    # Ex2: Set color of Node 2 is Red
    graph[2][3] = red
    graphUI.updateUI()

    # Ex3: Set all edge between node in a array.
    path = [4, 7, 9]  # -> set edge from 4-7, 7-9 is blue
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = blue
    graphUI.updateUI()
