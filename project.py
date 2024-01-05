import heapq
import numpy as np
import networkx as nx
from queue import PriorityQueue

def read_input_file(file_path):
   
    with open(file_path, 'r') as file:
        lines = file.readlines()

    
    adjacency_str, bandwidth_str, delay_str, reliability_str = ''.join(lines).split('\n\n')

   
    adjacency_matrix = np.array([list(map(int, row.split(':'))) for row in adjacency_str.strip().split('\n')])
    bandwidth_matrix = np.array([list(map(int, row.split(':'))) for row in bandwidth_str.strip().split('\n')])
    delay_matrix = np.array([list(map(int, row.split(':'))) for row in delay_str.strip().split('\n')])
    reliability_matrix = np.array([list(map(float, row.split(':'))) for row in reliability_str.strip().split('\n')])

    return adjacency_matrix, bandwidth_matrix, delay_matrix, reliability_matrix

def dijkstra_algorithm(graph, start, goal):
  
    priority_queue = [(0, start)]
    
    distances = {node: float('infinity') for node in graph.nodes}
    distances[start] = 0
  
    previous = {node: None for node in graph.nodes}

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node == goal:
            path = reconstruct_path(previous, start, goal)
            return path

        for neighbor, edge_weight in graph[current_node].items():
            new_distance = current_distance + edge_weight['weight']
            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                previous[neighbor] = current_node
                heapq.heappush(priority_queue, (new_distance, neighbor))

    return None

def bellman_ford_algorithm(graph, source, destination):
   
    _, path = nx.single_source_bellman_ford(graph, source, target=destination)
    return path

def floyd_warshall_algorithm(graph):
    num_nodes = graph.number_of_nodes()
    adjacency_matrix = nx.to_numpy_array(graph, weight='weight')

   
    distance_matrix = np.copy(adjacency_matrix)

    for k in range(num_nodes):
        for i in range(num_nodes):
            for j in range(num_nodes):
                if distance_matrix[i, j] > distance_matrix[i, k] + distance_matrix[k, j]:
                    distance_matrix[i, j] = distance_matrix[i, k] + distance_matrix[k, j]

    return distance_matrix

def constraints_check(path, bandwidth_matrix, delay_matrix, reliability_matrix):
   
    bandwidth_constraint = np.min(bandwidth_matrix[path[:-1], path[1:]]) >= 5
    delay_constraint = np.max(delay_matrix[path[:-1], path[1:]]) < 40
    reliability_constraint = np.min(reliability_matrix[path[:-1], path[1:]]) > 0.70

   

    return bandwidth_constraint and delay_constraint and reliability_constraint

def adjacency_matrix_to_graph(adjacency_matrix, create_using=nx.Graph):
  
    G = create_using()
    for i in range(adjacency_matrix.shape[0]):
        for j in range(adjacency_matrix.shape[1]):
            if adjacency_matrix[i, j] != 0:
                G.add_edge(i, j, weight=adjacency_matrix[i, j])
    return G

def heuristic(node, goal, graph):
   
    return np.linalg.norm(np.array(graph.nodes[node]['pos']) - np.array(graph.nodes[goal]['pos']))

def a_star_algorithm(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while not frontier.empty():
        _, current = frontier.get()

        if current == goal:
            path = reconstruct_path(came_from, start, goal)
            return path

        for next_node in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.get_edge_data(current, next_node)['weight']
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(next_node, goal, graph)
                frontier.put((priority, next_node))
                came_from[next_node] = current

    return None

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []

    while current != start:
        path.append(current)
        current = came_from[current]

    path.append(start)
    path.reverse()

    return path

def Solution(file_path, source, destination, bandwidth_requirement):
 
    adjacency_matrix, bandwidth_matrix, delay_matrix, reliability_matrix = read_input_file(file_path)

   
    G = adjacency_matrix_to_graph(adjacency_matrix, create_using=nx.Graph)

   
    pos = {node: (0, 0) for node in G.nodes}
    nx.set_node_attributes(G, pos, 'pos')

    dijkstra_path = dijkstra_algorithm(G, source, destination)

    bellman_ford_path = bellman_ford_algorithm(G, source, destination)

    floyd_warshall_matrix = floyd_warshall_algorithm(G)

    if floyd_warshall_matrix[source, destination] < np.inf and \
            constraints_check(list(nx.shortest_path(G, source=source, target=destination, weight='weight')),
                              bandwidth_matrix, delay_matrix, reliability_matrix):
        print("Floyd-Warshall Algorithm Solution: ",
              list(nx.shortest_path(G, source=source, target=destination, weight='weight')))
    else:
        print("Floyd-Warshall Algorithm Solution does not satisfy constraints.")

   
    if dijkstra_path and constraints_check(dijkstra_path, bandwidth_matrix, delay_matrix, reliability_matrix):
        print("Dijkstra's Algorithm Solution: ", dijkstra_path)
    else:
        print("Dijkstra's Algorithm Solution does not satisfy constraints.")

  
    if bellman_ford_path and constraints_check(bellman_ford_path, bandwidth_matrix, delay_matrix, reliability_matrix):
        print("Bellman-Ford Algorithm Solution: ", bellman_ford_path)
    else:
        print("Bellman-Ford Algorithm Solution does not satisfy constraints.")


    a_star_path = a_star_algorithm(G, source, destination)


    if a_star_path and constraints_check(a_star_path, bandwidth_matrix, delay_matrix, reliability_matrix):
        print("A* Algorithm Solution: ", a_star_path)
    else:
        print("A* Algorithm Solution does not satisfy constraints.")

source_node_id = 1
destination_node_id = 3
bandwidth_requirement = 10
file_path = "pyton/project.txt"  


Solution(file_path, source_node_id, destination_node_id, bandwidth_requirement)
