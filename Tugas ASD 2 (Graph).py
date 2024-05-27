def read_graph_from_file(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
    
    N, M = map(int, lines[0].split())
    edges = []
    max_vertex = -1  # Menyimpan vertex terbesar yang ditemukan
    for i in range(1, M + 1):
        p1, p2, d = map(int, lines[i].split())
        edges.append((p1, p2, d))
        max_vertex = max(max_vertex, p1, p2)
    S = int(lines[M + 1])
    
    # Jika max_vertex lebih besar dari N-1, kita perlu memperbarui N
    if max_vertex >= N:
        N = max_vertex + 1
    
    return N, M, edges, S

N, M, edges, S = read_graph_from_file('D:/UGM COOIII/nugass nugass/Semester 2/Algoritma Struktur Data/data.txt')

import heapq

def dijkstra(N, edges, S):
    graph = [[] for _ in range(N)]
    for p1, p2, d in edges:
        graph[p1].append((p2, d))
        graph[p2].append((p1, d))  # Jika graf tidak berarah
    
    dist = [float('inf')] * N
    dist[S] = 0
    pq = [(0, S)]  # (jarak, vertex)
    
    while pq:
        current_dist, u = heapq.heappop(pq)
        
        if current_dist > dist[u]:
            continue
        
        for v, weight in graph[u]:
            distance = current_dist + weight
            if distance < dist[v]:
                dist[v] = distance
                heapq.heappush(pq, (distance, v))
    
    return dist, graph

distances, graph = dijkstra(N, edges, S)

# Menangani kasus dimana tidak ada vertex yang terhubung
if all(d == float('inf') for d in distances):
    print("Tidak ada vertex yang terhubung dengan S.")
else:
    max_distance = max(d for d in distances if d < float('inf'))
    farthest_vertex = distances.index(max_distance)
    print(f"Vertex dengan jarak terpendek tertinggi dari S: {farthest_vertex}, Jarak: {max_distance}")

def dfs(graph, current, target_distance, visited, current_distance):
    if current_distance == target_distance:
        return [current]
    if current_distance > target_distance:
        return None
    
    visited.add(current)
    for neighbor, weight in graph[current]:
        if neighbor not in visited:
            path = dfs(graph, neighbor, target_distance, visited, current_distance + weight)
            if path:
                return [current] + path
    visited.remove(current)
    return None

# Memanggil dfs hanya jika ada graph yang terhubung
if graph:
    path_2024 = dfs(graph, S, 2024, set(), 0)
    if path_2024:
        print(f"Ada jalur dengan jarak 2024 dari S: Path - {path_2024}")
    else:
        print("Tidak ada jalur dengan jarak 2024 dari S")

from collections import deque

def bfs(N, edges, S):
    graph = [[] for _ in range(N)]
    for p1, p2, d in edges:
        graph[p1].append(p2)
        graph[p2].append(p1)  # Jika graf tidak berarah
    
    dist = [-1] * N
    dist[S] = 0
    queue = deque([S])
    
    while queue:
        u = queue.popleft()
        for v in graph[u]:
            if dist[v] == -1:
                dist[v] = dist[u] + 1
                queue.append(v)
    
    return dist

distances_bfs = bfs(N, edges, S)
max_distance_bfs = max(distances_bfs)
farthest_vertices_bfs = [i for i, d in enumerate(distances_bfs) if d == max_distance_bfs]
print(f"Vertex terjauh dari S tanpa memperhatikan bobot: {farthest_vertices_bfs}, Jarak: {max_distance_bfs}")
