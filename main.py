import math
import heapq

nodes = {
    'Bole Medhanealem': (9.00044, 38.78641),
    'Kazanchis': (9.01593, 38.77122),
    'Meskel Square': (9.01028, 38.76111),
    'Arat Kilo': (9.033333, 38.761111),
    'Piassa': (9.033697, 38.754753),
    'Mexico Square': (9.009, 38.742),
    'Lideta': (9.00368, 38.72605),
    'Tor Hailoch': (9.0114241, 38.7228182),
    'Merkato': (9.025, 38.737),
    'Saris': (8.95325, 38.76362)
}

graph = {
    'Bole Medhanealem': ['Kazanchis', 'Saris'],
    'Kazanchis': ['Bole Medhanealem', 'Meskel Square', 'Arat Kilo'],
    'Meskel Square': ['Kazanchis', 'Mexico Square', 'Arat Kilo', 'Saris'],
    'Arat Kilo': ['Kazanchis', 'Meskel Square', 'Piassa', 'Merkato'],
    'Piassa': ['Arat Kilo', 'Merkato'],
    'Mexico Square': ['Meskel Square', 'Lideta'],
    'Lideta': ['Mexico Square', 'Tor Hailoch'],
    'Tor Hailoch': ['Lideta', 'Merkato'],
    'Merkato': ['Tor Hailoch', 'Piassa', 'Arat Kilo'],
    'Saris': ['Bole Medhanealem', 'Meskel Square']
}

def haversine(coord1, coord2):
    R = 6371.0
    lat1 = math.radians(coord1[0])
    lon1 = math.radians(coord1[1])
    lat2 = math.radians(coord2[0])
    lon2 = math.radians(coord2[1])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

weights = {}
for node in graph:
    for neighbor in graph[node]:
        key = tuple(sorted([node, neighbor]))
        if key not in weights:
            dist = haversine(nodes[node], nodes[neighbor])
            weights[key] = dist

def get_weight(u, v):
    key = tuple(sorted([u, v]))
    return weights[key]

def a_star(start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    expanded = 0
    while frontier:
        _, current = heapq.heappop(frontier)
        expanded += 1
        if current == goal:
            break
        for next_node in graph[current]:
            new_cost = cost_so_far[current] + get_weight(current, next_node)
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + haversine(nodes[next_node], nodes[goal])
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from.get(current)
    path.reverse()
    total_cost = cost_so_far.get(goal, 0)
    return path, total_cost, expanded

# Example usage
start = 'Bole Medhanealem'
goal = 'Piassa'
path, cost, expanded = a_star(start, goal)
print('Path:', ' -> '.join(path))
print('Total Cost (km):', round(cost, 2))
print('Nodes Expanded:', expanded)