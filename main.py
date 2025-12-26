import math
import heapq
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt

# --------------------------------
# Nodes and Graph
# --------------------------------
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
    'Arat Kilo': ['Kazanchis', 'Meskel Square', 'Piassa'],
    'Piassa': ['Arat Kilo', 'Merkato'],
    'Mexico Square': ['Meskel Square', 'Lideta'],
    'Lideta': ['Mexico Square', 'Tor Hailoch'],
    'Tor Hailoch': ['Lideta', 'Merkato'],
    'Merkato': ['Tor Hailoch', 'Piassa', 'Arat Kilo'],
    'Saris': ['Bole Medhanealem', 'Meskel Square']
}

# --------------------------------
# Distance Function
# --------------------------------
def haversine(c1, c2):
    R = 6371
    lat1, lon1 = map(math.radians, c1)
    lat2, lon2 = map(math.radians, c2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# Precompute distances
weights = {}
for u in graph:
    for v in graph[u]:
        key = tuple(sorted((u, v)))
        if key not in weights:
            weights[key] = haversine(nodes[u], nodes[v])

def get_distance(u, v):
    return weights[tuple(sorted((u, v)))]

# --------------------------------
# A* Algorithm (Time-Based)
# --------------------------------
def a_star(start, goal, speed):
    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    expanded = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        expanded += 1

        if current == goal:
            break

        for neighbor in graph[current]:
            travel_time = get_distance(current, neighbor) / speed
            new_cost = cost_so_far[current] + travel_time

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                heuristic = haversine(nodes[neighbor], nodes[goal]) / speed
                priority = new_cost + heuristic
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from[node]
    path.reverse()

    return path, cost_so_far[goal] * 60, expanded

# --------------------------------
# Draw Optimal Path
# --------------------------------
def draw_path(path):
    plt.figure(figsize=(8, 6))

    # Plot all nodes
    for node, (lat, lon) in nodes.items():
        plt.plot(lon, lat, 'bo')
        plt.text(lon + 0.001, lat + 0.001, node, fontsize=9)

    # Plot all edges in light gray
    for u in graph:
        for v in graph[u]:
            lat1, lon1 = nodes[u]
            lat2, lon2 = nodes[v]
            plt.plot([lon1, lon2], [lat1, lat2], 'lightgray', zorder=1)

    # Highlight the optimal path in red
    for i in range(len(path) - 1):
        lat1, lon1 = nodes[path[i]]
        lat2, lon2 = nodes[path[i + 1]]
        plt.plot([lon1, lon2], [lat1, lat2], 'r', linewidth=2.5, zorder=2)

    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Optimal Path (A* Search)")
    plt.grid(True)
    plt.show()

# --------------------------------
# UI Logic
# --------------------------------
def find_route():
    start = start_var.get()
    goal = goal_var.get()

    try:
        speed = float(speed_var.get())
        if speed <= 0:
            raise ValueError
    except:
        messagebox.showerror("Invalid Input", "Speed must be a positive number")
        return

    if start == goal:
        messagebox.showwarning("Warning", "Start and Goal must be different")
        return

    path, time, expanded = a_star(start, goal, speed)

    result_box.config(state="normal")
    result_box.delete("1.0", tk.END)
    result_box.insert(tk.END,
        f"🚦 Optimal Route Found (A* Search)\n\n"
        f"Start: {start}\n"
        f"Goal: {goal}\n\n"
        f"Path:\n{'  →  '.join(path)}\n\n"
        f"Total Travel Time: {time:.2f} minutes\n"
        f"Nodes Expanded: {expanded}\n"
    )
    result_box.config(state="disabled")

    # Draw the optimal path graph
    draw_path(path)

# --------------------------------
# UI Layout
# --------------------------------
root = tk.Tk()
root.title("A* Route Finder – Time Optimized")
root.geometry("550x500")

title = ttk.Label(root, text="A* Route Finding System", font=("Arial", 16, "bold"))
title.pack(pady=5)

subtitle = ttk.Label(
    root,
    text="Find the fastest route between locations using A* Search Algorithm",
    font=("Arial", 10)
)
subtitle.pack(pady=2)

frame = ttk.Frame(root)
frame.pack(pady=10)

ttk.Label(frame, text="Start Location").grid(row=0, column=0, padx=10, pady=5)
start_var = tk.StringVar(value=list(nodes.keys())[0])
ttk.Combobox(frame, textvariable=start_var, values=list(nodes.keys()), width=25).grid(row=0, column=1)

ttk.Label(frame, text="Goal Location").grid(row=1, column=0, padx=10, pady=5)
goal_var = tk.StringVar(value=list(nodes.keys())[1])
ttk.Combobox(frame, textvariable=goal_var, values=list(nodes.keys()), width=25).grid(row=1, column=1)

ttk.Label(frame, text="Average Speed (km/h)").grid(row=2, column=0, padx=10, pady=5)
speed_var = tk.StringVar(value="30")
ttk.Entry(frame, textvariable=speed_var, width=10).grid(row=2, column=1, sticky="w")

ttk.Button(root, text="Find Best Route", command=find_route).pack(pady=10)

result_box = tk.Text(root, height=10, width=65, state="disabled", wrap="word")
result_box.pack(pady=10)

root.mainloop()
