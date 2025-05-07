import streamlit as st
import numpy as np
import networkx as nx
import random
import time
import plotly.graph_objects as go  # For 3D visualization

# Default Settings
MAP_SIZE = 20
OBSTACLE_COUNT = 30
WAYPOINT_COUNT = 3
ALGORITHMS = {"A*": nx.astar_path, "Dijkstra": nx.dijkstra_path}

# Generate random map
def generate_map(map_size, obstacle_count, waypoint_count):
    grid = np.zeros((map_size, map_size))
    
    # Place obstacles
    for _ in range(obstacle_count):
        x, y = random.randint(0, map_size - 1), random.randint(0, map_size - 1)
        grid[x][y] = -1  # Obstacle
    
    # Start and end points
    start, end = None, None
    while start == end or not start or not end or grid[start] == -1 or grid[end] == -1:
        start = (random.randint(0, map_size - 1), random.randint(0, map_size - 1))
        end = (random.randint(0, map_size - 1), random.randint(0, map_size - 1))
    
    # Waypoints
    waypoints = []
    for _ in range(waypoint_count):
        point = (random.randint(0, map_size - 1), random.randint(0, map_size - 1))
        while point in [start, end] or grid[point] == -1 or point in waypoints:
            point = (random.randint(0, map_size - 1), random.randint(0, map_size - 1))
        waypoints.append(point)
    
    return grid, start, end, waypoints

# Pathfinding function
def find_path(grid, start, end, algorithm):
    G = nx.grid_2d_graph(grid.shape[0], grid.shape[1])
    for (x, y) in zip(*np.where(grid == -1)):
        G.remove_node((x, y))
    
    try:
        return ALGORITHMS[algorithm](G, start, end)
    except nx.NetworkXNoPath:
        return []

# UI Elements
st.title("Drone Pathfinding Simulation")
st.sidebar.header("Settings")
map_size = st.sidebar.slider("Map Size", 10, 50, MAP_SIZE)
obstacle_count = st.sidebar.slider("Obstacle Density", 5, 100, OBSTACLE_COUNT)
waypoint_count = st.sidebar.slider("Waypoints", 0, 5, WAYPOINT_COUNT)
algorithm = st.sidebar.selectbox("Pathfinding Algorithm", list(ALGORITHMS.keys()))
speed = st.sidebar.slider("Animation Speed (seconds per step)", 0.1, 2.0, 0.5)
start_simulation = st.sidebar.button("Start Simulation")

if start_simulation:
    # Generate Map
    grid, start, end, waypoints = generate_map(map_size, obstacle_count, waypoint_count)
    st.session_state.map_data = (grid, start, end, waypoints)
    
    # Find paths for both algorithms
    all_points = [start] + waypoints + [end]
    paths = {algo: [] for algo in ALGORITHMS.keys()}
    for algo in ALGORITHMS.keys():
        path = []
        for i in range(len(all_points) - 1):
            segment = find_path(grid, all_points[i], all_points[i + 1], algo)
            if not segment:
                st.error(f"No path found for {algo}!")
                break
            path.extend(segment if not path else segment[1:])
        paths[algo] = path
    
    # 3D Visualization
    x_obs, y_obs = np.where(grid == -1)
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(x=x_obs, y=y_obs, z=[0]*len(x_obs), mode='markers', marker=dict(size=5, color='red'), name='Obstacles'))
    fig.add_trace(go.Scatter3d(x=[start[1]], y=[start[0]], z=[0], mode='markers', marker=dict(size=8, color='green'), name='Start'))
    fig.add_trace(go.Scatter3d(x=[end[1]], y=[end[0]], z=[0], mode='markers', marker=dict(size=8, color='blue'), name='End'))
    for point in waypoints:
        fig.add_trace(go.Scatter3d(x=[point[1]], y=[point[0]], z=[0], mode='markers', marker=dict(size=6, color='yellow'), name='Waypoint'))
    for algo, path in paths.items():
        if path:
            path_x, path_y = zip(*path)
            fig.add_trace(go.Scatter3d(x=path_y, y=path_x, z=[0]*len(path), mode='lines', line=dict(width=3, color='cyan' if algo == "A" else 'magenta'), name=f'Path - {algo}'))
    
    # Animation
    frames = []
    for i in range(max(len(paths["A*"]), len(paths["Dijkstra"]))):
        frame_data = []
        for algo, path in paths.items():
            if i < len(path):
                frame_data.append(go.Scatter3d(x=[path[i][1]], y=[path[i][0]], z=[0], mode='markers', marker=dict(size=8, color='orange' if algo == "A*" else 'purple'), name=f'Drone - {algo}'))
        frames.append(go.Frame(data=frame_data))
    fig.update(frames=frames, layout=dict(updatemenus=[dict(type="buttons", showactive=False, buttons=[dict(label="Play", method="animate", args=[None, dict(frame=dict(duration=speed*1000, redraw=True), fromcurrent=True)])])]))
    
    fig.update_layout(title="3D Drone Pathfinding Visualization", scene=dict(zaxis=dict(visible=False)))
    st.plotly_chart(fig)