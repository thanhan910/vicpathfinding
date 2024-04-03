import matplotlib.pyplot as plt
import psycopg2
import pandas as pd
import geopandas as gpd
import numpy as np
import json
from sqlalchemy import create_engine, text
from tqdm import tqdm
from geopy.distance import geodesic


# Define database connection parameters
database_connection = {
    'drivername': 'postgresql',
    'username': 'postgres',
    'password': 'postgres',
    'host': 'localhost',
    'port': '5432',
    'database': 'vic_db',
}


# A psycopg2 connection and cursor
conn = psycopg2.connect(user=database_connection['username'],
                        password=database_connection['password'],
                        host=database_connection['host'],
                        port=database_connection['port'],
                        database=database_connection['database'])
conn.autocommit = True
cursor = conn.cursor()

# Create a SQLAlchemy engine
engine = create_engine('postgresql://%(username)s:%(password)s@%(host)s/%(database)s' % database_connection, isolation_level="AUTOCOMMIT")
conn_alchemy = engine.connect()



# Implement A* algorithm to find the shortest path between two points

# def astar(start, goal):
#     frontier = []
#     visited = set()
#     path = []
#     frontier.append((0, start, []))
#     while frontier:
#         cost, current, path = frontier.pop(0)
#         if current == goal:
#             return path, cost
#         if current in visited:
#             continue
#         visited.add(current)
#         for neighbor_point, road_ufi, road_length in neighbors(current):
#             # frontier.append((cost + edge_cost(current, neighbor), neighbor, path + [neighbor]))
#             if neighbor_point not in visited:
#                 frontier.append((cost + road_length, neighbor_point, path + [road_ufi]))
#         frontier.sort(key=lambda x: x[0] + heuristic(x[1], goal))

# def neighbors(current):
#     sql = f"""
#     SELECT to_ufi, ufi, road_length_meters
#     FROM vmtrans.tr_road_all
#     WHERE from_ufi = {current}
#     AND (direction_code = 'B' OR direction_code = 'F')
#     """
#     cursor.execute(sql)
#     neighbors1 = cursor.fetchall()
#     sql = f"""
#     SELECT from_ufi, ufi, road_length_meters
#     FROM vmtrans.tr_road_all
#     WHERE to_ufi = {current}
#     AND (direction_code = 'B' OR direction_code = 'R')
#     """
#     cursor.execute(sql)
#     neighbors2 = cursor.fetchall()
#     return [(neighbor[0], neighbor[1], neighbor[2]) for neighbor in neighbors1 + neighbors2]


# def heuristic(current, goal):
#     sql = f"""
#     SELECT ST_Distance(
#         (SELECT geom FROM vmtrans.tr_points
#         WHERE ufi = {current}),
#         (SELECT geom FROM vmtrans.tr_points
#         WHERE ufi = {goal})
#     );
#     """
#     cursor.execute(sql)
#     return cursor.fetchall()[0][0]




points_coords = pd.read_sql_query(text("SELECT ufi, ST_X(geom::geometry) AS x, ST_Y(geom::geometry) AS y FROM vmtrans.tr_points;"), conn_alchemy)
points_coords.set_index('ufi', inplace=True)
points_coords = points_coords.to_dict(orient='index')
points_coords = {ufi: (coords['x'], coords['y']) for ufi, coords in points_coords.items()}
# 9s - 50s


# sql = "SELECT ufi, ezi_road_name_label, direction_code, from_ufi, to_ufi, road_length_meters, geom FROM vmtrans.tr_road_all;"
# roads_gdf = gpd.read_postgis(sql, con=engine)
roads_df = pd.read_sql_query(text("SELECT ufi, ezi_road_name_label, direction_code, from_ufi, to_ufi, road_length_meters FROM vmtrans.tr_road_all;"), conn_alchemy)
# 10s - 45s - 1m


# Convert roads into a list of tuples
roads = roads_df.to_dict(orient='records')


neighbors = {}
with tqdm(total=len(roads)) as pbar:
    for i, road in enumerate(roads):
        if road['direction_code'] == 'B' or road['direction_code'] == 'F':
            if road['from_ufi'] not in neighbors:
                neighbors[road['from_ufi']] = []
            neighbors[road['from_ufi']].append((road['to_ufi'], road['ufi'], road['road_length_meters']))
        if road['direction_code'] == 'B' or road['direction_code'] == 'R':
            if road['to_ufi'] not in neighbors:
                neighbors[road['to_ufi']] = []
            neighbors[road['to_ufi']].append((road['from_ufi'], road['ufi'], road['road_length_meters']))
        pbar.update(1)




def heuristic(current, goal):
    # Get distance in real world from coordinates
    # Convert to meters
    # Calculate geodesic distance between two points in meters
    lon1, lat1 = points_coords[current]
    lon2, lat2 = points_coords[goal]
    distance = geodesic((lat1, lon1), (lat2, lon2)).meters
    return distance

def astar(start, goal):
    frontier = []
    visited = set()
    path = []
    frontier.append((0, 0, start, []))
    while frontier:
        _, cost, current, path = frontier.pop(0)
        # print(cost, current)
        if current == goal:
            return path, cost
        if current in visited:
            continue
        visited.add(current)
        for neighbor_point, road_ufi, road_length in neighbors.get(current, []):
            # frontier.append((cost + edge_cost(current, neighbor), neighbor, path + [neighbor]))
            if neighbor_point not in visited:
                heuristic_cost = heuristic(neighbor_point, goal)
                frontier.append((cost + road_length + heuristic_cost, cost + road_length, neighbor_point, path + [road_ufi]))
        frontier.sort(key=lambda x: x[0])


# Pick a random start and goal
distance = 0
while distance <= 400000:
    start = np.random.choice(list(points_coords.keys()))
    goal = np.random.choice(list(points_coords.keys()))
    distance = heuristic(start, goal)
start, goal, heuristic(start, goal), points_coords[start], points_coords[goal]


path, cost = astar(start, goal)
# 3m


sql = f"""
SELECT ufi, ezi_road_name_label, direction_code, road_length_meters, geom
FROM vmtrans.tr_road_all
WHERE ufi IN ({','.join([str(int(ufi)) for ufi in path])});
"""
gdf = gpd.read_postgis(sql, con=engine)
gdf['geometry'] = gdf['geom'].apply(lambda x: x.geoms[0])
gdf.drop(columns=['geom'], inplace=True)
gdf = gpd.GeoDataFrame(gdf, crs='EPSG:7844', geometry='geometry')


# Plot the path

fig, ax = plt.subplots()
gdf.plot(ax=ax, color='red')
plt.show()


