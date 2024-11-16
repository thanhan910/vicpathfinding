from datetime import datetime
import matplotlib.pyplot as plt
import psycopg2
import pandas as pd
import geopandas as gpd
import numpy as np
import json
from sqlalchemy import create_engine, text
from tqdm import tqdm
from geopy.distance import geodesic
from pymongo import MongoClient
from shapely.geometry import Point, LineString

client = MongoClient('localhost', 27017)


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

# db = client['vic_db']
# collection = db['roads']
# for road in tqdm(roads):
#     collection.insert_one({'_id': road['ufi'], **road})
   


neighbors : dict[int, list] = {}
for i, road in tqdm(enumerate(roads), total=len(roads)):
    if road['direction_code'] == 'B' or road['direction_code'] == 'F':
        if road['from_ufi'] not in neighbors:
            neighbors[int(road['from_ufi'])] = []
        neighbors[int(road['from_ufi'])].append((int(road['to_ufi']), int(road['ufi']), road['road_length_meters']))
    if road['direction_code'] == 'B' or road['direction_code'] == 'R':
        if road['to_ufi'] not in neighbors:
            neighbors[int(road['to_ufi'])] = []
        neighbors[int(road['to_ufi'])].append((int(road['from_ufi']), int(road['ufi']), road['road_length_meters']))






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

def find_nearest_road(lon, lat, limit=1):
    """
    For a given point, for each road, find the nearest point on the road, and return the road with the shortest distance between that nearest point and the given point.
    """
    sql = f"""
    SELECT 
        ufi,
        direction_code,
        ST_X(ST_ClosestPoint(geom::geometry, ST_SetSRID(ST_MakePoint({lon}, {lat}), 7844))) AS closest_point_x, 
        ST_Y(ST_ClosestPoint(geom::geometry, ST_SetSRID(ST_MakePoint({lon}, {lat}), 7844))) AS closest_point_y, 
        from_ufi,
        to_ufi
    FROM vmtrans.tr_road_all
    WHERE direction_code IS NOT NULL
    ORDER BY ST_Distance(ST_ClosestPoint(geom::geometry, ST_SetSRID(ST_MakePoint({lon}, {lat}), 7844)), ST_SetSRID(ST_MakePoint({lon}, {lat}), 7844))
    LIMIT {limit};
    """
    cursor.execute(sql)
    return cursor.fetchall()

def search_path(lon1, lat1, lon2, lat2):
    start_road_info = find_nearest_road(lon1, lat1)[0]
    print(start_road_info)
    goal_road_info = find_nearest_road(lon2, lat2)[0]
    print(goal_road_info)
    start_road_ufi, start_road_direction, start_road_px, start_road_py, start_from_ufi, start_to_ufi = start_road_info
    goal_road_ufi, goal_road_direction, goal_road_px, goal_road_py, goal_from_ufi, goal_to_ufi = goal_road_info

    if start_road_ufi == goal_road_ufi:
        return [start_road_ufi], 0, start_road_info, goal_road_info
    
    special_neighbors : dict[int, list] = {}

    points_coords[0] = (start_road_px, start_road_py)
    points_coords[1] = (goal_road_px, goal_road_py)

    start_from_ufi_distance = geodesic((start_road_py, start_road_px), (points_coords[start_from_ufi][1], points_coords[start_from_ufi][0])).meters
    start_to_ufi_distance = geodesic((start_road_py, start_road_px), (points_coords[start_to_ufi][1], points_coords[start_to_ufi][0])).meters
    goal_from_ufi_distance = geodesic((goal_road_py, goal_road_px), (points_coords[goal_from_ufi][1], points_coords[goal_from_ufi][0])).meters
    goal_to_ufi_distance = geodesic((goal_road_py, goal_road_px), (points_coords[goal_to_ufi][1], points_coords[goal_to_ufi][0])).meters

    special_neighbors[start_from_ufi] = []
    special_neighbors[start_to_ufi] = []
    special_neighbors[goal_from_ufi] = []
    special_neighbors[goal_to_ufi] = []
    special_neighbors[0] = []
    special_neighbors[1] = []

    if start_road_direction == 'F' or start_road_direction == 'B':
        special_neighbors[start_from_ufi].append((0, start_road_ufi, start_from_ufi_distance))
        special_neighbors[0].append((start_to_ufi, start_road_ufi, start_to_ufi_distance))
    if start_road_direction == 'R' or start_road_direction == 'B':
        special_neighbors[start_to_ufi].append((0, start_road_ufi, start_to_ufi_distance))
        special_neighbors[0].append((start_from_ufi, start_road_ufi, start_from_ufi_distance))

    if goal_road_direction == 'F' or goal_road_direction == 'B':
        special_neighbors[goal_from_ufi].append((1, goal_road_ufi, goal_from_ufi_distance))
        special_neighbors[1].append((goal_to_ufi, goal_road_ufi, goal_to_ufi_distance))
    if goal_road_direction == 'R' or goal_road_direction == 'B':
        special_neighbors[goal_to_ufi].append((1, goal_road_ufi, goal_to_ufi_distance))
        special_neighbors[1].append((goal_from_ufi, goal_road_ufi, goal_from_ufi_distance))

    skip_neighbors = {
        start_from_ufi: start_to_ufi,
        start_to_ufi: start_from_ufi,
        goal_from_ufi: goal_to_ufi,
        goal_to_ufi: goal_from_ufi,
    }
    
    frontier = []
    visited = set()
    path = []
    frontier.append((0, 0, 0, []))
    while frontier:
        hcost, cost, current, path = frontier.pop(0)
        if current == 1:
            return path, cost, start_road_info, goal_road_info
        if current in visited:
            continue
        visited.add(current)
        neighbor_points = neighbors.get(current, []) + special_neighbors.get(current, [])
        for neighbor_point, road_ufi, road_length in neighbor_points:
            # frontier.append((cost + edge_cost(current, neighbor), neighbor, path + [neighbor]))
            if current in skip_neighbors and skip_neighbors[current] == neighbor_point:
                continue
            if neighbor_point not in visited:
                heuristic_cost = geodesic((goal_road_py, goal_road_px), (points_coords[neighbor_point][1], points_coords[neighbor_point][0])).meters
                frontier.append((cost + road_length + heuristic_cost, cost + road_length, neighbor_point, path + [road_ufi]))
        frontier.sort(key=lambda x: x[0])

def get_path_info(path):
    sql = f"""
    SELECT ufi, ezi_road_name_label, direction_code, road_length_meters, geom
    FROM vmtrans.tr_road_all
    WHERE ufi IN ({','.join([str(int(ufi)) for ufi in path])});
    """
    gdf = gpd.read_postgis(sql, con=engine)
    gdf['geometry'] = gdf['geom'].apply(lambda x: x.geoms[0])
    gdf.drop(columns=['geom'], inplace=True)
    gdf = gpd.GeoDataFrame(gdf, crs='EPSG:7844', geometry='geometry')
    roads_info = gdf.set_index('ufi').to_dict(orient='index')
    gdf = gpd.GeoDataFrame(
    [{
        'ufi': ufi,
        'ezi_road_name_label': roads_info[ufi]['ezi_road_name_label'],
        'direction_code': roads_info[ufi]['direction_code'],
        'road_length_meters': roads_info[ufi]['road_length_meters'],
        'geometry': roads_info[ufi]['geometry']
    } for ufi in path], crs='EPSG:7844', geometry='geometry')
    return gdf

# Total time: 1m 30s

db = client['vic_db']
db.drop_collection('points_neighbours')
collection = db['points_neighbours']
points_neighbours = [{'_id': point, 'point': points_coords[point], 'neighbours': neighbours} for point, neighbours in neighbors.items()]
collection.insert_many(points_neighbours)