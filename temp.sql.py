# %%
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
db = client['vic_db']

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



points_collection = db['points']
points_coords_mongo_get = []
collection_count = points_collection.estimated_document_count()
for i in tqdm(points_collection.find(), total=collection_count):
    points_coords_mongo_get.append(i)
points_coords = {i['_id']: i['coords'] for i in points_coords_mongo_get}


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
   


# neighbors : dict[int, list] = {}
# for i, road in tqdm(enumerate(roads), total=len(roads)):
#     if road['direction_code'] == 'B' or road['direction_code'] == 'F':
#         if road['from_ufi'] not in neighbors:
#             neighbors[int(road['from_ufi'])] = []
#         neighbors[int(road['from_ufi'])].append((int(road['to_ufi']), int(road['ufi']), road['road_length_meters']))
#     if road['direction_code'] == 'B' or road['direction_code'] == 'R':
#         if road['to_ufi'] not in neighbors:
#             neighbors[int(road['to_ufi'])] = []
#         neighbors[int(road['to_ufi'])].append((int(road['from_ufi']), int(road['ufi']), road['road_length_meters']))


db = client['vic_db']
collection = db['points_neighbours']
neighbors_mongo = []
collection_count = collection.estimated_document_count()
for i in tqdm(collection.find(), total=collection_count):
    neighbors_mongo.append(i)

neighbors = {n['_id']: n['neighbours'] for n in neighbors_mongo}



# %%

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
        print('iter', len(frontier))

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

# %%
from types import MappingProxyType
neighbors.keys().mapping
# What is mapping proxy?
# MappingProxyType is a wrapper around a standard dictionary that provides a read-only view into the wrapped dictionary’s data.
# 

# %%


# %%
# Find if vmtrans.tr_points has a primary key and what it is
sql = '''
SELECT
    c.column_name
FROM
    information_schema.table_constraints tc
    JOIN information_schema.constraint_column_usage AS ccu USING (constraint_schema, constraint_name)
    JOIN information_schema.columns AS c ON c.table_schema = tc.constraint_schema
    AND tc.table_name = c.table_name AND ccu.column_name = c.column_name
WHERE
    constraint_type = 'PRIMARY KEY'
    AND tc.table_name = 'tr_points'
    AND tc.table_schema = 'vmtrans';
'''
cursor.execute(sql)
cursor.fetchall()


# %%

# Set ufi as primary key of vmtrans.tr_points and generate a spatial index
sql = '''
ALTER TABLE vmtrans.tr_points
ADD PRIMARY KEY (ufi);
CREATE INDEX tr_points_geom_idx
ON vmtrans.tr_points
USING GIST (geom);
'''

cursor.execute(sql)


# %%
db = client['vic_db']
collection = db['points_neighbours']
neighbors_mongo = []
collection_count = collection.estimated_document_count()
for i in tqdm(collection.find(), total=collection_count):
    neighbors_mongo.append(i)
# 5s - 20s

# # sql = '''
# # -- Create a GIST spatial index on the lines dataset
# # CREATE INDEX geom_idx ON vmtrans.tr_road_all USING GIST (geom);
# # '''
# # cursor.execute(sql)



# # sql = '''
# # -- For each point, find the nearest line
# # SELECT points.id, lines.id, ST_Distance(points.geom, lines.geom) AS distance
# # FROM points, lines
# # WHERE ST_DWithin(points.geom, lines.geom, some_threshold)
# # ORDER BY points.id, ST_Distance(points.geom, lines.geom)
# # LIMIT 1;
# # '''

# find_nearest_road(144.9631, -37.8136, 1)

# %%


# %%
# Copy gtfs_4.stops to a new table, and generate geom column from stop_lat and stop_lon
sql = '''
DROP TABLE IF EXISTS vmtrans.stops;
CREATE TABLE vmtrans.stops AS
SELECT 
    stop_id, 
    stop_name, 
    stop_lat, 
    stop_lon, 
    ST_SetSRID(ST_MakePoint(stop_lon, stop_lat), 7844) AS geom
FROM gtfs_4.stops;
'''
cursor.execute(sql)
print(cursor.statusmessage)
cursor.execute('CREATE INDEX stops_geom_idx ON vmtrans.stops USING GIST (geom);')
print(cursor.statusmessage)

sql = '''
SELECT * FROM vmtrans.stops;
'''
cursor.execute(sql)
stops = cursor.fetchall()
stops_df = pd.DataFrame(stops, columns=[desc[0] for desc in cursor.description])
stops_gdf = gpd.read_postgis(sql, con=engine)
stops = stops_gdf.to_dict(orient='records')

# %%
nearest_roads = {}
for stop in tqdm(stops):
    stop_lat = stop['stop_lat']
    stop_lon = stop['stop_lon']
    stop_id = stop['stop_id']
    nearest_road = find_nearest_road(stop_lon, stop_lat, 1)
    nearest_roads[stop_id] = nearest_road
    # 12 hours for 19000 stops

# %%
import asyncio
from tqdm.asyncio import tqdm as atqdm
nearest_roads = {}
async def get_nearest_road(stop):
    stop_lat = stop['stop_lat']
    stop_lon = stop['stop_lon']
    stop_id = stop['stop_id']
    nearest_road = find_nearest_road(stop_lon, stop_lat, 1)
    nearest_roads[stop_id] = nearest_road

async def main():
    # Use atqdm instead of tqdm
    for stop in atqdm(stops):
        await get_nearest_road(stop)

await main()

# %%
# List all index created
sql = '''
SELECT * FROM pg_indexes;
'''
cursor.execute(sql)
indexes = cursor.fetchall()
indexes_df = pd.DataFrame(indexes, columns=[desc[0] for desc in cursor.description])
indexes_df

# %%
# -- Create a spatial index on the lines
# CREATE INDEX lines_geom_idx ON lines USING GIST (geom);

# -- Query to find the nearest line for each point
# SELECT points.id, lines.id, ST_Distance(points.geom, lines.geom) AS distance
# FROM points, lines
# ORDER BY points.id, ST_Distance(points.geom, lines.geom)
# LIMIT 1;




# For each stop in vmtrans.stops, find the nearest road and the nearest point on the road in vmtrans.tr_road_all
sql = '''
SELECT 
    stops.stop_id, 
    stops.stop_name, 
    stops.geom AS stop_geom, 
    roads.ufi, 
    roads.ezi_road_name_label, 
    roads.direction_code, 
    roads.geom AS road_geom, 
    ST_Distance(stops.geom, roads.geom) AS distance
FROM vmtrans.stops AS stops, vmtrans.tr_road_all AS roads
ORDER BY stops.stop_id, ST_Distance(stops.geom, roads.geom)
LIMIT 1;
'''
cursor.execute(sql)
nearest_road_stops = cursor.fetchall()


# %%
nearest_road_stops = pd.DataFrame(nearest_road_stops, columns=[desc[0] for desc in cursor.description])

# %%
import sys

ans = sys.getsizeof(neighbors) / 1024 / 1024
print(f"Size of neighbors: {ans} MB")

# %%
sys.getsizeof(neighbors)

# %%
db = client['vic_db']
collection = db['points_neighbours']

# %%
collection.find_one()

# %%

db = client['vic_db']
collection = db['points_neighbours']
for point, neighbours in tqdm(neighbors.items()):
    collection.insert_one({
        '_id': point,
        'point': points_coords[point],
        'neighbours': neighbours
    })
    # 11m 30s
neighbors[2309370]

# %%
len(roads)

# %%
geodesic((-37.9154782598288, 145.135624833031), (-37.8136, 144.9631)).meters
bus_stops_df = pd.read_sql_query(text("SELECT * FROM gtfs_4.stops;"), conn_alchemy)
bus_stops_info = bus_stops_df.set_index('stop_id').to_dict(orient='index')

# %%
path, cost, start_road_info, goal_road_info = search_path(145.135624833031, -37.9154782598288, 144.9631, -37.8136)
print(len(path), cost)
gdf = get_path_info(path)
fig, ax = plt.subplots()
gdf.plot(ax=ax, color='red')
plt.show()

# %%
# Set ufi as primary key
sql = '''
ALTER TABLE vmtrans.tr_road_all
DROP COLUMN gid,
ADD PRIMARY KEY (ufi);
'''
cursor.execute(sql)


# %%

# # Drop gid column
# sql = '''
# ALTER TABLE vmtrans.tr_road_all
# DROP COLUMN gid;
# '''

sql = '''
SELECT * FROM vmtrans.tr_road_all WHERE ufi = 1419037;
'''
cursor.execute(sql)
cursor.fetchall(), cursor.description


# %%


# %%

def find_nearest_roads(lon, lat, limit=1):
    """
    For a given point, for each road, find the nearest point on the road, and return the road with the shortest distance between that nearest point and the given point.
    """
    sql = f"""
    SELECT 
        ufi,
        ezi_road_name_label,
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

