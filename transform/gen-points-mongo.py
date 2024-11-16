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

points_coords = pd.read_sql_query(text("SELECT ufi, ST_X(geom::geometry) AS x, ST_Y(geom::geometry) AS y FROM vmtrans.tr_points;"), conn_alchemy)
points_coords.set_index('ufi', inplace=True)
points_coords = points_coords.to_dict(orient='index')
points_coords = {ufi: (coords['x'], coords['y']) for ufi, coords in points_coords.items()}
# 6s - 10s

collection = db['points']
collection.drop()
points_coords_mongo = [
    {
        '_id': ufi,
        'coords': coords,
    }
    for ufi, coords in points_coords.items()
]
collection.insert_many(points_coords_mongo) 
# 15s - 20s
