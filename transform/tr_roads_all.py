import psycopg2
from sqlalchemy import create_engine, text


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

# Add new columns to the table
sql = """
ALTER TABLE vmtrans.tr_road_all
ADD COLUMN road_length_meters double precision,
ADD COLUMN from_point geometry(Point, 7844),
ADD COLUMN to_point geometry(Point, 7844);

UPDATE vmtrans.tr_road_all
SET road_length_meters = ST_Length(geom::geography),
    from_point = ST_StartPoint(ST_GeometryN(geom, 1)),
    to_point = ST_EndPoint(ST_GeometryN(geom, 1));
"""
cursor.execute(sql)
# 1m 30s


# vmtrans.tr_road_all has columns from_ufi, to_ufi, from_point, to_point
# Create a new table from vmtrans.tr_road_all, with 2 columns: ufi, point
# The new table should have 2 rows for each row in vmtrans.tr_road_all, one for from_ufi and from_point, one for to_ufi and to_point
sql = """
CREATE TABLE vmtrans.tr_points AS
SELECT from_ufi AS ufi, from_point AS geom FROM vmtrans.tr_road_all
UNION ALL
SELECT to_ufi AS ufi, to_point AS geom FROM vmtrans.tr_road_all;
"""
cursor.execute(sql)
# 17s - 30s

# Check that each ufi has only one point
sql = """
SELECT ufi, COUNT(DISTINCT geom) AS count
FROM vmtrans.tr_points
GROUP BY ufi
HAVING COUNT(DISTINCT geom) > 1;
"""
cursor.execute(sql)
result = cursor.fetchall()
assert len(result) == 0

# Remove duplicate ufi-geom pairs
sql = """
CREATE TABLE vmtrans.tr_points_clean AS
SELECT ufi, geom
FROM vmtrans.tr_points
GROUP BY ufi, geom;
"""
cursor.execute(sql)

# Rename the table from tr_points_clean to tr_points
sql = """
DROP TABLE IF EXISTS vmtrans.tr_points;
ALTER TABLE vmtrans.tr_points_clean
RENAME TO tr_points;
"""
cursor.execute(sql)

# Assert that ufi is unique
cursor.execute("SELECT COUNT(ufi), COUNT(DISTINCT ufi) FROM vmtrans.tr_points;")
result = cursor.fetchall()
assert result[0][0] == result[0][1]


# Check that all ezi_road_name are just ezi_road_name_label in uppercase, with ' - ' replaced by '-'
sql = """
SELECT ezi_road_name, ezi_road_name_label
FROM vmtrans.tr_road_all
WHERE REPLACE(UPPER(ezi_road_name_label), ' - ', '-') != ezi_road_name;
"""
cursor.execute(sql)
result = cursor.fetchall()
assert len(result) == 0