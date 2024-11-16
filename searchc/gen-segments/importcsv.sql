-- using schema name: vmtrans
SET search_path TO vmtrans;
DROP TABLE IF EXISTS segments;
CREATE TABLE segments (
    roadufi INTEGER,
    x1 DECIMAL,
    y1 DECIMAL,
    x2 DECIMAL,
    y2 DECIMAL
);

DROP TABLE IF EXISTS boundary;
CREATE TABLE boundary (
    x_min DECIMAL,
    y_min DECIMAL,
    x_max DECIMAL,
    y_max DECIMAL
);

\COPY segments FROM 'segments.csv' DELIMITER ',' CSV HEADER;
-- COPY segments FROM '/workspaces/vicpathfinding/gen-segments/segments.csv' DELIMITER ',' CSV HEADER;

\COPY boundary FROM 'boundary.csv' DELIMITER ',' CSV HEADER;
-- COPY boundary FROM '/workspaces/vicpathfinding/gen-segments/boundary.csv' DELIMITER ',' CSV HEADER;

-- "dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432
-- Run this SQL using the following command:
-- export PGPASSWORD=postgres && psql -h host.docker.internal -U postgres -d vic_db -f /workspaces/vicpathfinding/gen-segments/importcsv.sql
-- export PGPASSWORD=postgres && psql -U postgres -h localhost -p 5432 -d vic_db -f importcsv.sql


-- On Windows:
-- $PGPASSWORD = "postgres"
-- $env:PGPASSWORD = $PGPASSWORD
-- psql -U postgres -h localhost -p 5432 -d vic_db -f importcsv.sql
