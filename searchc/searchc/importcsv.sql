-- using schema name: vmtrans
SET search_path TO vmtrans;

DROP TABLE IF EXISTS quadtrees;
CREATE TABLE quadtrees (
    quadid TEXT,
    x_min DECIMAL,
    y_min DECIMAL,
    x_max DECIMAL,
    y_max DECIMAL,
    x_mid DECIMAL,
    y_mid DECIMAL,
    divided BOOLEAN,
    segments_count INTEGER,
    quad0 INTEGER,
    quad1 INTEGER,
    quad2 INTEGER,
    quad3 INTEGER,
    PRIMARY KEY (quadid)
);

DROP TABLE IF EXISTS quadsegments;
CREATE TABLE quadsegments (
    quadid TEXT,
    roadufi INTEGER,
    x1 DECIMAL,
    y1 DECIMAL,
    x2 DECIMAL,
    y2 DECIMAL,
    FOREIGN KEY (quadid) REFERENCES quadtrees (quadid)
);

\COPY quadtrees FROM '../local/quadtrees.csv' DELIMITER ',' CSV HEADER;
-- COPY segments FROM '/workspaces/vicpathfinding/gen-segments/segments.csv' DELIMITER ',' CSV HEADER;

\COPY quadsegments FROM '../local/quadsegments.csv' DELIMITER ',' CSV HEADER;
-- COPY boundary FROM '/workspaces/vicpathfinding/gen-segments/boundary.csv' DELIMITER ',' CSV HEADER;

-- ALTER TABLE quadsegments ADD FOREIGN KEY (quadid) REFERENCES quadtrees (quadid);

-- "dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432

/*
-- Run this SQL using the following command:
export PGPASSWORD=postgres && psql -h host.docker.internal -U postgres -d vic_db -f /workspaces/vicpathfinding/gen-segments/importcsv.sql
export PGPASSWORD=postgres && psql -U postgres -h localhost -p 5432 -d vic_db -f importcsv.sql


-- On Windows:

$PGPASSWORD = "postgres"
$env:PGPASSWORD = $PGPASSWORD
psql -U postgres -h localhost -p 5432 -d vic_db -f importcsv.sql


-- Select all order by text length
SELECT * FROM vmtrans.quadtrees
ORDER BY length(quadid)
LIMIT 100

*/