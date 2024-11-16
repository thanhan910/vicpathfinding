#include <chrono>
#include <iostream>
#include <pqxx/pqxx>
#include <string>
#include <vector>
#include <sstream>

// MULTILINESTRING((144.10242311600007 -38.455164802999946,144.10346247500001 -38.45479963199995)
std::chrono::steady_clock::time_point begin, end;

pqxx::connection conn("dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432");

// Function to parse a WKT MULTILINESTRING into vector of vector of (x, y) pairs
std::vector<std::vector<std::pair<double, double>>> parseWKTtoMultiLineString(const std::string &wkt)
{
    std::vector<std::vector<std::pair<double, double>>> multiLineString;

    // Remove the "MULTILINESTRING(" and ")" from the WKT string
    std::string stripped_wkt = wkt.substr(16, wkt.size() - 17);
    std::istringstream lines_stream(stripped_wkt);
    std::string line;

    while (std::getline(lines_stream, line, ')'))
    {
        // Skip any commas at the beginning of each line string segment
        if (!line.empty() && line[0] == ',')
            line = line.substr(2);

        std::vector<std::pair<double, double>> lineString;
        std::istringstream points_stream(line.substr(1)); // Remove the leading '('

        std::string point;
        while (std::getline(points_stream, point, ','))
        {
            std::istringstream point_stream(point);
            // double x, y;
            // x and y are 16 characters long
            double x, y;
            point_stream >> x >> y; // Read x and y coordinates
            lineString.emplace_back(x, y);
        }

        multiLineString.push_back(lineString);
    }
    return multiLineString;
}

using CoordinateValue = double;
using CoordinatePair = std::pair<CoordinateValue, CoordinateValue>;

std::vector<CoordinatePair> parseGeomPoints(const pqxx::field &geom_points_sql)
{
    pqxx::array<std::string> geom_points_sql_array = geom_points_sql.as_sql_array<std::string>();
    std::vector<CoordinatePair> geom_points;
    for (size_t i = 0; i < geom_points_sql_array.size(); i++)
    {
        std::string point = geom_points_sql_array[i];
        std::istringstream iss(point);
        double x, y;
        iss >> x >> y;
        geom_points.emplace_back(x, y);
    }
    return geom_points;
}

// Structure to store each row's data
struct RoadData
{
    int ufi;
    std::vector<CoordinatePair> geom_points;
};

std::size_t get_results()
{
    pqxx::work txn(conn);

/*
-- This query gets the ufi and raw geom as a string
-- This query takes about 4s - 5s to run
SELECT 
	ufi,
	ST_AsText(geom) AS geom_points
FROM 
	vmtrans.tr_road_all
WHERE 
	direction_code IS NOT NULL;
*/

/*
-- This query further removes the MULTILINESTRING(( )) wrapper from the geom string
-- This query takes about 8s - 15s to run
SELECT 
    ufi,
    REGEXP_REPLACE(ST_AsText(geom), '^MULTILINESTRING\(\(|\)\)$', '', 'g') AS geom_points
FROM 
    vmtrans.tr_road_all
WHERE 
    direction_code IS NOT NULL;
*/

/*
-- This query further converts the geom string to a string array
-- This query takes about 10s - 15s to run
SELECT 
    ufi,
    STRING_TO_ARRAY(
        REGEXP_REPLACE(ST_AsText(geom), '^MULTILINESTRING\(\(|\)\)$', '', 'g'),
        ','
    ) AS geom_points
FROM 
    vmtrans.tr_road_all
WHERE 
    direction_code IS NOT NULL;
*/

/*
-- This query further splits each string in the array into an array of lon/lat pairs
-- This query takes about 30s - 40s - 1 minute to run
SELECT 
    ufi,
    ARRAY(
        SELECT 
            string_to_array(point_str, ' ')  -- Split into lon/lat and cast to float array
        FROM 
            unnest(
                STRING_TO_ARRAY(
                    REGEXP_REPLACE(ST_AsText(geom), '^MULTILINESTRING\(\(|\)\)$', '', 'g'), 
                    ','
                )
            ) AS point_str
    ) AS geom_points_split
FROM 
    vmtrans.tr_road_all
WHERE 
    direction_code IS NOT NULL;
*/

/*
-- This query further splits the array of lon/lat pairs into segments
-- This query took more than 1 hour in pgAdmin 4 and still not finished
SELECT 
    ufi,
    ARRAY[
        geom_points_split[i], 
        geom_points_split[i + 1]
    ] AS segment
FROM 
    vmtrans.tr_road_all,
    LATERAL (
        SELECT 
            ARRAY(
                SELECT 
                    string_to_array(point_str, ' ')::float[]  -- Split into lon/lat and cast to float array
                FROM 
                    unnest(
                        STRING_TO_ARRAY(
                            REGEXP_REPLACE(ST_AsText(geom), '^MULTILINESTRING\(\(|\)\)$', '', 'g'), 
                            ','
                        )
                    ) AS point_str
            ) AS geom_points_split
    ) AS points_array,
    LATERAL generate_subscripts(points_array.geom_points_split, 1) AS i
WHERE 
    direction_code IS NOT NULL
    AND i < array_length(points_array.geom_points_split, 1);  -- Ensure we don't go out of bounds
*/


    std::string query = R"(
SELECT 
    ufi,
    STRING_TO_ARRAY(
        REGEXP_REPLACE(ST_AsText(geom), '^MULTILINESTRING\(\(|\)\)$', '', 'g'),
        ','
    ) AS geom_points
FROM 
    vmtrans.tr_road_all
WHERE 
    direction_code IS NOT NULL;
    )";

    begin = std::chrono::steady_clock::now();
    pqxx::result result = txn.exec(query);
    end = std::chrono::steady_clock::now();
    std::cout << "SQL get tr_road_all: Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    // Vector to store all rows
    std::vector<RoadData> roads;

    // Process each row in the result
    for (const auto &row : result)
    {
        RoadData data;
        data.ufi = row["ufi"].as<int>();

        // Parse the geometry points
        data.geom_points = parseGeomPoints(row["geom_points"]);
        
        roads.push_back(data);
    }

    end = std::chrono::steady_clock::now();
    std::cout << "Parse SQL result: Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return roads.size();
}

int main()
{
    std::size_t count = get_results();
    std::cout << "Count: " << count << std::endl;

    return 0;
}


// SQL get tr_road_all: Time difference = 14578[ms]
// Parse SQL result: Time difference = 54075[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 15892[ms]
// Parse SQL result: Time difference = 69554[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 31365[ms]
// Parse SQL result: Time difference = 102060[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 31509[ms]
// Parse SQL result: Time difference = 64954[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 38035[ms]
// Parse SQL result: Time difference = 96299[ms]
// Count: 1234693


// When only ufi and geom_points are selected, and using number-based index for row[0] and row[1]:

// SQL get tr_road_all: Time difference = 31092[ms]
// Parse SQL result: Time difference = 98560[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 30125[ms]
// Parse SQL result: Time difference = 79909[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 30153[ms]
// Parse SQL result: Time difference = 76110[ms]
// Count: 1234693


// When only ufi and geom_points are selected, and using string-based index for row["ufi"] and row["geom_points"]:

// SQL get tr_road_all: Time difference = 29916[ms]
// Parse SQL result: Time difference = 85360[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 83995[ms]
// Parse SQL result: Time difference = 283855[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 17493[ms]
// Parse SQL result: Time difference = 81744[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 20468[ms]
// Parse SQL result: Time difference = 62712[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 18287[ms]
// Parse SQL result: Time difference = 63737[ms]
// Count: 1234693



// When lifted the geom_points parsing to a separate function:

// SQL get tr_road_all: Time difference = 17322[ms]
// Parse SQL result: Time difference = 75808[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 14217[ms]
// Parse SQL result: Time difference = 47573[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 16709[ms]
// Parse SQL result: Time difference = 41798[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 14238[ms]
// Parse SQL result: Time difference = 33345[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 13691[ms]
// Parse SQL result: Time difference = 32695[ms]
// Count: 1234693


// When changed from std::string to double for the geom_points parsing:

// SQL get tr_road_all: Time difference = 25233[ms]
// Parse SQL result: Time difference = 42326[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 24705[ms]
// Parse SQL result: Time difference = 39925[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 17628[ms]
// Parse SQL result: Time difference = 38272[ms]
// Count: 1234693


// When reverted back to using string:

// SQL get tr_road_all: Time difference = 24085[ms]
// Parse SQL result: Time difference = 81767[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 19245[ms]
// Parse SQL result: Time difference = 51175[ms]
// Count: 1234693

// SQL get tr_road_all: Time difference = 22654[ms]
// Parse SQL result: Time difference = 62005[ms]
// Count: 1234693
