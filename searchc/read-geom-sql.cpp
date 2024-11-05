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

using CoordinateValue = std::string;
using CoordinatePair = std::pair<CoordinateValue, CoordinateValue>;

std::vector<CoordinatePair> parseGeomPoints(const pqxx::field &geom_points_sql)
{
    pqxx::array<std::string> geom_points_sql_array = geom_points_sql.as_sql_array<std::string>();
    std::vector<CoordinatePair> geom_points;
    for (size_t i = 0; i < geom_points_sql_array.size(); i++)
    {
        std::string point = geom_points_sql_array[i];
        std::istringstream iss(point);
        std::string token;
        std::vector<std::string> elems;
        while (std::getline(iss, token, ' '))
        {
            elems.push_back(token);
        }
        CoordinateValue point_lon_str = elems[0];
        CoordinateValue point_lat_str = elems[1];
        CoordinatePair point_pair(point_lon_str, point_lat_str);
        geom_points.push_back(point_pair);
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

    std::string query = R"(
        SELECT 
            ufi,
            STRING_TO_ARRAY(
                REGEXP_REPLACE(ST_AsText(geom), '^MULTILINESTRING\(\(|\)\)$', '', 'g'),
                ','
            ) AS geom_points
        FROM 
            vmtrans.tr_road_all
        WHERE direction_code IS NOT NULL;
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
        // // ezi_road_name_label can be null
        // if (row["ezi_road_name_label"].is_null())
        // {
        //     data.ezi_road_name_label = "";
        // }
        // else
        // {
        //     data.ezi_road_name_label = row["ezi_road_name_label"].as<std::string>();
        // }
        // if (row["direction_code"].is_null())
        // {
        //     data.direction_code = "";
        // }
        // else
        // {
        //     data.direction_code = row["direction_code"].as<std::string>();
        // }
        // data.road_length_meters = row["road_length_meters"].as<double>();

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
// root@d46c569b1b86:/workspaces/vicpathfinding/searchc#


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
// root@d46c569b1b86:/workspaces/vicpathfinding/searchc#