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

// Structure to store each row's data
struct RoadData
{
    int ufi;
    std::string ezi_road_name_label;
    std::string direction_code;
    double road_length_meters;
    std::vector<std::pair<std::string, std::string>> geom_points;
};

// Function to parse points from a single string into a vector of points
std::vector<std::string> parsePoints(const std::string &points_str)
{
    std::vector<std::string> points;
    std::istringstream iss(points_str);
    std::string point;

    // Split by comma
    while (std::getline(iss, point, ','))
    {
        points.push_back(point);
    }
    return points;
}

std::size_t get_results()
{
    pqxx::work txn(conn);

    // Read table using the following query
    // SELECT
    //     ufi,
    //     ezi_road_name_label,
    //     direction_code,
    //     road_length_meters,
    //     STRING_TO_ARRAY(
    //         REGEXP_REPLACE(ST_AsText(geom), '^MULTILINESTRING\\(\\(|\\)\\)$', '', 'g'),
    //         ','
    //     ) AS geom_points
    // FROM
    //     vmtrans.tr_road_all;

    std::string query = R"(
        SELECT 
            ufi,
            ezi_road_name_label,
            direction_code,
            road_length_meters,
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

    size_t iter = 0;

    // Process each row in the result
    for (const auto &row : result)
    {
        iter++;
        RoadData data;
        data.ufi = row["ufi"].as<int>();
        // ezi_road_name_label can be null
        if (row["ezi_road_name_label"].is_null())
        {
            data.ezi_road_name_label = "";
        }
        else
        {
            data.ezi_road_name_label = row["ezi_road_name_label"].as<std::string>();
        }
        if (row["direction_code"].is_null())
        {
            data.direction_code = "";
        }
        else
        {
            data.direction_code = row["direction_code"].as<std::string>();
        }
        data.road_length_meters = row["road_length_meters"].as<double>();

        // Parse the geometry points
        
        pqxx::array<std::string> geom_points_sql_array = row["geom_points"].as_sql_array<std::string>();
        size_t size = geom_points_sql_array.size();        
        for (size_t i = 0; i < size; i++)
        {
            std::string point = geom_points_sql_array[i];
            std::istringstream iss(point);
            std::string token;
            std::vector<std::string> elems;
            while (std::getline(iss, token, ' '))
            {
                elems.push_back(token);
            }
            std::string point_lon_str = elems[0];
            std::string point_lat_str = elems[1];
            std::pair<std::string, std::string> point_pair(point_lon_str, point_lat_str);
            data.geom_points.push_back(point_pair);
        }

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