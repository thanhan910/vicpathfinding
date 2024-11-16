#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <iostream>
#include <pqxx/pqxx>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>

#include "number.h"

#define USE_STRING
#define USE_FINE_TUNED_ESTIMATION

#ifdef USE_STRING
using Component = std::string;
#else
using Component = double;
#endif
using Coordinate = std::pair<Component, Component>;
using Line = std::vector<Coordinate>;
using Ufi = int;
using RoadMap = std::unordered_map<Ufi, Line>;

std::chrono::steady_clock::time_point begin, end;

pqxx::connection conn("dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432");

size_t max_p = 0;

// Function to parse a WKT MULTILINESTRING into vector of vector of (x, y) pairs
std::vector<Line> parseWKTtoMultiLineString(const std::string &wkt)
{
    std::vector<Line> multiLineString;

    // Remove the "MULTILINESTRING(" and ")" from the WKT string
    std::string stripped_wkt = wkt.substr(16, wkt.size() - 17);
    std::istringstream lines_stream(stripped_wkt);
    std::string line;

    while (std::getline(lines_stream, line, ')'))
    {
        // Skip any commas at the beginning of each line string segment
        if (!line.empty() && line[0] == ',')
            line = line.substr(2);

        Line lineString;
        std::istringstream points_stream(line.substr(1)); // Remove the leading '('
        std::vector<std::string> lineStr;
        // lineStr.push_back(line);
        // Split the line by comma
        line = line.substr(1);
        for (int i = 0; i < line.size(); i++)
        {
            if (line[i] == ',')
            {
                lineStr.push_back(line.substr(0, i));
                line = line.substr(i + 1);
                i = 0;
            }
            else if (line[i] == ' ')
            {
                lineStr.push_back(line.substr(0, i));
                line = line.substr(i + 1);
                i = 0;
            }
        }
        lineStr.push_back(line);

        std::string point;
        while (std::getline(points_stream, point, ','))
        {
            // Split the point into two strings by space
            std::istringstream point_stream(point);
#ifdef USE_STRING
            std::string x_str, y_str;
            std::getline(point_stream, x_str, ' ');
            std::getline(point_stream, y_str, ' ');
            lineString.emplace_back(x_str, y_str);
#else
            Component x, y;
            point_stream >> x >> y;
            lineString.emplace_back(x, y);
#endif
        }

        multiLineString.push_back(lineString);
    }
    return multiLineString;
}

std::unordered_map<Ufi, Line> get_results()
{
    pqxx::work txn(conn);

    std::string query = "SELECT ufi, ST_AsText(geom) FROM vmtrans.tr_road_all WHERE direction_code IS NOT NULL;";

    begin = std::chrono::steady_clock::now();
    std::cout << "Get all from PostgreSQL" << std::endl;
    pqxx::result result = txn.exec(query);
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();
    std::cout << "Parse SQL result:" << std::endl;
    std::unordered_map<int, Line> roads;

    for (auto row : result)
    {

        Ufi id = row[0].as<int>();
        std::string multiline_wkt = row[1].as<std::string>();

        std::vector<Line> multiLineString = parseWKTtoMultiLineString(multiline_wkt);

        assert(multiLineString.size() == 1);

        roads[id] = multiLineString[0];
    }

    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return roads;
}

// Point structure
struct Point
{
    Component x, y;
};

// Segment structure
// template <typename T>
struct Segment
{
    Point p1, p2;

    int roadufi;
};

struct SegmentsData
{
    std::vector<Segment> segments;
    Component x_min, y_min, x_max, y_max;
};

SegmentsData gen_segments(const RoadMap &roads)
{

    std::cout << "Get segments and min max boundary:" << std::endl;
    begin = std::chrono::steady_clock::now();
#ifdef USE_STRING
#ifdef USE_FINE_TUNED_ESTIMATION
    Component x_min = "360.0";
    Component y_min = "-10.0";
    Component x_max = "100.0";
    Component y_max = "-90.0";
#else
    Component x_min = "90";
    Component y_min = "360";
    Component x_max = "-90";
    Component y_max = "0";
#endif

#else

    double x_min = std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
#endif
    std::vector<Segment> segments;

    int count = 0;
    int bar_count = 0;
    int bar_size = 10000;

    for (const auto &[id, road] : roads)
    {
        for (const auto &[x, y] : road)
        {
#ifdef USE_STRING
#ifdef USE_FINE_TUNED_ESTIMATION
            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            y_min = std::max(y_min, y);
            y_max = std::min(y_max, y);
#else
            x_min = is_greater(x_min, x) ? x : x_min;
            y_min = is_greater(y_min, y) ? y : y_min;
            x_max = is_greater(x, x_max) ? x : x_max;
            y_max = is_greater(y, y_max) ? y : y_max;
#endif
#else
            x_min = std::min(x_min, x);
            y_min = std::min(y_min, y);
            x_max = std::max(x_max, x);
            y_max = std::max(y_max, y);
#endif
        }
        for (size_t i = 0; i < road.size() - 1; i++)
        {
            Component x1 = road[i].first;
            Component y1 = road[i].second;
            Component x2 = road[i + 1].first;
            Component y2 = road[i + 1].second;
            Segment segment = {{x1, y1}, {x2, y2}, id};
            segments.push_back(segment);
            // segments.push_back({road[i], road[i+1]});
        }
        count++;
        if (count % bar_size == 0)
        {
            bar_count++;
            std::cout << "Bar " << bar_count << std::endl;
        }
    }

    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    return {segments, x_min, y_min, x_max, y_max};
}

// Example usage
int main()
{

    RoadMap roads = get_results();

    auto [segments, x_min, y_min, x_max, y_max] = gen_segments(roads);

    // Write segments to a csv file
    begin = std::chrono::steady_clock::now();

    std::ofstream segments_file("segments.csv");

    segments_file << "roadufi,x1,y1,x2,y2\n";

    for (const Segment &segment : segments)
    {
        // Write data to file
        segments_file << segment.roadufi << "," << segment.p1.x << "," << segment.p1.y << ","
                      << segment.p2.x << "," << segment.p2.y << ","
                      << "\n";
    }

    segments_file.close();

    end = std::chrono::steady_clock::now();
    std::cout << "Write segments to a csv file: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    // Save the min max boundary to a file
    std::ofstream boundary_file("boundary.csv");
    boundary_file << "x_min,y_min,x_max,y_max\n";
    boundary_file << x_min << "," << y_min << "," << x_max << "," << y_max << "\n";
    boundary_file.close();

    std::cout << "x_min: " << x_min << std::endl;
    std::cout << "y_min: " << y_min << std::endl;
    std::cout << "x_max: " << x_max << std::endl;
    std::cout << "y_max: " << y_max << std::endl;

    return 0;

    // g++ r-tree.cpp number.cpp -o mytest $(pkg-config --cflags --libs libpqxx libpq) && ./mytest
}
