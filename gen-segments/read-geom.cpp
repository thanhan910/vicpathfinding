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
std::vector<std::vector<std::pair<double, double>>>parseWKTtoMultiLineString(const std::string& wkt) {
    std::vector<std::vector<std::pair<double, double>>>multiLineString;
    
    // Remove the "MULTILINESTRING(" and ")" from the WKT string
    std::string stripped_wkt = wkt.substr(16, wkt.size() - 17);
    std::istringstream lines_stream(stripped_wkt);
    std::string line;
    
    while (std::getline(lines_stream, line, ')')) {
        // Skip any commas at the beginning of each line string segment
        if (!line.empty() && line[0] == ',') line = line.substr(2);
        
        std::vector<std::pair<double, double>> lineString;
        std::istringstream points_stream(line.substr(1)); // Remove the leading '('

        std::string point;
        while (std::getline(points_stream, point, ',')) {
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


std::size_t get_results() {
    pqxx::work txn(conn);

    std::string query = "SELECT ufi, ST_AsText(geom) FROM vmtrans.tr_road_all WHERE direction_code IS NOT NULL;";

    begin = std::chrono::steady_clock::now();
    pqxx::result result = txn.exec(query);
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();
    std::unordered_map<int, std::vector<std::pair<double, double>>> roads;

    for (auto row : result) {

        int id = row[0].as<int>();
        std::string multiline_wkt = row[1].as<std::string>();
        
        auto multiLineString = parseWKTtoMultiLineString(multiline_wkt);

        assert(multiLineString.size() == 1);

        roads[id] = multiLineString[0];

        // std::cout << "ID: " << id << ", Parsed Geometry:" << multiline_wkt << std::endl;
        // for (const auto& line : multiLineString) {
        //     std::cout << "  Line:";
        //     for (const auto& [x, y] : line) {
        //         std::cout << " (" << x << ", " << y << ")";
        //     }
        //     std::cout << std::endl;
        // }
        // rows.emplace_back(id, multiLineString);
    }

    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return roads.size();
}

int main() {
    std::size_t count = get_results();
    std::cout << "Count: " << count << std::endl;

    return 0;
}


// Time difference = 23782[ms]
// Time difference = 31045[ms]
// Count: 1235086

// Time difference = 13698[ms]
// Time difference = 13905[ms]
// Count: 1235086

// Time difference = 41031[ms]
// Time difference = 39682[ms]
// Count: 1235086

// Time difference = 29496[ms]
// Time difference = 29291[ms]
// Count: 1235086

// Time difference = 26900[ms]
// Time difference = 33493[ms]
// Count: 1235086