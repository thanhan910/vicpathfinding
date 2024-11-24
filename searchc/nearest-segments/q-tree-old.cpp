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

// MULTILINESTRING((144.10242311600007 -38.455164802999946,144.10346247500001 -38.45479963199995)
std::chrono::steady_clock::time_point begin, end;

pqxx::connection conn("dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432");

size_t max_p = 0;

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
        std::vector<std::string> lineStr;
        // lineStr.push_back(line);
        // Split the line by comma
        line = line.substr(1);
        for (int i = 0; i < line.size(); i++) {
            if (line[i] == ',') {
                lineStr.push_back(line.substr(0, i));
                line = line.substr(i+1);
                i = 0;
            }
            else if (line[i] == ' ')
            {
                lineStr.push_back(line.substr(0, i));
                line = line.substr(i+1);
                i = 0;
            }
        }
        lineStr.push_back(line);

        for (const auto& p : lineStr) {
            // std::cout << "s: " << p << " " << p.size() << std::endl;
            max_p = std::max(max_p, p.size());
            std::istringstream pi_stream(p);
            long double x;
            pi_stream >> x;
            // std::cout << "d: " << x << std::endl;
        }

        // lineStr = line.substr(1);
        // std::cout << "Line: " << line << std::endl;

        std::string point;
        while (std::getline(points_stream, point, ',')) {
            std::istringstream point_stream(point);
            // double x, y;
            // x and y are 16 characters long
            double x, y;
            point_stream >> x >> y; // Read x and y coordinates
            // Print number of digits of x and y
            // std::cout << "x: " << std::to_string(x).size() << ", y: " << std::to_string(y).size() << std::endl;
            lineString.emplace_back(x, y);
        }
        
        multiLineString.push_back(lineString);
    }
    return multiLineString;
}


std::unordered_map<int, std::vector<std::pair<double, double>>> get_results() {
    pqxx::work txn(conn);

    std::string query = "SELECT ufi, ST_AsText(geom) FROM vmtrans.tr_road_all";

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

    return roads;
}

// Point structure
struct Point {
    double x, y;

    double distanceTo(const Point& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

// Segment structure
// template <typename T>
struct Segment {
    Point p1, p2;

    int roadufi;

    Point midpoint() const {
        return {(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
    }

    // Compute the distance from a point to the line segment
    double perpendicularDistanceToPoint(const Point& p) const {
        // Project p onto the line segment, clamp to endpoints
        double A = p.x - p1.x;
        double B = p.y - p1.y;
        double C = p2.x - p1.x;
        double D = p2.y - p1.y;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = (len_sq != 0) ? dot / len_sq : -1;

        double xx, yy;
        if (param < 0) {
            xx = p1.x;
            yy = p1.y;
        } else if (param > 1) {
            xx = p2.x;
            yy = p2.y;
        } else {
            xx = p1.x + param * C;
            yy = p1.y + param * D;
        }

        double dx = p.x - xx;
        double dy = p.y - yy;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// Quadtree node
class Quadtree {
public:
    // Bounding box for each quadrant
    struct Boundary {
        double x_min, y_min, x_max, y_max;

        bool contains(const Point& p) const {
            return (p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max);
        }

        bool intersects(const Boundary& other) const {
            return !(other.x_min > x_max || other.x_max < x_min ||
                     other.y_min > y_max || other.y_max < y_min);
        }
    };

    Quadtree(Boundary boundary, int capacity = 4)
        : boundary(boundary), capacity(capacity), divided(false) {}

    // Insert a segment
    bool insert(const Segment& segment) {
        Point midpoint = segment.midpoint();
        if (!boundary.contains(midpoint)) return false;

        if (segments.size() < capacity) {
            segments.push_back(segment);
            return true;
        }

        if (!divided) subdivide();

        return (northwest->insert(segment) || northeast->insert(segment) ||
                southwest->insert(segment) || southeast->insert(segment));
    }

    // Find the nearest segment to a point
    Segment nearestSegment(const Point& point, double& minDistance) const {
        Segment nearest;
        minDistance = std::numeric_limits<double>::infinity();

        for (const Segment& segment : segments) {
            double distance = segment.perpendicularDistanceToPoint(point);
            if (distance < minDistance) {
                minDistance = distance;
                nearest = segment;
            }
        }

        if (divided) {
            if (northwest->boundary.contains(point)) {
                nearest = northwest->nearestSegment(point, minDistance);
            }
            if (northeast->boundary.contains(point) && minDistance > 0) {
                nearest = northeast->nearestSegment(point, minDistance);
            }
            if (southwest->boundary.contains(point) && minDistance > 0) {
                nearest = southwest->nearestSegment(point, minDistance);
            }
            if (southeast->boundary.contains(point) && minDistance > 0) {
                nearest = southeast->nearestSegment(point, minDistance);
            }
        }

        return nearest;
    }

private:
    Boundary boundary;
    int capacity;
    bool divided;
    std::vector<Segment> segments;

    // Quadrants
    Quadtree* northwest;
    Quadtree* northeast;
    Quadtree* southwest;
    Quadtree* southeast;

    // Divide the current quadrant into 4
    void subdivide() {
        double x_mid = (boundary.x_min + boundary.x_max) / 2;
        double y_mid = (boundary.y_min + boundary.y_max) / 2;

        northwest = new Quadtree({boundary.x_min, boundary.y_min, x_mid, y_mid}, capacity);
        northeast = new Quadtree({x_mid, boundary.y_min, boundary.x_max, y_mid}, capacity);
        southwest = new Quadtree({boundary.x_min, y_mid, x_mid, boundary.y_max}, capacity);
        southeast = new Quadtree({x_mid, y_mid, boundary.x_max, boundary.y_max}, capacity);

        divided = true;
    }
};

std::unordered_map<int, std::pair<double, double>> get_stops() {
    pqxx::work txn(conn);

    std::string query = "SELECT stop_id, stop_lat, stop_lon FROM vmtrans.stops";

    pqxx::result result = txn.exec(query);

    std::unordered_map<int, std::pair<double, double>> stops;

    for (auto row : result) {
        int id = row[0].as<int>();
        double lat = row[1].as<double>();
        double lon = row[2].as<double>();
        // stops.emplace_back(id, std::make_pair(lat, lon));
        stops[id] = std::make_pair(lon, lat);
    }

    return stops;
}

// Example usage
int main() {
    // Define the boundary of the quadtree (whole plane or a limited area)
    std::unordered_map<int, std::vector<std::pair<double, double>>> roads = get_results();

    begin = std::chrono::steady_clock::now();
    std::unordered_map<int, std::pair<double, double>> stops = get_stops();
    end = std::chrono::steady_clock::now();
    std::cout << "Get stops: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    std::cout << "max p: " << max_p << std::endl;

    std::cout << "Generate segment" << std::endl;

    begin = std::chrono::steady_clock::now();

    std::vector<std::pair<double, double>> segments_points;
    std::vector<std::pair<std::pair<std::pair<double, double>, std::pair<double, double>>, int>> segments;
    for (const auto& [id, road] : roads) {
        for (const auto& [x, y] : road) {
            segments_points.emplace_back(
                x, 
                y
            );
        }
        for (size_t i=0; i<road.size()-1; i++) {
            double x1 = road[i].first;
            double y1 = road[i].second;
            double x2 = road[i+1].first;
            double y2 = road[i+1].second;
            segments.push_back({{{x1, y1}, {x2, y2}}, id});
            // segments.push_back({road[i], road[i+1]});
        }
    }

    end = std::chrono::steady_clock::now();
    std::cout << "Generate segment: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    // Point queryPointOriginal = {144.10242311600007, -38.455264802999946};
    // std::cout << "Query point: (" << std::to_string(queryPointOriginal.x) << ", " << std::to_string(queryPointOriginal.y) << ")" << std::endl;

    begin = std::chrono::steady_clock::now();

    double x_min = std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    for (const auto& [x, y] : segments_points) {
        x_min = std::min(x_min, x);
        y_min = std::min(y_min, y);
        x_max = std::max(x_max, x);
        y_max = std::max(y_max, y);
    }

    end = std::chrono::steady_clock::now();

    std::cout << "Find min max coords: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    // Quadtree::Boundary boundary = {0, 0, 100, 100};
    // Quadtree quadtree(boundary);
    Quadtree::Boundary boundary = {x_min, y_min, x_max, y_max};
    Quadtree quadtree(boundary);

    end = std::chrono::steady_clock::now();

    std::cout << "Create quadtree from boundary: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    for (int i = 0; i < segments.size(); i++) {
        double x1 = segments[i].first.first.first;
        double y1 = segments[i].first.first.second;
        double x2 = segments[i].first.second.first;
        double y2 = segments[i].first.second.second;
        int roadufi = segments[i].second;
        Segment segment = {{x1, y1}, {x2, y2}, roadufi};
        quadtree.insert(segment);
    }

    end = std::chrono::steady_clock::now();

    int64_t time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    int i = 0;
    begin = std::chrono::steady_clock::now();
    for (const auto& [id, stop] : stops) {
        i++;
        Point p = {stop.first, stop.second};
        std::cout << "Stop: " << i << " " << id << " (" << stop.first << ", " << stop.second << ")" << std::endl;
        double minDistance;
        Segment nearestSegment = quadtree.nearestSegment(p, minDistance);
        std::cout << "Nearest segment to (" << stop.first << ", " << stop.second << ") "
                  << "is from (" << nearestSegment.p1.x << ", " << nearestSegment.p1.y << ") "
                  << "to (" << nearestSegment.p2.x << ", " << nearestSegment.p2.y << ") "
                  << "with distance: " << minDistance << " "
                  << "roadufi: " << nearestSegment.roadufi << std::endl;
    }
    end = std::chrono::steady_clock::now();
    std::cout << "Total stops: " << stops.size() << std::endl;
    std::cout << "Find nearest segment: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    std::cout << "Insert segments: time difference = " << time_diff << "[ms]" << std::endl;
    
    return 0;

    // g++ r-tree.cpp -o mytest $(pkg-config --cflags --libs libpqxx libpq) && ./mytest
}
