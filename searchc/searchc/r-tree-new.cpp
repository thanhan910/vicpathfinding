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

// MULTILINESTRING((144.10242311600007 -38.455164802999946,144.10346247500001 -38.45479963199995)
std::chrono::steady_clock::time_point begin, end;

pqxx::connection conn("dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432");

size_t max_p = 0;

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
    double distanceToPoint(const Point& p) const {
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
            double distance = segment.distanceToPoint(point);
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

std::vector<Segment> get_segments() {
    pqxx::work txn(conn);

    std::string query = "SELECT * FROM vmtrans.segments";

    begin = std::chrono::steady_clock::now();
    pqxx::result result = txn.exec(query);
    end = std::chrono::steady_clock::now();
    std::cout << "Get from PostgreSQL = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    std::vector<Segment> segments;

    for (auto row : result) {
        int roadufi = row[0].as<int>();
        double x1 = row[1].as<double>();
        double y1 = row[2].as<double>();
        double x2 = row[3].as<double>();
        double y2 = row[4].as<double>();
        Point p1 = {x1, y1};
        Point p2 = {x2, y2};
        Segment segment = {p1, p2, roadufi};
        segments.push_back(segment);
    }

    end = std::chrono::steady_clock::now();
    std::cout << "Gen segments = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return segments;
}

std::tuple<double, double, double, double> get_min_max_coords() {
    pqxx::work txn(conn);

    std::string query = "SELECT * FROM vmtrans.boundary";

    pqxx::result result = txn.exec(query);

    double x_min = result[0][0].as<double>();
    double y_min = result[0][1].as<double>();
    double x_max = result[0][2].as<double>();
    double y_max = result[0][3].as<double>();

    x_min = std::floor(x_min) - 1;
    y_min = std::floor(y_min) - 1;
    x_max = std::ceil(x_max) + 1;
    y_max = std::ceil(y_max) + 1;

    std::cout << "x_min: " << x_min << " y_min: " << y_min << " x_max: " << x_max << " y_max: " << y_max << std::endl;

    return std::make_tuple(x_min, y_min, x_max, y_max);
}

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


Quadtree gen_quadtree() {
    std::vector<Segment> segments = get_segments();

    begin = std::chrono::steady_clock::now();
    auto [x_min, y_min, x_max, y_max] = get_min_max_coords();
    end = std::chrono::steady_clock::now();
    std::cout << "Get min max coords: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    
    begin = std::chrono::steady_clock::now();
    Quadtree::Boundary boundary = {x_min, y_min, x_max, y_max};
    Quadtree quadtree(boundary);
    end = std::chrono::steady_clock::now();
    std::cout << "Create quadtree from boundary: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    for (const Segment& segment : segments) {
        quadtree.insert(segment);
    }

    end = std::chrono::steady_clock::now();

    std::cout << "Insert segments: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return quadtree;
}

// Example usage
int main() {

    Quadtree quadtree = gen_quadtree();

    begin = std::chrono::steady_clock::now();
    std::unordered_map<int, std::pair<double, double>> stops = get_stops();
    end = std::chrono::steady_clock::now();
    std::cout << "Get stops: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    int i = 0;

    // csv file
    std::ofstream file("stops_nearest_segment.csv");
    file << "stop_id,stop_lat,stop_lon,roadufi,segment_x1,segment_y1,segment_x2,segment_y2,distance" << std::endl;


    begin = std::chrono::steady_clock::now();
    for (const auto& [id, stop] : stops) {
        i++;
        Point p = {stop.first, stop.second};
        // std::cout << "Stop: " << i << " " << id << " (" << stop.first << ", " << stop.second << ")" << std::endl;
        double minDistance = 361;
        Segment nearestSegment = quadtree.nearestSegment(p, minDistance);
        file << id << "," << stop.first << "," << stop.second << "," << nearestSegment.roadufi << "," << nearestSegment.p1.x << "," << nearestSegment.p1.y << "," << nearestSegment.p2.x << "," << nearestSegment.p2.y << "," << minDistance << std::endl;
        if (nearestSegment.roadufi < 0) {
             std::cout << "Nearest segment to (" << stop.first << ", " << stop.second << ") "
                  << "is from (" << nearestSegment.p1.x << ", " << nearestSegment.p1.y << ") "
                  << "to (" << nearestSegment.p2.x << ", " << nearestSegment.p2.y << ") "
                  << "with distance: " << minDistance << " "
                  << "roadufi: " << nearestSegment.roadufi << std::endl;
        }
    }
    end = std::chrono::steady_clock::now();
    std::cout << "Total stops: " << stops.size() << std::endl;
    std::cout << "Find nearest segment: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    file.close();

    return 0;

    // g++ r-tree.cpp -o mytest $(pkg-config --cflags --libs libpqxx libpq) && ./mytest
}
