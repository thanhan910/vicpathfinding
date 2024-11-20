#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <stack>
#include <chrono>
#include <iostream>
#include <pqxx/pqxx>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>

// #define TEST_STOPS


std::chrono::steady_clock::time_point begin, end;

pqxx::connection conn("dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432");

size_t max_p = 0;

// Point structure
struct Point
{
    double x, y;

    double distanceTo(const Point &other) const
    {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }

    double cross(const Point &other) const
    {
        return x * other.y - y * other.x;
    }
};

double cross(double dx1, double dy1, double dx2, double dy2)
{
    return dx1 * dy2 - dy1 * dx2;
}

struct Segment
{
    Point p1, p2;

    int roadufi;

    Point midpoint() const
    {
        return {(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
    }

    // Compute the distance from a point to the line segment
    double distanceToPoint(const Point &p) const
    {
        // Project p onto the line segment, clamp to endpoints
        double A = p.x - p1.x;
        double B = p.y - p1.y;
        double C = p2.x - p1.x;
        double D = p2.y - p1.y;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = (len_sq != 0) ? dot / len_sq : -1;

        double xx, yy;
        if (param < 0)
        {
            xx = p1.x;
            yy = p1.y;
        }
        else if (param > 1)
        {
            xx = p2.x;
            yy = p2.y;
        }
        else
        {
            xx = p1.x + param * C;
            yy = p1.y + param * D;
        }

        double dx = p.x - xx;
        double dy = p.y - yy;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool intersects(const double &x1, const double &y1, const double &x2, const double &y2, const bool &is_vertical) const
    {

        // lambda function to check if a horizontal or vertical line contains a point
        auto contains = [](const Point &p, const double &x1, const double &y1, const double &x2, const double &y2, const bool &is_vertical)
        {
            if (is_vertical)
            {
                return (p.x == x1 && p.y >= y1 && p.y <= y2);
            }
            else
            {
                return (p.y == y1 && p.x >= x1 && p.x <= x2);
            }
        };

        if (is_vertical)
        {
            bool crossLine = (p1.x < x1 && p2.x > x1) || (p1.x > x1 && p2.x < x1);
            if (!crossLine)
            {
                if (p1.x == x1)
                    return contains(p1, x1, y1, x2, y2, is_vertical);
                if (p2.x == x1)
                    return contains(p2, x1, y1, x2, y2, is_vertical);
                else
                    return false;
            }
        }
        else
        {
            bool crossLine = (p1.y < y1 && p2.y > y1) || (p1.y > y1 && p2.y < y1);
            if (!crossLine)
            {
                if (p1.y == y1)
                    return contains(p1, x1, y1, x2, y2, is_vertical);
                else if (p2.y == y1)
                    return contains(p2, x1, y1, x2, y2, is_vertical);
                else
                    return false;
            }
        }

        double delta_y = p2.y - p1.y;
        double delta_x = p2.x - p1.x;
        if (is_vertical)
        {
            double line_intersect_y = p1.y + delta_y * (x1 - p1.x) / delta_x;
            return (line_intersect_y >= y1 && line_intersect_y <= y2);
        }
        else
        {
            double line_intersect_x = p1.x + delta_x * (y1 - p1.y) / delta_y;
            return (line_intersect_x >= x1 && line_intersect_x <= x2);
        }
    }
};

// Quadtree node
class Quadtree
{
public:
    int segment_count = 0;
    // Bounding box for each quadrant
    struct Boundary
    {
        double x_min, y_min, x_max, y_max;

        bool contains(const Point &p) const
        {
            return (p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max);
        }

        bool intersects(const Boundary &other) const
        {
            return !(other.x_min > x_max || other.x_max < x_min ||
                     other.y_min > y_max || other.y_max < y_min);
        }

        bool intersects(const Segment &segment) const
        {
            if (contains(segment.p1) || contains(segment.p2))
                return true;

            return (
                segment.intersects(x_min, y_max, x_max, y_max, false) || segment.intersects(x_max, y_max, x_max, y_min, true) || segment.intersects(x_max, y_min, x_min, y_min, false) || segment.intersects(x_min, y_min, x_min, y_max, true));
        }
    };

    Quadtree(Boundary boundary, int capacity = 4)
        : boundary(boundary), capacity(capacity), divided(false) {}

    // Insert a segment
    bool insert(Segment segment)
    {
        // std::cout << "Insert segment: " << segment.p1.x << " " << segment.p1.y << " " << segment.p2.x << " " << segment.p2.y << std::endl;

        if (!(boundary.intersects(segment)))
            return false;

        if (!divided) {
            if ((segments.size() < capacity || (boundary.x_max - boundary.x_min) < 0.000001 || (boundary.y_max - boundary.y_min) < 0.000001))
            {
                segments.push_back(segment);
                if (boundary.contains(segment.midpoint()))
                {
                    midpoint_count++;
                }
                segment_count++;
                return true;
            }
            else {
                subdivide();
            }
        }

        for (Quadtree *child : children)
        {
            if (child->insert(segment))
            {
                segment_count++;
                return true;
            }
        }
        return false;
    }

    // Find the nearest segment to a point
    std::tuple<Segment, std::vector<Quadtree *>> nearestSegment(const Point &point, double &minDistance) const
    {
        std::cout << "Quadtree: " << boundary.x_min << ", " << boundary.y_min << ", " << boundary.x_max << ", " << boundary.y_max << std::endl;
        // minDistance = std::numeric_limits<double>::infinity();

        if (divided)
        {
            std::vector<Quadtree *> container_quads;
            std::vector<Quadtree *> non_container_quads;
            for (Quadtree *child : children)
            {
                if (child->boundary.contains(point))
                {
                    container_quads.push_back(child);
                }
                else
                {
                    non_container_quads.push_back(child);
                }
            }
            for (Quadtree *child : container_quads)
            {
                if (child->segment_count == 0)
                {
                    continue;
                }
                else
                {
                    auto [segment, quad] = child->nearestSegment(point, minDistance);
                    quad.push_back(const_cast<Quadtree *>(this));
                    return {segment, quad};
                }
            }
            Segment nearestSegment;
            std::vector<Quadtree *> quads;
            for (Quadtree *child : non_container_quads)
            {
                if (child->segment_count == 0)
                {
                    continue;
                }
                auto [segment, quad] = child->nearestSegment(point, minDistance);
                nearestSegment = segment;
                quads = quad;
            }
            quads.push_back(const_cast<Quadtree *>(this));
            return {nearestSegment, quads};
        }

        else
        {
            Segment nearestSegment;

            std::vector<Quadtree *> quads;
            // Segment nearest;
            for (const Segment& segment : segments)
            {
                std::cout << segment.p1.x << "," << segment.p1.y << "," << segment.p2.x << "," << segment.p2.y << std::endl; 
                std::cout << "Nearest segment: " << nearestSegment.p1.x << "," << nearestSegment.p1.y << "," << nearestSegment.p2.x << "," << nearestSegment.p2.y << std::endl;
                double distance = segment.distanceToPoint(point);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    // nearestSegment = segment; Reference move instead of copy
                    nearestSegment = segment;
                    std::cout << "Nearest segment chosen: " << nearestSegment.p1.x << "," << nearestSegment.p1.y << "," << nearestSegment.p2.x << "," << nearestSegment.p2.y << std::endl;
                }
            }
            quads.push_back(const_cast<Quadtree *>(this));
            return {nearestSegment, quads};
        }
    }

    // Return bounding box
    Boundary getBoundary() const
    {
        return boundary;
    }

    // Return the segments
    std::vector<Segment> getSegments()
    {
        return segments;
    }

    bool isDivided() const
    {
        return divided;
    }

    std::vector<Quadtree *> getChildren() const
    {
        return children;
    }

private:
    Boundary boundary;
    int capacity;
    bool divided;
    std::vector<Segment> segments;
    int midpoint_count;

    // Quadrants
    std::vector<Quadtree *> children;

    // Divide the current quadrant into 4
    void subdivide()
    {
        double x_mid = (boundary.x_min + boundary.x_max) / 2;
        double y_mid = (boundary.y_min + boundary.y_max) / 2;

        children.push_back(new Quadtree({boundary.x_min, boundary.y_min, x_mid, y_mid}, capacity));
        children.push_back(new Quadtree({x_mid, boundary.y_min, boundary.x_max, y_mid}, capacity));
        children.push_back(new Quadtree({boundary.x_min, y_mid, x_mid, boundary.y_max}, capacity));
        children.push_back(new Quadtree({x_mid, y_mid, boundary.x_max, boundary.y_max}, capacity));

        for (const Segment& seg : segments)
        {
            for (Quadtree *child : children)
            {
                child->insert(seg);
            }
        }

        // Clear the segments vector
        segments.clear();

        divided = true;
    }
};

std::vector<Segment> get_segments()
{
    pqxx::work txn(conn);

    std::string query = "SELECT * FROM vmtrans.segments";

    begin = std::chrono::steady_clock::now();
    pqxx::result result = txn.exec(query);
    end = std::chrono::steady_clock::now();
    std::cout << "Get from PostgreSQL = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    std::vector<Segment> segments;

    for (auto row : result)
    {
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

std::tuple<double, double, double, double> get_min_max_coords()
{
    pqxx::work txn(conn);

    std::string query = "SELECT * FROM vmtrans.boundary";

    pqxx::result result = txn.exec(query);

    double x_min = result[0][0].as<double>();
    double y_min = result[0][1].as<double>();
    double x_max = result[0][2].as<double>();
    double y_max = result[0][3].as<double>();

    x_min = std::floor(x_min) - 1.0;
    y_min = std::floor(y_min) - 1.0;
    x_max = std::ceil(x_max) + 1.0;
    y_max = std::ceil(y_max) + 1.0;

    std::cout << "x_min: " << x_min << " y_min: " << y_min << " x_max: " << x_max << " y_max: " << y_max << std::endl;

    return std::make_tuple(x_min, y_min, x_max, y_max);
}

std::unordered_map<int, std::pair<double, double>> get_stops()
{
    pqxx::work txn(conn);

    std::string query = "SELECT stop_id, stop_lat, stop_lon FROM vmtrans.stops";

    pqxx::result result = txn.exec(query);

    std::unordered_map<int, std::pair<double, double>> stops;

    for (auto row : result)
    {
        int id = row[0].as<int>();
        double lat = row[1].as<double>();
        double lon = row[2].as<double>();
        // stops.emplace_back(id, std::make_pair(lat, lon));
        stops[id] = std::make_pair(lon, lat);
    }

    return stops;
}

Quadtree gen_quadtree()
{
    // std::vector<Segment> segments = get_segments();

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

    // Read segments from a csv file
    begin = std::chrono::steady_clock::now();

    std::ifstream file("segments.csv");

    std::string line;

    // Skip the first line
    std::getline(file, line);
    int roadufi;
    double x1, y1, x2, y2;
    char delimiter;
    int i = 0;
    while (file >> roadufi >> delimiter >> x1 >> delimiter >> y1 >> delimiter >> x2 >> delimiter >> y2)
    {
        i++;
        if (i % 100000 == 0)
        {
            std::cout << "Batch: " << i / 100000 << std::endl;
        }
        Segment segment = {{x1, y1}, {x2, y2}, roadufi};
        quadtree.insert(segment);
    }

    end = std::chrono::steady_clock::now();

    std::cout << "Insert segments: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return quadtree;
}

// Example usage
int main()
{

    Quadtree quadtree = gen_quadtree();

#ifdef TEST_STOPS

    begin = std::chrono::steady_clock::now();
    std::unordered_map<int, std::pair<double, double>> stops = get_stops();
    end = std::chrono::steady_clock::now();
    std::cout << "Get stops: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;


    begin = std::chrono::steady_clock::now();

    std::ofstream file("stops_nearest_segment.csv");

    file << "stop_id,stop_lat,stop_lon,roadufi,segment_x1,segment_y1,segment_x2,segment_y2,distance_degree,distance_meter" << std::endl;

    int i = 0;
    for (const auto &[id, stop] : stops)
    {
        i++;
        Point p = {stop.first, stop.second};
        // std::cout << "Stop: " << i << " " << id << " (" << stop.first << ", " << stop.second << ")" << std::endl;
        double minDistance = std::numeric_limits<double>::infinity();
        // Segment nearestSegment = quadtree.nearestSegment(p, minDistance);
        auto [nearestSegment, quads] = quadtree.nearestSegment(p, minDistance);
        double distanceMeter = minDistance * 111139;
        file << id << "," << stop.first << "," << stop.second << "," << nearestSegment.roadufi << "," << nearestSegment.p1.x << "," << nearestSegment.p1.y << "," << nearestSegment.p2.x << "," << nearestSegment.p2.y << "," << minDistance << "," << distanceMeter << std::endl;
        if (nearestSegment.roadufi < 0)
        {
            std::cout << "Nearest segment to (" << stop.first << ", " << stop.second << ") "
                      << "is from (" << nearestSegment.p1.x << ", " << nearestSegment.p1.y << ") "
                      << "to (" << nearestSegment.p2.x << ", " << nearestSegment.p2.y << ") "
                      << "with distance: " << minDistance << " "
                      << "roadufi: " << nearestSegment.roadufi << std::endl;
        }
    }

    file.close();
    end = std::chrono::steady_clock::now();
    // std::cout << "Total stops: " << stops.size() << std::endl;
    std::cout << "Find nearest segment: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

#endif

    begin = std::chrono::steady_clock::now();

    // Point p = {144.866, -37.7512};
    Point p = {145.183, -37.9948};
    double minDistance = std::numeric_limits<double>::infinity();
    auto [nearestSegment, quads] = quadtree.nearestSegment(p, minDistance);
    end = std::chrono::steady_clock::now();
    std::cout << "Nearest segment to (" << p.x << ", " << p.y << ") "
              << "is from (" << nearestSegment.p1.x << ", " << nearestSegment.p1.y << ") "
              << "to (" << nearestSegment.p2.x << ", " << nearestSegment.p2.y << ") "
              << "with distance: " << minDistance << " "
              << "roadufi: " << nearestSegment.roadufi << std::endl;

    // Traverse in reverse
    for (auto it = quads.rbegin(); it != quads.rend(); ++it)
    {
        Quadtree *quad = *it;
        std::cout << "Quad: " << quad->getBoundary().x_min << " " << quad->getBoundary().y_min << " " << quad->getBoundary().x_max << " " << quad->getBoundary().y_max << " " << quad->getSegments().size() << " " << quad->isDivided() << " " << quad->segment_count << std::endl;
        if (quad->getSegments().size() > 0)
        {
            for (auto seg : quad->getSegments())
            {
                std::cout << "Segment: " << seg.p1.x << " " << seg.p1.y << " " << seg.p2.x << " " << seg.p2.y << std::endl;
            }
        }
    }

    std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    

    return 0;

    // docker exec -it searchc-dev-container-1 /bin/bash
    // g++ r-tree.cpp -o mytest $(pkg-config --cflags --libs libpqxx libpq) && ./mytest
}
