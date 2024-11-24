#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <stack>
#include <queue>
#include <iomanip>
#include <chrono>
#include <iostream>
#include <pqxx/pqxx>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>

#define TEST_STOPS
#define TEST_SINGLE_POINT
// #define USE_MIDPOINT_COUNT

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
    double perpendicularDistanceToPoint(const Point &p) const
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
        : boundary(boundary), capacity(capacity), divided(false), segment_count(0) {}

    // Insert a segment
    bool insert(Segment segment)
    {
        if (!(boundary.intersects(segment)))
            return false;

        if (!divided)
        {
            if ((segments.size() < capacity || (boundary.x_max - boundary.x_min) < 0.000001 || (boundary.y_max - boundary.y_min) < 0.000001))
            {
                segments.push_back(segment);
#ifdef USE_MIDPOINT_COUNT
                if (boundary.contains(segment.midpoint()))
                {
                    midpoint_count++;
                }
#endif
                segment_count++;
                return true;
            }
            else
            {
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

    std::tuple<Segment, double, std::vector<Quadtree *>> find_nearest_segment(const Point &p)
    {
        double min_distance = std::numeric_limits<double>::max();
        Segment nearest_segment = {{0, 0}, {0, 0}, -1};
        auto quads = nearestSegmentOld(p, min_distance, nearest_segment);
        return {nearest_segment, min_distance, quads};
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

    int getSegmentCount() const
    {
        return segment_count;
    }

private:
    Boundary boundary;
    int capacity;
    bool divided;
#ifdef USE_MIDPOINT_COUNT
    int midpoint_count;
#endif
    int segment_count;
    std::vector<Segment> segments;

    // Quadrants
    std::vector<Quadtree *> children;

    // Divide the current quadrant into 4
    void subdivide()
    {
        double x_mid = (boundary.x_min + boundary.x_max) / 2;
        double y_mid = (boundary.y_min + boundary.y_max) / 2;

        children.push_back(new Quadtree({boundary.x_min, boundary.y_min, x_mid, y_mid}, capacity));
        children.push_back(new Quadtree({boundary.x_min, y_mid, x_mid, boundary.y_max}, capacity));
        children.push_back(new Quadtree({x_mid, boundary.y_min, boundary.x_max, y_mid}, capacity));
        children.push_back(new Quadtree({x_mid, y_mid, boundary.x_max, boundary.y_max}, capacity));

        for (const Segment &seg : segments)
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

    // Find the nearest segment to a point
    std::vector<Quadtree *> nearestSegmentOld(const Point &point, double &minDistance, Segment &nearestSegment) const
    {
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
                    auto quad = child->nearestSegmentOld(point, minDistance, nearestSegment);
                    quad.push_back(const_cast<Quadtree *>(this));
                    return quad;
                }
            }
            std::vector<Quadtree *> quads;
            for (Quadtree *child : non_container_quads)
            {
                if (child->segment_count == 0)
                {
                    continue;
                }
                auto quad = child->nearestSegmentOld(point, minDistance, nearestSegment);
                quads = quad;
            }
            quads.push_back(const_cast<Quadtree *>(this));
            return quads;
        }

        else
        {
            std::vector<Quadtree *> quads;
            // Segment nearest;
            for (const Segment &segment : segments)
            {
                double distance = segment.perpendicularDistanceToPoint(point);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestSegment = segment;
                }
            }
            quads.push_back(const_cast<Quadtree *>(this));
            return quads;
        }
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

    std::ifstream file("../local/segments.csv");

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
        if (i % 108000 == 0)
        {
            std::cout << "Batch: " << i / 108000 << std::endl;
        }
        Segment segment = {{x1, y1}, {x2, y2}, roadufi};
        quadtree.insert(segment);
    }

    end = std::chrono::steady_clock::now();

    std::cout << "Insert segments: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return quadtree;
}

void store_quadtree(Quadtree *quadtree) {
    begin = std::chrono::steady_clock::now();

    std::ofstream quadfile("../local/quadtrees.csv");
    quadfile << "quadid,x_min,y_min,x_max,y_max,x_mid,y_mid,divided,segments_count,quad0,quad1,quad2,quad3" << std::endl;

    std::ofstream segmentsfile("../local/quadsegments.csv");
    segmentsfile << "quadid,roadufi,x1,y1,x2,y2" << std::endl;
    
    int quadcount = 0; 
    std::stack<std::tuple<Quadtree *, std::string>> frontier;
    frontier.push(std::make_tuple(quadtree, "0"));
    while (!frontier.empty())
    {
        auto [quad, quadid] = frontier.top();
        quadcount++;
        frontier.pop();
        if (quadcount % 130251 == 0)
        {
            std::cout << "Batch: " << quadcount / 130251 << std::endl;
        }
        double x_mid = (quad->getBoundary().x_min + quad->getBoundary().x_max) / 2;
        double y_mid = (quad->getBoundary().y_min + quad->getBoundary().y_max) / 2;
        quadfile << quadid << ',' << std::setprecision(17) << quad->getBoundary().x_min << ',' << std::setprecision(17) << quad->getBoundary().y_min << ',' << std::setprecision(17) << quad->getBoundary().x_max << ',' << std::setprecision(17) << quad->getBoundary().y_max << ',' << std::setprecision(17) << x_mid << ',' << std::setprecision(17) << y_mid << ',' << quad->isDivided() << ',' << quad->getSegmentCount();
        if (quad->isDivided())
        {
            int i = 0;
            for (Quadtree *child_quad : quad->getChildren())
            {
                frontier.push(std::make_tuple(child_quad, quadid + std::to_string(i)));
                quadfile << ',' << child_quad->getSegmentCount();
                i++;
            }
        }
        else 
        {
            for (int i = 0; i < 4; i++)
            {
                quadfile << ',' << 0;
            }
        }
        quadfile << std::endl;
        for (const Segment &seg : quad->getSegments())
        {
            segmentsfile << quadid << ',' << seg.roadufi << ',' << std::setprecision(17) << seg.p1.x << ',' << std::setprecision(17) << seg.p1.y << ',' << std::setprecision(17) << seg.p2.x << ',' << std::setprecision(17) << seg.p2.y << std::endl;
        }
    }
    quadfile.close();
    segmentsfile.close();

    end = std::chrono::steady_clock::now();

    std::cout << "Store tree: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
} 

// Example usage
int main()
{

    Quadtree quadtree = gen_quadtree();

    store_quadtree(&quadtree);

#ifdef TEST_STOPS

    begin = std::chrono::steady_clock::now();
    std::unordered_map<int, std::pair<double, double>> stops = get_stops();
    end = std::chrono::steady_clock::now();
    std::cout << "Get stops: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    std::ofstream file("../local/stops_nearest_segment.csv");

    file << "stop_id,stop_lat,stop_lon,roadufi,segment_x1,segment_y1,segment_x2,segment_y2,distance_degree,distance_meter" << std::endl;

    int i = 0;
    for (const auto &[id, stop] : stops)
    {
        i++;
        Point p = {stop.first, stop.second};
        // std::cout << "Stop: " << i << " " << id << " (" << stop.first << ", " << stop.second << ")" << std::endl;
        auto [nearestSegment, minDistance, quads] = quadtree.find_nearest_segment(p);
        double distanceMeter = minDistance * 111139;
        file << id << "," << std::setprecision(17) << stop.first << "," << std::setprecision(17) << stop.second << "," << std::setprecision(17) << stop.second << "," << nearestSegment.roadufi << "," << std::setprecision(17) << nearestSegment.p1.x << "," << std::setprecision(17) << nearestSegment.p1.y << "," << std::setprecision(17) << nearestSegment.p2.x << "," << std::setprecision(17) << nearestSegment.p2.y << "," << std::setprecision(17) << minDistance << "," << std::setprecision(17) << distanceMeter << std::endl;
        if (nearestSegment.roadufi < 0)
        {
            std::cout << "Nearest segment to (" << stop.first << ", " << stop.second << ") " << "is from (" << std::setprecision(17) << nearestSegment.p1.x << ", " << std::setprecision(17) << nearestSegment.p1.y << ") " << "to (" << std::setprecision(17) << nearestSegment.p2.x << ", " << std::setprecision(17) << nearestSegment.p2.y << ") " << "with distance: " << std::setprecision(17) << minDistance << " " << "roadufi: " << nearestSegment.roadufi << std::endl;
        }
    }

    file.close();
    end = std::chrono::steady_clock::now();
    // std::cout << "Total stops: " << stops.size() << std::endl;
    std::cout << "Find stops' nearest segments: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

#endif

#ifdef TEST_SINGLE_POINT

    begin = std::chrono::steady_clock::now();

    Point p = {144.866, -37.7512};
    // Point p = {145.183, -37.9948};
    auto [nearestSegment, minDistance, quads] = quadtree.find_nearest_segment(p);
    end = std::chrono::steady_clock::now();
    std::cout << "Nearest segment to (" << std::setprecision(17) << p.x << ", " << std::setprecision(17) << p.y << ") " << "is from (" << std::setprecision(17) << nearestSegment.p1.x << ", " << std::setprecision(17) << nearestSegment.p1.y << ") " << "to (" << std::setprecision(17) << nearestSegment.p2.x << ", " << std::setprecision(17) << nearestSegment.p2.y << ") " << "with distance: " << std::setprecision(17) << minDistance << " " << "roadufi: " << nearestSegment.roadufi << std::endl;

    // Traverse in reverse
    for (auto it = quads.rbegin(); it != quads.rend(); ++it)
    {
        Quadtree *quad = *it;
        std::cout << "Quad: " << std::setprecision(17) << quad->getBoundary().x_min << " " << std::setprecision(17) << quad->getBoundary().y_min << " " << std::setprecision(17) << quad->getBoundary().x_max << " " << std::setprecision(17) << quad->getBoundary().y_max << " " << quad->getSegments().size() << " " << quad->isDivided() << " " << quad->getSegmentCount() << std::endl;
        if (quad->getSegments().size() > 0)
        {
            for (auto seg : quad->getSegments())
            {
                std::cout << "Segment: " << seg.p1.x << " " << seg.p1.y << " " << seg.p2.x << " " << seg.p2.y << std::endl;
            }
        }
    }

    std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

#endif

    return 0;

    // docker exec -it searchc-dev-container-1 /bin/bash
    // g++ q-tree-gensql.cpp -o local-exe $(pkg-config --cflags --libs libpqxx libpq) && ./local-exe
}
