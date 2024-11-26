#include <chrono>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

#include <pqxx/pqxx>

#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/pool.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/cursor.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>

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

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/asio.hpp>

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <iostream>
#include <thread>

#include "db.h"

// Namespace alias for simplicity
namespace bg = boost::geometry;


#define TEST_STOPS
#define TEST_SINGLE_POINT

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

std::chrono::steady_clock::time_point begin, end;

// Point structure
struct Point
{
    double x, y;

    double distanceTo(const Point &other) const
    {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

struct Segment
{
    Point p1, p2;

    int roadufi;

    Point midpoint() const
    {
        return {(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
    }

    Point projectionPoint(const Point &p) const
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
        return {xx, yy};
    }

    Point nearestPoint(const Point &p) const
    {
        Point projection = projectionPoint(p);
        double x_min = std::min(p1.x, p2.x), x_max = std::max(p1.x, p2.x), y_min = std::min(p1.y, p2.y), y_max = std::max(p1.y, p2.y);
        bool ge_x_min = projection.x >= x_min;
        bool le_x_max = projection.x <= x_min;
        bool ge_y_min = projection.y >= y_min;
        bool le_y_max = projection.y <= y_min;
        // Check if the closest point is on the line segment
        if (ge_x_min && le_x_max && ge_y_min && le_y_max)
        {
            return projection;
        }
        else
        {
            if (p1.x == p2.x)
            {
                if (p1.y < p2.y)
                {
                    if (projection.y < p1.y)
                    {
                        return p1;
                    }
                    else
                    {
                        return p2;
                    }
                }
                else
                {
                    if (projection.y < p2.y)
                    {
                        return p1;
                    }
                    else
                    {
                        return p2;
                    }
                }
            }
            else if (p1.x < p2.x)
            {
                if (projection.x < p1.x)
                {
                    return p1;
                }
                else
                {
                    return p2;
                }
            }
            else
            {
                if (projection.x < p2.x)
                {
                    return p1;
                }
                else
                {
                    return p2;
                }
            }
        }
    }

    // Compute the distance from a point to the line segment
    double perpendicularDistanceToPoint(const Point &p) const
    {
        Point projection = projectionPoint(p);
        return p.distanceTo(projection);
    }

    double minDistanceToPoint(const Point &p) const
    {
        Point closest = nearestPoint(p);
        return p.distanceTo(closest);
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

struct Boundary
{
    double x_min, y_min, x_max, y_max;

    bool contains(const Point &p) const
    {
        return (p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max);
    }

    bool intersects_boundary(const Boundary &other) const
    {
        return !(other.x_min > x_max || other.x_max < x_min ||
                 other.y_min > y_max || other.y_max < y_min);
    }

    bool intersects_segment(const Segment &segment) const
    {
        if (contains(segment.p1) || contains(segment.p2))
            return true;

        return (
            segment.intersects(x_min, y_max, x_max, y_max, false) || segment.intersects(x_max, y_max, x_max, y_min, true) || segment.intersects(x_max, y_min, x_min, y_min, false) || segment.intersects(x_min, y_min, x_min, y_max, true));
    }

    double distance_min(const Point &p) const
    {
        bool ge_x_min = (p.x >= x_min);
        bool le_x_max = (p.x <= x_max);
        bool ge_y_min = (p.y >= y_min);
        bool le_y_max = (p.y <= y_max);

        if (ge_x_min && le_x_max && ge_y_min && le_y_max)
        {
            return 0.0;
        }
        if (ge_x_min && le_x_max && !ge_y_min)
        {
            return y_min - p.y;
        }
        if (ge_x_min && le_x_max && !le_y_max)
        {
            return p.y - y_max;
        }
        if (ge_y_min && le_y_max && !ge_x_min)
        {
            return x_min - p.x;
        }
        if (ge_y_min && le_y_max && !le_x_max)
        {
            return p.x - x_max;
        }

        if (!ge_x_min && !ge_y_min)
        {
            return sqrt((x_min - p.x) * (x_min - p.x) + (y_min - p.y) * (y_min - p.y));
        }
        if (!ge_x_min && !le_y_max)
        {
            return sqrt((x_min - p.x) * (x_min - p.x) + (p.y - y_max) * (p.y - y_max));
        }
        if (!le_x_max && !ge_y_min)
        {
            return sqrt((p.x - x_max) * (p.x - x_max) + (y_min - p.y) * (y_min - p.y));
        }
        if (!le_x_max && !le_y_max)
        {
            return sqrt((p.x - x_max) * (p.x - x_max) + (p.y - y_max) * (p.y - y_max));
        }
        return -1.0;
    }
};

// QuadNode node
class QuadNode
{
public:

    friend class QuadTree;
    // Bounding box for each quadrant
    QuadNode(Boundary boundary, int capacity = 4)
        : boundary(boundary), capacity(capacity), divided(false), segment_count(0) {}

    // Insert a segment
    bool insert(Segment segment)
    {
        if (!(boundary.intersects_segment(segment)))
            return false;

        if (!divided)
        {
            if ((segments.size() < capacity || (boundary.x_max - boundary.x_min) < 0.000001 || (boundary.y_max - boundary.y_min) < 0.000001))
            {
                segments.push_back(segment);
                segment_count++;
                return true;
            }
            else
            {
                subdivide();
            }
        }

        for (QuadNode *child : children)
        {
            if (child->insert(segment))
            {
                segment_count++;
                return true;
            }
        }
        return false;
    }

    std::tuple<Segment, Point, double, std::vector<QuadNode *>> find_nearest_segment(const Point &p)
    {
        double min_distance = std::numeric_limits<double>::max();
        Segment nearest_segment = {{0, 0}, {0, 0}, -1};
        Point nearest_point = {0, 0};
        auto quads = nearestSegment(p, min_distance, nearest_segment, nearest_point);
        return {nearest_segment, nearest_point, min_distance, quads};
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

    std::vector<QuadNode *> getChildren() const
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
    std::vector<QuadNode *> children;

    // Divide the current quadrant into 4
    void subdivide()
    {
        double x_mid = (boundary.x_min + boundary.x_max) / 2;
        double y_mid = (boundary.y_min + boundary.y_max) / 2;

        children.push_back(new QuadNode({boundary.x_min, boundary.y_min, x_mid, y_mid}, capacity));
        children.push_back(new QuadNode({boundary.x_min, y_mid, x_mid, boundary.y_max}, capacity));
        children.push_back(new QuadNode({x_mid, boundary.y_min, boundary.x_max, y_mid}, capacity));
        children.push_back(new QuadNode({x_mid, y_mid, boundary.x_max, boundary.y_max}, capacity));

        for (const Segment &seg : segments)
        {
            for (QuadNode *child : children)
            {
                child->insert(seg);
            }
        }

        // Clear the segments vector
        segments.clear();

        divided = true;
    }

    // Find the nearest segment to a point
    std::vector<QuadNode *> nearestSegment(const Point &point, double &minDistance, Segment &nearestSegment, Point &nearestPoint) const
    {
        if (divided)
        {
            std::vector<QuadNode *> container_quads;
            std::vector<QuadNode *> non_container_quads;
            for (QuadNode *child : children)
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
            for (QuadNode *child : container_quads)
            {
                if (child->segment_count == 0)
                {
                    continue;
                }
                else
                {
                    auto quad = child->nearestSegment(point, minDistance, nearestSegment, nearestPoint);
                    quad.push_back(const_cast<QuadNode *>(this));
                    return quad;
                }
            }
            std::vector<QuadNode *> quads;
            for (QuadNode *child : non_container_quads)
            {
                if (child->segment_count == 0)
                {
                    continue;
                }
                auto quad = child->nearestSegment(point, minDistance, nearestSegment, nearestPoint);
                quads = quad;
            }
            quads.push_back(const_cast<QuadNode *>(this));
            return quads;
        }

        else
        {
            std::vector<QuadNode *> quads;
            // Segment nearest;
            for (const Segment &segment : segments)
            {
                Point closest = segment.nearestPoint(point);
                double distance = point.distanceTo(closest);
                if (distance < minDistance)
                {
                    nearestPoint = closest;
                    minDistance = distance;
                    nearestSegment = segment;
                }
            }
            quads.push_back(const_cast<QuadNode *>(this));
            return quads;
        }
    }
};

class QuadTree
{
private:
    QuadNode *root;

    using FrontierElement = std::pair<QuadNode *, double>;

    struct CompareFrontierElement
    {
        bool operator()(const FrontierElement &q1, const FrontierElement &q2)
        {
            return q1.second > q2.second;
        }
    };

public:
    QuadTree() : root(nullptr) {}

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

    void gen_quadtree()
    {
        // std::vector<Segment> segments = get_segments();

        begin = std::chrono::steady_clock::now();
        auto [x_min, y_min, x_max, y_max] = get_min_max_coords();
        end = std::chrono::steady_clock::now();
        std::cout << "Get min max coords: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

        begin = std::chrono::steady_clock::now();
        Boundary boundary = {x_min, y_min, x_max, y_max};
        root = new QuadNode(boundary);
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
            root->insert(segment);
        }

        end = std::chrono::steady_clock::now();

        std::cout << "Insert segments: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    }

    std::tuple<Segment, Point, double, std::vector<QuadNode *>> find_nearest_segment(const Point &p)
    {
        double min_distance = std::numeric_limits<double>::max();
        Segment nearest_segment = {{0, 0}, {0, 0}, -1};
        Point min_point = {0, 0};

        std::priority_queue<FrontierElement, std::vector<FrontierElement>, CompareFrontierElement> frontier;

        frontier.push({root, 0.0});

        std::vector<QuadNode *> quads;
        while (!frontier.empty())
        {
            FrontierElement q = frontier.top();
            frontier.pop();
            QuadNode *quad = q.first;
            double distance = q.second;
            quads.push_back(quad);

            if (distance > min_distance)
            {
                break;
            }
            if (quad->segment_count == 0)
            {
                continue;
            }
            if (quad->isDivided())
            {
                for (QuadNode *child : quad->children)
                {
                    if (child->segment_count <= 0)
                    {
                        continue;
                    }
                    double child_boundary_distance = child->boundary.distance_min(p);
                    frontier.push({child, child_boundary_distance});
                }
            }
            else
            {
                for (const Segment &segment : quad->segments)
                {
                    Point closest = segment.nearestPoint(p);
                    distance = p.distanceTo(closest);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        nearest_segment = segment;
                        min_point = closest;
                    }
                }
            }
        }

        return {nearest_segment, min_point, min_distance, quads};
    }

    void test_quadtree()
    {
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
            // auto [nearestSegment, minDistance, quads] = quadtree.find_nearest_segment(p);
            auto [nearestSegment, closestPoint, minDistance, quads] = find_nearest_segment(p);
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

        Point p = {144.866, -37.7512};
        // Point p = {145.183, -37.9948};
        // Point p = {145.183, -37.9948};

        begin = std::chrono::steady_clock::now();

        auto [nearestSegment, closestPoint, minDistance, quads] = root->find_nearest_segment(p);
        end = std::chrono::steady_clock::now();
        std::cout << "Nearest segment to (" << std::setprecision(17) << p.x << ", " << std::setprecision(17) << p.y << ") " << "is from (" << std::setprecision(17) << nearestSegment.p1.x << ", " << std::setprecision(17) << nearestSegment.p1.y << ") " << "to (" << std::setprecision(17) << nearestSegment.p2.x << ", " << std::setprecision(17) << nearestSegment.p2.y << ") " << "with distance: " << std::setprecision(17) << minDistance << " " << "roadufi: " << nearestSegment.roadufi << std::endl;

        // Traverse in reverse
        for (auto it = quads.rbegin(); it != quads.rend(); ++it)
        {
            QuadNode *quad = *it;
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

        begin = std::chrono::steady_clock::now();

        std::tie(nearestSegment, closestPoint, minDistance, quads) = find_nearest_segment(p);
        end = std::chrono::steady_clock::now();
        std::cout << "Nearest segment to (" << std::setprecision(17) << p.x << ", " << std::setprecision(17) << p.y << ") " << "is from (" << std::setprecision(17) << nearestSegment.p1.x << ", " << std::setprecision(17) << nearestSegment.p1.y << ") " << "to (" << std::setprecision(17) << nearestSegment.p2.x << ", " << std::setprecision(17) << nearestSegment.p2.y << ") " << "with distance: " << std::setprecision(17) << minDistance << " " << "roadufi: " << nearestSegment.roadufi << std::endl;

        // Traverse in reverse
        for (auto it = quads.begin(); it != quads.end(); ++it)
        {
            QuadNode *quad = *it;
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
    }
};


class SearchGraph {

private:

    using BPoint = bg::model::point<double, 2, bg::cs::geographic<bg::degree>>;
    using PointUFI = int;
    using Coordinate = std::pair<double, double>;
    using RoadUFI = int;
    using RoadDirection = std::string;
    using RoadLength = double;
    using Neighbor = std::tuple<PointUFI, RoadUFI, RoadLength>;
    using NeighborList = std::vector<Neighbor>;
    using NeighborMap = std::map<PointUFI, NeighborList>;

    using PointMap = std::map<PointUFI, PointUFI>;

    #define START_POINT_UFI 0
    #define GOAL_POINT_UFI 1
    #define DIRECTION_FORWARD "F"
    #define DIRECTION_REVERSE "R"
    #define DIRECTION_BOTH "B"

    using NearestRoadInfo = std::tuple<RoadUFI, RoadDirection, double, double, PointUFI, PointUFI>;

    using Path = std::vector<RoadUFI>;
    using FrontierItem = std::tuple<RoadLength, RoadLength, PointUFI, Path>;

    std::map<PointUFI, Coordinate> points_coords;
    NeighborMap neighbors_map;

    
    void get_points() {
        // Initialize MongoDB instance and client

        mongocxx::collection points_collection = db["points"];

        // Vector to store documents from MongoDB
        std::vector<bsoncxx::document::value> points_coords_mongo_get;

        // Get the estimated document count
        auto collection_count = points_collection.estimated_document_count();

        // Iterate over the collection and append documents to the vector
        for (auto&& doc : points_collection.find({})) {
            points_coords_mongo_get.push_back(bsoncxx::document::value(doc));
        }

        // Convert BSON documents to the desired map structure
        for (const auto& doc : points_coords_mongo_get) {
            auto view = doc.view();
            PointUFI id = view["_id"].get_int32().value;
            bsoncxx::array::view coordinates_tuple = view["coords"].get_array().value;
            Coordinate coordinates = {coordinates_tuple[0].get_double(), coordinates_tuple[1].get_double()};

            points_coords[id] = coordinates;
        }

        // // Output the neighbors map for verification
        // for (const auto& [id, neighbor_list] : neighbors) {
        //     std::cout << "ID: " << id << "\n";
        //     for (const auto& [neighbor_id, neighbor_name, neighbor_distance] : neighbor_list) {
        //         std::cout << "  Neighbor ID: " << neighbor_id << ", Name: " << neighbor_name << ", Distance: " << neighbor_distance << "\n";
        //     }
        // }

        // return neighbors;
    }

    void get_neighbors() {
        // Initialize MongoDB instance and client

        mongocxx::collection neighbours_collection = db["points_neighbours"];

        // Vector to store documents from MongoDB
        std::vector<bsoncxx::document::value> neighbors_mongo;

        // // Get the estimated document count
        // auto collection_count = neighbours_collection.estimated_document_count();

        // Iterate over the collection and append documents to the vector
        for (auto&& doc : neighbours_collection.find({})) {
            neighbors_mongo.push_back(bsoncxx::document::value(doc));
        }

        // Convert BSON documents to the desired map structure
        for (const auto& doc : neighbors_mongo) {
            auto view = doc.view();
            int id = view["_id"].get_int32().value;
            std::vector<Neighbor> neighbor_list;

            for (const auto& neighbor : view["neighbours"].get_array().value) {
                auto neighbor_view = neighbor.get_array().value;
                PointUFI neighbor_id = neighbor_view[0].get_int32().value;
                RoadUFI road_id = neighbor_view[1].get_int32().value;
                RoadLength neighbor_distance = neighbor_view[2].get_double().value;
                neighbor_list.emplace_back(neighbor_id, road_id, neighbor_distance);
            }

            neighbors_map[id] = neighbor_list;
        }
    }

    // Function to calculate geodesic distance between two points (lon1, lat1) and (lon2, lat2)
    double geodesic_distance(double lat1, double lon1, double lat2, double lon2) {
        BPoint p1(lat1, lon1), p2(lat2, lon2);
        return bg::distance(p1, p2) * 1000.0;  // Convert to meters
    }

    double heuristic(PointUFI current, PointUFI goal) {
        auto [lon1, lat1] = points_coords[current];
        auto [lon2, lat2] = points_coords[goal];
        return geodesic_distance(lat1, lon1, lat2, lon2);
    }


    std::pair<NeighborMap, PointMap> gen_extra_info(NearestRoadInfo start_road_info, NearestRoadInfo goal_road_info) {
        
        auto [start_road_ufi, start_road_direction, start_road_px, start_road_py, start_from_ufi, start_to_ufi] = start_road_info;
        auto [goal_road_ufi, goal_road_direction, goal_road_px, goal_road_py, goal_from_ufi, goal_to_ufi] = goal_road_info;


        NeighborMap special_neighbors;

        points_coords[START_POINT_UFI] = {start_road_px, start_road_py};
        points_coords[GOAL_POINT_UFI] = {goal_road_px, goal_road_py};

        double start_from_ufi_distance = geodesic_distance(start_road_py, start_road_px, points_coords[start_from_ufi].second, points_coords[start_from_ufi].first);
        double start_to_ufi_distance = geodesic_distance(start_road_py, start_road_px, points_coords[start_to_ufi].second, points_coords[start_to_ufi].first);
        double goal_from_ufi_distance = geodesic_distance(goal_road_py, goal_road_px, points_coords[goal_from_ufi].second, points_coords[goal_from_ufi].first);
        double goal_to_ufi_distance = geodesic_distance(goal_road_py, goal_road_px, points_coords[goal_to_ufi].second, points_coords[goal_to_ufi].first);

        special_neighbors[start_from_ufi] = {};
        special_neighbors[start_to_ufi] = {};
        special_neighbors[goal_from_ufi] = {};
        special_neighbors[goal_to_ufi] = {};
        special_neighbors[START_POINT_UFI] = {};
        special_neighbors[GOAL_POINT_UFI] = {};

        if (start_road_direction == DIRECTION_FORWARD || start_road_direction == DIRECTION_BOTH) {
            special_neighbors[start_from_ufi].push_back({START_POINT_UFI, start_road_ufi, start_from_ufi_distance});
            special_neighbors[START_POINT_UFI].push_back({start_to_ufi, start_road_ufi, start_to_ufi_distance});
        }
        if (start_road_direction == DIRECTION_REVERSE || start_road_direction == DIRECTION_BOTH) {
            special_neighbors[start_to_ufi].push_back({START_POINT_UFI, start_road_ufi, start_to_ufi_distance});
            special_neighbors[START_POINT_UFI].push_back({start_from_ufi, start_road_ufi, start_from_ufi_distance});
        }
        if (goal_road_direction == DIRECTION_FORWARD || goal_road_direction == DIRECTION_BOTH) {
            special_neighbors[goal_from_ufi].push_back({GOAL_POINT_UFI, goal_road_ufi, goal_from_ufi_distance});
            special_neighbors[GOAL_POINT_UFI].push_back({goal_to_ufi, goal_road_ufi, goal_to_ufi_distance});
        }
        if (goal_road_direction == DIRECTION_REVERSE || goal_road_direction == DIRECTION_BOTH) {
            special_neighbors[goal_to_ufi].push_back({GOAL_POINT_UFI, goal_road_ufi, goal_to_ufi_distance});
            special_neighbors[GOAL_POINT_UFI].push_back({goal_from_ufi, goal_road_ufi, goal_from_ufi_distance});
        }

        PointMap skip_neighbors = {
            {start_from_ufi, start_to_ufi},
            {start_to_ufi, start_from_ufi},
            {goal_from_ufi, goal_to_ufi},
            {goal_to_ufi, goal_from_ufi},
        };

        return {special_neighbors, skip_neighbors};
    }


    std::pair<Path, RoadLength> astar(PointUFI start, PointUFI goal, NeighborMap special_neighbors, PointMap skip_neighbors) {
        // std::vector<FrontierItem> frontier;
        std::priority_queue<FrontierItem, std::vector<FrontierItem>, std::greater<FrontierItem>> frontier;
        std::set<PointUFI> visited;
        Path path;

        // frontier.emplace_back(START_POINT_UFI, 0, start, path);
        frontier.emplace(0, 0, start, path);

        while (!frontier.empty()) {
            // std::sort(frontier.begin(), frontier.end());
            auto [_, cost, current, current_path] = frontier.top();
            frontier.pop();
            // frontier.erase(frontier.begin());

            if (current == goal) {
                return {current_path, cost};
            }

            if (current == 1) {
                return {path, cost};
            }

            if (visited.find(current) != visited.end()) {
                continue;
            }
            
            visited.insert(current);

            NeighborList neighbor_points = {};
            if (neighbors_map.find(current) != neighbors_map.end()) {
                for (const auto& p : neighbors_map[current]) {
                    neighbor_points.push_back(p);
                }
            }
            if (special_neighbors.find(current) != neighbors_map.end()) {
                for (const auto& p : special_neighbors[current]) {
                    neighbor_points.push_back(p);
                }
            }

            for (const auto& [neighbor_point, road_ufi, road_length] : neighbor_points) {
                if (skip_neighbors.find(current) != skip_neighbors.end() && skip_neighbors[current] == neighbor_point) {
                    continue;
                }
                
                if (visited.find(neighbor_point) == visited.end()) {
                    double heuristic_cost = heuristic(neighbor_point, goal);
                    std::vector<int> new_path = current_path;
                    new_path.push_back(road_ufi);
                    // frontier.emplace(cost + road_length + heuristic_cost, cost + road_length, neighbor_point, new_path);
                    frontier.push({cost + road_length + heuristic_cost, cost + road_length, neighbor_point, new_path});
                }
            }
        }

        return {path, 0.0};
    }


public:

    QuadTree quadtree;

    SearchGraph() {}

    std::vector<NearestRoadInfo> find_nearest_road(double lon, double lat) {

        Point p = {lon, lat};

        auto [nearestSegment, closestPoint, minDistance, quads] = quadtree.find_nearest_segment(p);

        RoadUFI roadufi = nearestSegment.roadufi;

        pqxx::work txn(conn);

        std::string query = "SELECT ufi, direction_code, from_ufi, to_ufi FROM vmtrans.tr_road_all WHERE ufi = " + std::to_string(roadufi) + " LIMIT 1;";

        pqxx::result result = txn.exec(query);
        std::vector<NearestRoadInfo> roads;

        for (auto row : result) {
            roads.emplace_back(    
                row[0].as<RoadUFI>(), 
                row[1].as<RoadDirection>(), 
                closestPoint.x, 
                closestPoint.y, 
                row[2].as<PointUFI>(), 
                row[3].as<PointUFI>()
            );
        }

        return roads;
    }



    std::pair<Path, RoadLength> search_path(double lon1, double lat1, double lon2, double lat2) {

        NearestRoadInfo start_road_info = find_nearest_road(lon1, lat1).at(0);
        auto [start_road_ufi, start_road_direction, start_road_px, start_road_py, start_from_ufi, start_to_ufi] = start_road_info;

        std::cout << start_road_ufi << " " << start_from_ufi << " " << start_to_ufi << std::endl;

        NearestRoadInfo goal_road_info = find_nearest_road(lon2, lat2).at(0);
        auto [goal_road_ufi, goal_road_direction, goal_road_px, goal_road_py, goal_from_ufi, goal_to_ufi] = goal_road_info;

        std::cout << goal_road_ufi << " " << goal_from_ufi << " " << goal_to_ufi << std::endl;

        if (start_road_ufi == goal_road_ufi) {
            return {{start_road_ufi}, 0.0};
        }

        auto [special_neighbors, skip_neighbors] = gen_extra_info(start_road_info, goal_road_info);

        return astar(START_POINT_UFI, GOAL_POINT_UFI, special_neighbors, skip_neighbors);  // Assuming start is 0 and goal is 1
    }


    void build() {
        // Populate the points_coords and neighbors collections from MongoDB (similar to Python code)
        // Example usage:
        std::chrono::steady_clock::time_point begin, end;
        begin = std::chrono::steady_clock::now();
        get_neighbors();
        end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
        // Calculate size on RAM
        std::cout << "Size of neighbors_map: " << sizeof(neighbors_map) << " bytes" << std::endl;
        begin = std::chrono::steady_clock::now();
        get_points();
        end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
        // Calculate size on RAM
        std::cout << "Size of points_coords: " << sizeof(points_coords) << " bytes" << std::endl;

        quadtree.gen_quadtree();
    }
};

void test_searchgraph() {

    SearchGraph searchgraph = SearchGraph();

    searchgraph.build();
    
    begin = std::chrono::steady_clock::now();    
    double lon1 = 144.9631, lat1 = -37.8136; // Melbourne
    double lon2 = 145.0458, lat2 = -37.8768; // Nearby suburb
    auto [path, cost] = searchgraph.search_path(lon1, lat1, lon2, lat2);
    std::cout << "Path: ";
    for (const auto& p : path) std::cout << p << " ";
    std::cout << "\nTotal Cost: " << cost << std::endl;
    end = std::chrono::steady_clock::now();
    std::cout << "Find shortest path = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    // Sample 10 points and find the nearest roads of each
    std::vector<double> sample_lons = {
        144.9631, 145.0458, 144.9731, 144.9831, 144.9931, 145.0031, 145.0131, 145.0231, 145.0331, 145.0431
    };
    std::vector<double> sample_lats = {
        -37.8136, -37.8768, -37.8236, -37.8336, -37.8436, -37.8536, -37.8636, -37.8736, -37.8836, -37.8936
    };
    for (int i = 0; i < 10; i++) {
        double lon = sample_lons[i], lat = sample_lats[i];
        auto roads = searchgraph.find_nearest_road(lon, lat);
        std::cout << "Point " << i << " (" << lon << ", " << lat << "): ";
        for (const auto& [road_ufi, direction, px, py, from_ufi, to_ufi] : roads) {
            std::cout << road_ufi << " ";
        }
        std::cout << std::endl;
    }
    // g++ main.cpp -o test $(pkg-config --cflags --libs libpqxx libpq libmongocxx-static) -I/vcpkg/installed/x64-linux/include -L/vcpkg/installed/x64-linux/lib
}



// Server Class
class Server
{
public:
    Server(const std::string& address, unsigned short port)
        : endpoint_(net::ip::make_address(address), port), acceptor_(ioc_, endpoint_), searchgraph_()
    {
        searchgraph_.build();
    }

    // Run the server
    void run()
    {
        std::cout << "Server is running on http://" << endpoint_.address() << ":" << endpoint_.port() << std::endl;

        // Main server loop
        for (;;)
        {
            tcp::socket socket = acceptor_.accept();
            std::thread(&Server::do_session, this, std::move(socket)).detach();
        }
    }

private:
    std::map<std::string, double> parse_query(const std::string_view& query) {
        std::map<std::string, double> params;
        std::string query_str = std::string(query);
        std::istringstream iss(query_str);
        std::string token;

        while (std::getline(iss, token, '&')) {
            auto pos = token.find('=');
            if (pos != std::string::npos) {
                std::string key = token.substr(0, pos);
                std::string value = token.substr(pos + 1);
                try {
                    params[key] = std::stod(value);
                } catch (...) {
                    return params;
                }
            }
        }
        return params;
    }

    // Function to parse query parameters from the URL
    std::optional<std::pair<double, double>> parse_query_xy(const std::string_view& query) {
        double x = 0.0, y = 0.0;
        std::string query_str = std::string(query);
        std::istringstream iss(query_str);
        std::string token;

        while (std::getline(iss, token, '&')) {
            auto pos = token.find('=');
            if (pos != std::string::npos) {
                std::string key = token.substr(0, pos);
                std::string value = token.substr(pos + 1);
                try {
                    if (key == "x") x = std::stod(value);
                    if (key == "y") y = std::stod(value);
                } catch (...) {
                    return std::nullopt; // Handle invalid input
                }
            }
        }
        return {{x, y}};
    }

    // HTTP request handler
    void handle_request(const http::request<http::string_body>& req, http::response<http::string_body>& res)
    {
        if (req.method() == http::verb::get && req.target() == "/")
        {
            res.result(http::status::ok);
            res.set(http::field::content_type, "text/plain");
            res.body() = "QuadNode summary stub";
        }
        else if (req.method() == http::verb::get && req.target().starts_with("/nearestsegment"))
        {
            // Extract query string
            std::string_view target = req.target();
            auto pos = target.find('?');
            if (pos == std::string::npos)
            {
                res.result(http::status::bad_request);
                res.body() = "Missing query parameters.";
                res.prepare_payload();
                return;
            }

            std::string_view query = target.substr(pos + 1);
            auto params = parse_query(query);
            double x = params["x"], y = params["y"];

            Point p = {x, y};
            auto [nearestSegment, closestPoint, minDistance, quads] = searchgraph_.quadtree.find_nearest_segment(p);
            res.result(http::status::ok);
            res.set(http::field::content_type, "application/json");
            res.body() = "{\"roadufi\": " + std::to_string(nearestSegment.roadufi) + ", \"distance\": " + std::to_string(minDistance) + ", \"x1\": " + std::to_string(nearestSegment.p1.x) + ", \"y1\": " + std::to_string(nearestSegment.p1.y) + ", \"x2\": " + std::to_string(nearestSegment.p2.x) + ", \"y2\": " + std::to_string(nearestSegment.p2.y) + ", \"px\": " + std::to_string(closestPoint.x) + ", \"py\": " + std::to_string(closestPoint.y) + "}";
        }
        else if (req.method() == http::verb::get && req.target().starts_with("/searchpath"))
        {
            // Extract query string
            std::string_view target = req.target();
            auto pos = target.find('?');
            if (pos == std::string::npos)
            {
                res.result(http::status::bad_request);
                res.body() = "Missing query parameters.";
                res.prepare_payload();
                return;
            }

            std::string_view query = target.substr(pos + 1);
            auto params = parse_query(query);

            double x1 = params["x1"], y1 = params["y1"], x2 = params["x2"], y2 = params["y2"];

            auto [path, cost] = searchgraph_.search_path(x1, y1, x2, y2);
            res.result(http::status::ok);
            res.set(http::field::content_type, "application/json");
            std::string path_str = "[";
            for (int i = 0; i < path.size(); i++)
            {
                if (i > 0) path_str += ", ";
                path_str += std::to_string(path[i]);
            }
            path_str += "]";
            res.body() = "{\"path\": " + path_str + ", \"cost\": " + std::to_string(cost) + "}";
        }
        else
        {
            res.result(http::status::not_found);
            res.set(http::field::content_type, "text/plain");
            res.body() = "404 Not Found";
        }
        res.prepare_payload();
    }

    // Session handler
    void do_session(tcp::socket socket)
    {
        try
        {
            beast::flat_buffer buffer;

            // Read the HTTP request
            http::request<http::string_body> req;
            http::read(socket, buffer, req);

            // Prepare HTTP response
            http::response<http::string_body> res;

            // Handle the request
            handle_request(req, res);

            // Write the HTTP response back to the client
            http::write(socket, res);
        }
        catch (std::exception& e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

    // Member variables
    net::io_context ioc_;
    tcp::endpoint endpoint_;
    tcp::acceptor acceptor_;
    SearchGraph searchgraph_;
};

int main()
{
    const std::string address = "0.0.0.0";
    const unsigned short port = 8080;

    try
    {
        Server server(address, port);
        server.run();
    }
    catch (std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

// docker exec -it searchc-dev-container-1 /bin/bash
// g++ search-path-server.cpp -o local-server.exe $(pkg-config --cflags --libs libpqxx libpq libmongocxx-static) -I/vcpkg/installed/x64-linux/include -L/vcpkg/installed/x64-linux/lib && ./local-server.exe
// http://localhost:8080/nearestsegment?x=144.866&y=-37.751275
// http://localhost:8080/searchpath?x1=144.9631&y1=-37.8136&x2=145.0458&y2=-37.8768