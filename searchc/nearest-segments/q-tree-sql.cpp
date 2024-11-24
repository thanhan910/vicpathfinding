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

// #define TEST_STOPS
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

struct Boundary
{
    double x_min, y_min, x_max, y_max;

    bool contains(const double &x, const double &y) const
    {
        return (x >= x_min && x <= x_max && y >= y_min && y <= y_max);
    }

    bool distance_min(double x, double y) const
    {
        bool ge_x_min = (x >= x_min);
        bool le_x_max = (x <= x_max);
        bool ge_y_min = (y >= y_min);
        bool le_y_max = (y <= y_max);

        if (ge_x_min && le_x_max && ge_y_min && le_y_max)
        {
            return 0.0;
        }
        if (ge_x_min && le_x_max && !ge_y_min)
        {
            return y_min - y;
        }
        if (ge_x_min && le_x_max && !le_y_max)
        {
            return y - y_max;
        }
        if (ge_y_min && le_y_max && !ge_x_min)
        {
            return x_min - x;
        }
        if (ge_y_min && le_y_max && !le_x_max)
        {
            return x - x_max;
        }

        if (!ge_x_min && !ge_y_min)
        {
            return sqrt((x_min - x) * (x_min - x) + (y_min - y) * (y_min - y));
        }
        if (!ge_x_min && !le_y_max)
        {
            return sqrt((x_min - x) * (x_min - x) + (y - y_max) * (y - y_max));
        }
        if (!le_x_max && !ge_y_min)
        {
            return sqrt((x - x_max) * (x - x_max) + (y_min - y) * (y_min - y));
        }
        if (!le_x_max && !le_y_max)
        {
            return sqrt((x - x_max) * (x - x_max) + (y - y_max) * (y - y_max));
        }
        return -1.0;
    }
};

// Quadtree node
struct Quadtree
{
    std::string quadid;
    Boundary boundary;
    double x_mid, y_mid;
    bool divided;
    int segment_count;
    std::vector<int> child_segment_counts;
};

Quadtree get_quadtree_from_sql_table(std::string quadid)
{
    std::string query = "SELECT * FROM vmtrans.quadtrees WHERE quadid = '" + quadid + "' LIMIT 1";
    pqxx::work txn(conn);
    pqxx::result result = txn.exec(query);
    if (result.size() == 0)
    {
        return {quadid, {0, 0, 0, 0}, 0, 0, false, -1, {0, 0, 0, 0}};
    }

    // std::string quadid = result[0][0].as<std::string>();
    double x_min = result[0][1].as<double>();
    double y_min = result[0][2].as<double>();
    double x_max = result[0][3].as<double>();
    double y_max = result[0][4].as<double>();
    double x_mid = result[0][5].as<double>();
    double y_mid = result[0][6].as<double>();
    bool divided = result[0][7].as<bool>();
    int segment_count = result[0][8].as<int>();
    int quad0_segment_count = result[0][9].as<int>();
    int quad1_segment_count = result[0][10].as<int>();
    int quad2_segment_count = result[0][11].as<int>();
    int quad3_segment_count = result[0][12].as<int>();

    return {quadid, {x_min, y_min, x_max, y_max}, x_mid, y_mid, divided, segment_count, {quad0_segment_count, quad1_segment_count, quad2_segment_count, quad3_segment_count}};
}

std::vector<Segment> get_segments_from_sql_table(std::string quadid)
{
    std::string query = "SELECT * FROM vmtrans.quadsegments WHERE quadid = '" + quadid + "'";
    pqxx::work txn(conn);
    pqxx::result result = txn.exec(query);

    std::vector<Segment> segments;

    for (auto row : result)
    {
        int roadufi = row[1].as<int>();
        double x1 = row[2].as<double>();
        double y1 = row[3].as<double>();
        double x2 = row[4].as<double>();
        double y2 = row[5].as<double>();
        Point p1 = {x1, y1};
        Point p2 = {x2, y2};
        Segment segment = {p1, p2, roadufi};
        segments.push_back(segment);
    }

    return segments;
}

// implement QuadtreeComparator

using FrontierElement = std::pair<std::string, double>;

struct QuadtreeComparator
{
    bool operator()(const FrontierElement &q1, const FrontierElement &q2)
    {
        return q1.second > q2.second;
    }
};

std::pair<Segment, double> find_nearest_segment(Point p)
{

    double min_distance = std::numeric_limits<double>::max();
    Segment nearest_segment = {{0, 0}, {0, 0}, -1};

    std::priority_queue<FrontierElement, std::vector<FrontierElement>, QuadtreeComparator> frontier;
    frontier.push({"0", 0});

    while (!frontier.empty())
    {
        FrontierElement q = frontier.top();
        frontier.pop();
        std::string quadid = q.first;
        double distance = q.second;
        Quadtree quadtree = get_quadtree_from_sql_table(quadid);
        if (distance > min_distance)
        {
            break;
        }
        if (quadtree.segment_count == 0)
        {
            continue;
        }
        if (quadtree.divided)
        {
            for (int i = 0; i < 4; i++)
            {
                if (quadtree.child_segment_counts[i] <= 0)
                {
                    continue;
                }
                std::string child_quadid = quadtree.quadid + std::to_string(i);
                Boundary child_boundary;
                switch (i)
                {
                case 0:
                    child_boundary = {quadtree.boundary.x_min, quadtree.boundary.y_min, quadtree.x_mid, quadtree.y_mid};
                    break;

                case 1:
                    child_boundary = {quadtree.boundary.x_min, quadtree.y_mid, quadtree.x_mid, quadtree.boundary.y_max};
                    break;

                case 2:
                    child_boundary = {quadtree.x_mid, quadtree.boundary.y_min, quadtree.boundary.x_max, quadtree.y_mid};
                    break;

                case 3:
                    child_boundary = {quadtree.x_mid, quadtree.y_mid, quadtree.boundary.x_max, quadtree.boundary.y_max};
                    break;

                default:
                    break;
                }
                double child_distance = child_boundary.distance_min(p.x, p.y);
                frontier.push({child_quadid, child_distance});
            }
            continue;
        }
        else
        {
            std::vector<Segment> segments = get_segments_from_sql_table(quadtree.quadid);
            for (Segment segment : segments)
            {
                double segment_distance = segment.perpendicularDistanceToPoint(p);
                if (segment_distance < min_distance)
                {
                    min_distance = segment_distance;
                    nearest_segment = segment;
                }
            }
        }
    }

    return {nearest_segment, min_distance};
}

struct QuadData
{
    std::string quadid;
    Boundary boundary;
    bool divided;
    int segment_count;
};

class QuadNode
{
public:
    QuadData data;
    std::vector<QuadNode *> children;
    std::vector<Segment> segments;

    QuadNode(QuadData data) : data(data) {}
};

QuadNode *build_quadtree()
{

    std::string query = "SELECT quadid, x_min, y_min, x_max, y_max, divided, segments_count FROM vmtrans.quadtrees ORDER BY LENGTH(quadid) DESC, quadid ASC";
    std::cout << "SELECT * SQL query: " << query << std::endl;
    begin = std::chrono::steady_clock::now();
    pqxx::work txn(conn);
    pqxx::result result = txn.exec(query);
    txn.commit();
    end = std::chrono::steady_clock::now();
    std::cout << "SELECT * SQL Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    std::unordered_map<std::string, QuadNode *> quadtree_map;
    int quadcount = 0;
    int batchsize = 130251;
    for (auto row : result)
    {
        std::string quadid = row[0].as<std::string>();
        double x_min = row[1].as<double>();
        double y_min = row[2].as<double>();
        double x_max = row[3].as<double>();
        double y_max = row[4].as<double>();
        bool divided = row[5].as<bool>();
        int segment_count = row[6].as<int>();
        Boundary boundary = {x_min, y_min, x_max, y_max};
        QuadData data = {quadid, boundary, divided, segment_count};
        if (!divided)
        {
            quadtree_map[quadid] = new QuadNode(data);
        }
        else
        {
            QuadNode *node = new QuadNode(data);
            for (int i = 0; i < 4; i++)
            {
                std::string child_quadid = quadid + std::to_string(i);
                node->children.push_back(quadtree_map[child_quadid]);
                // Remove quadtree_map[child_quadid]
                // quadtree_map.erase(child_quadid);
            }
            quadtree_map[quadid] = node;
        }

        quadcount++;
        if (quadcount % batchsize == 0)
        {
            std::cout << "Batch: " << quadcount / batchsize << std::endl;
        }
    }

    query = "SELECT * FROM vmtrans.quadsegments";
    std::cout << "SELECT * quadsegements: " << query << std::endl;
    begin = std::chrono::steady_clock::now();
    pqxx::work txn2(conn);
    result = txn2.exec(query);
    txn2.commit();
    end = std::chrono::steady_clock::now();
    std::cout << "SELECT * quadsegements Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    int segmentcount = 0;
    int batchsize2 = 130251;
    for (auto row : result)
    {
        std::string quadid = row[0].as<std::string>();
        int roadufi = row[1].as<int>();
        double x1 = row[2].as<double>();
        double y1 = row[3].as<double>();
        double x2 = row[4].as<double>();
        double y2 = row[5].as<double>();
        Point p1 = {x1, y1};
        Point p2 = {x2, y2};
        Segment segment = {p1, p2, roadufi};
        quadtree_map[quadid]->segments.push_back(segment);

        segmentcount++;
        if (segmentcount % batchsize2 == 0)
        {
            std::cout << "Batch: " << segmentcount / batchsize2 << std::endl;
        }
    }

    return quadtree_map["0"];
}

using FrontierElement1 = std::pair<QuadNode *, double>;

struct QuadtreeComparator1
{
    bool operator()(const FrontierElement1 &q1, const FrontierElement1 &q2)
    {
        return q1.second > q2.second;
    }
};

std::pair<Segment, double> find_nearest_segment_map(Point p, QuadNode *root)
{

    double min_distance = std::numeric_limits<double>::max();
    Segment nearest_segment = {{0, 0}, {0, 0}, -1};

    std::priority_queue<FrontierElement1, std::vector<FrontierElement1>, QuadtreeComparator1> frontier;
    frontier.push({root, 0});

    while (!frontier.empty())
    {
        FrontierElement1 q = frontier.top();
        frontier.pop();
        QuadNode *quadnode = q.first;
        double distance = q.second;
        QuadData quadtree = quadnode->data;
        if (distance > min_distance)
        {
            break;
        }
        if (quadtree.segment_count <= 0)
        {
            continue;
        }
        if (quadtree.divided)
        {
            for (QuadNode *child : quadnode->children)
            {
                QuadData child_quad = child->data;
                if (child_quad.segment_count <= 0)
                {
                    continue;
                }
                double child_distance = child_quad.boundary.distance_min(p.x, p.y);
                frontier.push({child, child_distance});
            }
            continue;
        }
        else
        {
            std::vector<Segment> segments = get_segments_from_sql_table(quadtree.quadid);
            for (Segment segment : segments)
            {
                double segment_distance = segment.perpendicularDistanceToPoint(p);
                if (segment_distance < min_distance)
                {
                    min_distance = segment_distance;
                    nearest_segment = segment;
                }
            }
        }
    }

    return {nearest_segment, min_distance};
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

// Example usage
int main()
{

    begin = std::chrono::steady_clock::now();
    QuadNode *root = build_quadtree();
    end = std::chrono::steady_clock::now();
    std::cout << "Build quadtree: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

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
        auto [nearestSegment, minDistance] = find_nearest_segment(p);
        double distanceMeter = minDistance * 111139;
        file << id << "," << std::setprecision(17) << stop.first << "," << std::setprecision(17) << stop.second << "," << std::setprecision(17) << stop.second << "," << nearestSegment.roadufi << "," << std::setprecision(17) << nearestSegment.p1.x << "," << std::setprecision(17) << nearestSegment.p1.y << "," << std::setprecision(17) << nearestSegment.p2.x << "," << std::setprecision(17) << nearestSegment.p2.y << "," << std::setprecision(17) << minDistance << "," << std::setprecision(17) << distanceMeter << std::endl;
        std::cout << "Nearest segment to (" << stop.first << ", " << stop.second << ") " << "is from (" << std::setprecision(17) << nearestSegment.p1.x << ", " << std::setprecision(17) << nearestSegment.p1.y << ") " << "to (" << std::setprecision(17) << nearestSegment.p2.x << ", " << std::setprecision(17) << nearestSegment.p2.y << ") " << "with distance: " << std::setprecision(17) << minDistance << " " << "roadufi: " << nearestSegment.roadufi << std::endl;
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
    auto [nearestSegment, minDistance] = find_nearest_segment_map(p, root);

    end = std::chrono::steady_clock::now();
    std::cout << "Nearest segment to (" << std::setprecision(17) << p.x << ", " << std::setprecision(17) << p.y << ") " << "is from (" << std::setprecision(17) << nearestSegment.p1.x << ", " << std::setprecision(17) << nearestSegment.p1.y << ") " << "to (" << std::setprecision(17) << nearestSegment.p2.x << ", " << std::setprecision(17) << nearestSegment.p2.y << ") " << "with distance: " << std::setprecision(17) << minDistance << " " << "roadufi: " << nearestSegment.roadufi << std::endl;

    std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

#endif

    return 0;

    // docker exec -it searchc-dev-container-1 /bin/bash
    // g++ q-tree-heavy.cpp -o local-exe $(pkg-config --cflags --libs libpqxx libpq) && ./local-exe
}
