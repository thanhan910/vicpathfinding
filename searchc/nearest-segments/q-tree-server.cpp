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



namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;


std::chrono::steady_clock::time_point begin, end;

pqxx::connection conn("dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432");

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


struct Boundary
{
    double x_min, y_min, x_max, y_max;

    bool contains(const Point &p) const
    {
        return (p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max);
    }

    bool intersects_bpundary(const Boundary &other) const
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



// Quadtree node
class Quadtree
{
public:
    // Bounding box for each quadrant
    Quadtree(Boundary boundary, int capacity = 4)
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
                double distance = segment.distanceToPoint(point);
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


    // Find the nearest segment to a point
    std::vector<Quadtree *> nearestSegmentNew(const Point &point, double &minDistance, Segment &nearestSegment) const
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
                double distance = segment.distanceToPoint(point);
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

using FrontierElement = std::pair<Quadtree*, double>;

struct CompareFrontierElement
{
    bool operator()(const FrontierElement &q1, const FrontierElement &q2)
    {
        return q1.second > q2.second;
    }
};


std::tuple<Segment, double, std::vector<Quadtree *>> find_nearest_segment_frontier_full(const Point &p, Quadtree* root)
{
    double min_distance = std::numeric_limits<double>::max();
    Segment nearest_segment = {{0, 0}, {0, 0}, -1};

    std::priority_queue<FrontierElement, std::vector<FrontierElement>, CompareFrontierElement> frontier; 

    frontier.push({root, 0.0});

    std::vector<Quadtree *> quads;
    while (!frontier.empty())
    {
        FrontierElement q = frontier.top();
        frontier.pop();
        Quadtree* quad = q.first;
        double distance = q.second;
        quads.push_back(quad);

        if (distance > min_distance)
        {
            break;
        }
        if (quad->getSegmentCount() == 0)
        {
            continue;
        }
        if (quad->isDivided())
        {
            for (Quadtree* child : quad->getChildren())
            {
                if(child->getSegmentCount() <= 0) {
                    continue;
                }
                frontier.push({child, child->getBoundary().distance_min(p)});
                
            }
        }
        else
        {
            for (const Segment& segment : quad->getSegments())
            {
                double distance = segment.distanceToPoint(p);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_segment = segment;
                }
            }
        }
    }
    
    return {nearest_segment, min_distance, quads};
}

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
    Boundary boundary = {x_min, y_min, x_max, y_max};
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

// Function to parse query parameters from the URL
std::optional<std::pair<double, double>> parse_query(const std::string_view& query) {
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
void handle_request(Quadtree* quadtree, http::request<http::string_body>& req, http::response<http::string_body>& res) {
    if (req.method() == http::verb::get && req.target() == "/quadtree/summary") {
        // Return the summary of the quadtree
        res.result(http::status::ok);
        res.set(http::field::content_type, "text/plain");
        res.body() = "Quadtree summary stub";
    }
    else if (req.method() == http::verb::get && req.target().starts_with("/quadtree/segments")) {
        // Extract query string
        std::string_view target = req.target();
        auto pos = target.find('?');
        if (pos == std::string::npos) {
            res.result(http::status::bad_request);
            res.body() = "Missing query parameters.";
            res.prepare_payload();
            return;
        }

        std::string_view query = target.substr(pos + 1);
        auto params = parse_query(query);
        if (!params) {
            res.result(http::status::bad_request);
            res.body() = "Invalid query parameters.";
            res.prepare_payload();
            return;
        }

        double x = params->first, y = params->second;

        Point p = {x, y};
        auto [nearestSegment, minDistance, quads] = find_nearest_segment_frontier_full(p, quadtree);
        res.result(http::status::ok);
        res.set(http::field::content_type, "application/json");
        res.body() = "{\"roadufi\": " + std::to_string(nearestSegment.roadufi) + ", \"distance\": " + std::to_string(minDistance) + ", \"x1\": " + std::to_string(nearestSegment.p1.x) + ", \"y1\": " + std::to_string(nearestSegment.p1.y) + ", \"x2\": " + std::to_string(nearestSegment.p2.x) + ", \"y2\": " + std::to_string(nearestSegment.p2.y) + "}";
    }

    else {
        // Handle unknown paths
        res.result(http::status::not_found);
        res.set(http::field::content_type, "text/plain");
        res.body() = "404 Not Found";
    }
    res.prepare_payload();
}

void do_session(tcp::socket socket, Quadtree* quadtree) {
    try {
        beast::flat_buffer buffer;

        // Read the HTTP request
        http::request<http::string_body> req;
        http::read(socket, buffer, req);

        // Prepare HTTP response
        http::response<http::string_body> res;

        // Handle the request
        handle_request(quadtree, req, res);

        // Write the HTTP response back to the client
        http::write(socket, res);
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

int main() {
    // Generate the quadtree
    Quadtree quadtree = gen_quadtree();

    // Define the server endpoint
    const auto address = net::ip::make_address("0.0.0.0");
    const unsigned short port = 8080;

    try {
        net::io_context ioc;

        // Create an acceptor to listen for incoming connections
        tcp::acceptor acceptor(ioc, tcp::endpoint(address, port));

        std::cout << "Server is running on http://localhost:" << port << std::endl;

        // Main server loop
        for (;;) {
            // Accept an incoming connection
            tcp::socket socket = acceptor.accept();

            // Start a session to handle the request
            std::thread(&do_session, std::move(socket), &quadtree).detach();
        }
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}

// docker exec -it searchc-dev-container-1 /bin/bash
// g++ q-tree-server.cpp -o local-exe $(pkg-config --cflags --libs libpqxx libpq) && ./local-exe

