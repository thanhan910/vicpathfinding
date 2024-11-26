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

mongocxx::instance instance{};
// mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
mongocxx::client client{mongocxx::uri{"mongodb://host.docker.internal:27017"}};

mongocxx::database db = client["vic_db"];

// pqxx::connection conn("dbname=vic_db user=postgres password=postgres host=localhost port=5432");
pqxx::connection conn("dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432");