#include <iostream>
#include <pqxx/pqxx>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/options/find.hpp>
#include <bsoncxx/json.hpp>

int main() {
    try {
        pqxx::connection C("dbname=vic_db user=postgres password=postgres host=" + std::string(getenv("POSTGRES_HOST")) + " port=" + std::string(getenv("POSTGRES_PORT")));
        if (C.is_open()) {

            // Print the POSTGRES_HOST and POSTGRES_PORT environment variables
            std::cout << "POSTGRES_HOST: " << std::string(getenv("POSTGRES_HOST")) << std::endl;
            std::cout << "POSTGRES_PORT: " << std::string(getenv("POSTGRES_PORT")) << std::endl;

            std::cout << "Connected to PostgreSQL!" << std::endl;

            // Create a transactional object
            pqxx::work W(C);
            pqxx::result R = W.exec("SELECT * FROM vmtrans.stops LIMIT 5");

            for (auto row : R) {
                std::cout << "ID = " << row[0].as<int>() << " Name = " << row[1].as<std::string>() << std::endl;
            }

            W.commit();
        } else {
            std::cout << "Failed to connect to PostgreSQL." << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    // MongoDB connection
    try {
        mongocxx::instance inst{};
        mongocxx::client conn{mongocxx::uri{"mongodb://" + std::string(getenv("MONGO_HOST")) + ":" + std::string(getenv("MONGO_PORT"))}};
        auto db = conn["vic_db"];
        auto collection = db["points"];

        // Set the limit option to 5
        mongocxx::options::find find_options;
        find_options.limit(5);

        // Apply the options when performing the find operation
        auto cursor = collection.find({}, find_options);
    
        for (auto&& doc : cursor) {
            std::cout << bsoncxx::to_json(doc) << std::endl;
        }

        std::cout << "Connected to MongoDB!" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "MongoDB connection error: " << e.what() << std::endl;
    }

    return 0;
}
