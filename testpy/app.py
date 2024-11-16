import psycopg2
from pymongo import MongoClient

# PostgreSQL connection
def fetch_postgres_data():
    try:
        conn = psycopg2.connect(
            host="host.docker.internal",
            database="vic_db",
            user="postgres",
            password="postgres",
            port=5432
        )
        cur = conn.cursor()
        cur.execute("SELECT * FROM vmtrans.stops LIMIT 5;")
        rows = cur.fetchall()
        print("PostgreSQL Data:")
        for row in rows:
            print(row)
        cur.close()
        conn.close()
    except Exception as e:
        print(f"Error connecting to PostgreSQL: {e}")

# MongoDB connection
def fetch_mongodb_data():
    try:
        client = MongoClient("mongodb://host.docker.internal:27017/")
        db = client['vic_db']
        collection = db['points']
        data = collection.find().limit(5)
        print("MongoDB Data:")
        for doc in data:
            print(doc)
    except Exception as e:
        print(f"Error connecting to MongoDB: {e}")

if __name__ == "__main__":
    fetch_postgres_data()
    fetch_mongodb_data()
