# Use the official Python image
FROM python:latest

# Set the working directory
WORKDIR /app

# Copy the current directory contents into the container
COPY . .

# Install the Python dependencies
RUN pip install --no-cache-dir psycopg2 pymongo

# Set the entry point for the container to run your script
CMD ["python", "app.py"]
