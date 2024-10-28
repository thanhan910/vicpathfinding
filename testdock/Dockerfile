FROM gcc:latest

# Install necessary packages
RUN apt-get update && apt-get install -y \
    libpq-dev \
    libmongocxx-dev \
    cmake \
    build-essential

# Create the app directory
WORKDIR /app

# Copy source files into container
COPY . /app

# Build the C++ app
RUN mkdir build && cd build && cmake .. && make

CMD ["./app"]
