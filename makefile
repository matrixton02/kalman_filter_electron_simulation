# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -I /C++/mingw32/include

# Source files
SRC = main.cpp electron_sim.cpp kalman.cpp
OBJ = $(SRC:.cpp=.o)

# Output executable name
TARGET = electron_sim

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
