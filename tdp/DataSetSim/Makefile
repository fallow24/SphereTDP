COMPILER = g++
BUILD_DIR = ./bin
FLAGS = -Wall -Wno-deprecated -fopenmp -std=c++17 -I$(BUILD_DIR)/
BOOST_LIB_PATH = /usr/lib/x86_64-linux-gnu
BOOST_LIB = /libboost_program_options.so

OBJS = line_scanner.o noisy_line_scanner.o livoxMid40.o livoxMid100.o livoxMid70C.o roller.o descender.o stepwise_descender.o rectangle_world.o cylinder_world.o parsing.o utils.o

OBJS_IN_DIR = $(patsubst %, $(BUILD_DIR)/%, $(OBJS))

$(BUILD_DIR)/dataset_sim: $(OBJS_IN_DIR) dataset_sim.hpp dataset_sim.cpp
	$(COMPILER) $(FLAGS) -I$(BOOST_LIB_PATH) -o $(BUILD_DIR)/dataset_sim $(OBJS_IN_DIR) dataset_sim.cpp $(BOOST_LIB_PATH)$(BOOST_LIB)

debug:
	$(COMPILER) -g $(FLAGS) -I$(BOOST_LIB_PATH) -o $(BUILD_DIR)/dataset_sim utils.cpp sensors/line_scanner.cpp sensors/noisy_line_scanner.cpp sensors/livoxMid40.cpp sensors/livoxMid70C.cpp sensors/livoxMid100.cpp   movers/roller.cpp movers/descender.cpp movers/stepwise_descender.cpp worlds/rectangle_world.cpp worlds/cylinder_world.cpp parsing.cpp dataset_sim.cpp $(BOOST_LIB_PATH)$(BOOST_LIB)

$(BUILD_DIR)/line_scanner.o: $(BUILD_DIR)/utils.o sensors/line_scanner.hpp sensors/line_scanner.cpp sensors/sensor.hpp
	$(COMPILER) $(FLAGS) -c sensors/line_scanner.cpp -o $(BUILD_DIR)/line_scanner.o

$(BUILD_DIR)/noisy_line_scanner.o: $(BUILD_DIR)/line_scanner.o sensors/noisy_line_scanner.hpp sensors/noisy_line_scanner.cpp sensors/sensor.hpp
	$(COMPILER) $(FLAGS) -c sensors/noisy_line_scanner.cpp -o $(BUILD_DIR)/noisy_line_scanner.o

$(BUILD_DIR)/livoxMid40.o: $(BUILD_DIR)/utils.o $(BUILD_DIR)/noisy_line_scanner.o sensors/livoxMid40.hpp sensors/livoxMid40.cpp
	$(COMPILER) $(FLAGS) -c sensors/livoxMid40.cpp -o $(BUILD_DIR)/livoxMid40.o

$(BUILD_DIR)/livoxMid70C.o: $(BUILD_DIR)/utils.o $(BUILD_DIR)/noisy_line_scanner.o $(BUILD_DIR)/livoxMid40.o sensors/livoxMid70C.hpp sensors/livoxMid70C.cpp 
	$(COMPILER) $(FLAGS) -c sensors/livoxMid70C.cpp -o $(BUILD_DIR)/livoxMid70C.o

$(BUILD_DIR)/livoxMid100.o: $(BUILD_DIR)/utils.o $(BUILD_DIR)/noisy_line_scanner.o $(BUILD_DIR)/livoxMid40.o sensors/livoxMid100.hpp sensors/livoxMid100.cpp
	$(COMPILER) $(FLAGS) -c sensors/livoxMid100.cpp -o $(BUILD_DIR)/livoxMid100.o

$(BUILD_DIR)/descender.o: $(BUILD_DIR)/utils.o movers/mover.hpp movers/descender.hpp movers/descender.cpp
	$(COMPILER) $(FLAGS) -c movers/descender.cpp -o $(BUILD_DIR)/descender.o

$(BUILD_DIR)/stepwise_descender.o: $(BUILD_DIR)/utils.o movers/mover.hpp movers/stepwise_descender.hpp movers/stepwise_descender.cpp
	$(COMPILER) $(FLAGS) -c movers/stepwise_descender.cpp -o $(BUILD_DIR)/stepwise_descender.o

$(BUILD_DIR)/roller.o: $(BUILD_DIR)/utils.o movers/mover.hpp movers/roller.hpp movers/roller.cpp 
	$(COMPILER) $(FLAGS) -c movers/roller.cpp -o $(BUILD_DIR)/roller.o

$(BUILD_DIR)/cylinder_world.o: $(BUILD_DIR)/utils.o worlds/cylinder_world.hpp worlds/cylinder_world.cpp worlds/world.hpp
	$(COMPILER) $(FLAGS) -c worlds/cylinder_world.cpp -o $(BUILD_DIR)/cylinder_world.o

$(BUILD_DIR)/rectangle_world.o: $(BUILD_DIR)/utils.o worlds/rectangle_world.hpp worlds/rectangle_world.cpp worlds/world.hpp
	$(COMPILER) $(FLAGS) -c worlds/rectangle_world.cpp -o $(BUILD_DIR)/rectangle_world.o

$(BUILD_DIR)/parsing.o: parsing.hpp parsing.cpp 
	$(COMPILER) $(FLAGS) -c parsing.cpp -o $(BUILD_DIR)/parsing.o

$(BUILD_DIR)/utils.o: utils.cpp constants.hpp
	$(COMPILER) $(FLAGS) -c utils.cpp -o $(BUILD_DIR)/utils.o

clean:
	rm -f ./bin/*
