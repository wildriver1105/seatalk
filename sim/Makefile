# Host simulator: streams the same JSON-over-TCP the ESP32 firmware emits.
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2
SKETCH   := ../seatalk_gateway

sim_server: sim_server.cpp $(SKETCH)/n2k_decoder.cpp $(SKETCH)/n2k_decoder.h
	$(CXX) $(CXXFLAGS) -I$(SKETCH) -o $@ sim_server.cpp $(SKETCH)/n2k_decoder.cpp

run: sim_server
	./sim_server

clean:
	rm -f sim_server

.PHONY: run clean
