# Host-side logic test for the SeaTalkng/NMEA2000 decoder.
# Reuses the exact same decoder source the ESP32 firmware compiles.
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2
SKETCH   := ../seatalk_gateway

test: test_decoder
	./test_decoder

test_decoder: test_decoder.cpp $(SKETCH)/n2k_decoder.cpp $(SKETCH)/n2k_decoder.h
	$(CXX) $(CXXFLAGS) -I$(SKETCH) -o $@ test_decoder.cpp $(SKETCH)/n2k_decoder.cpp

clean:
	rm -f test_decoder

.PHONY: test clean
