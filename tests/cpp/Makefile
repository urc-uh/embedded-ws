# tests/cpp/Makefile -- native build of the firmware's portable core.
#
# These tests compile the SAME rover_protocol.* and rover_controller.* sources
# that the Teensy build compiles. There is no mock or reimplementation of the
# safety logic: what is tested here is what gets flashed.
#
#   make          build both suites
#   make test     build and run them
#   make clean
#
# Needs nothing but a C++17 compiler. No Teensyduino, no board, no install.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g
SRC_DIR   = ../../firmware/src
BUILD_DIR = build

CORE = $(SRC_DIR)/rover_protocol.cpp $(SRC_DIR)/rover_controller.cpp \
       $(SRC_DIR)/rover_drive.cpp
# Listed so that editing only a header (e.g. a constant in rover_config.h)
# still triggers a rebuild instead of silently testing the old binaries.
HDRS = $(SRC_DIR)/rover_config.h $(SRC_DIR)/rover_protocol.h \
       $(SRC_DIR)/rover_controller.h $(SRC_DIR)/rover_drive.h
BINS = $(BUILD_DIR)/test_protocol $(BUILD_DIR)/test_controller \
       $(BUILD_DIR)/test_drive

.PHONY: all test clean
all: $(BINS)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/test_protocol: test_protocol.cpp $(CORE) $(HDRS) test_framework.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) test_protocol.cpp $(CORE) -o $@

$(BUILD_DIR)/test_controller: test_controller.cpp $(CORE) $(HDRS) test_framework.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) test_controller.cpp $(CORE) -o $@

$(BUILD_DIR)/test_drive: test_drive.cpp $(CORE) $(HDRS) test_framework.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) test_drive.cpp $(CORE) -o $@

test: $(BINS)
	@fail=0; for t in $(BINS); do ./$$t || fail=1; echo; done; \
	 if [ $$fail -eq 0 ]; then echo "ALL C++ TESTS PASSED"; else echo "C++ TESTS FAILED"; fi; \
	 exit $$fail

clean:
	@rm -rf $(BUILD_DIR)
