CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wno-psabi -Wall -Wextra
INCLUDES := -Isrc -Ithird_party/cdt -Ithird_party
DEPFLAGS := -MMD -MP
LDFLAGS ?=
LDLIBS ?= -pthread

BUILD := build
BIN := $(BUILD)/terrain
SRC := $(wildcard src/terrain/*.cpp) src/cli_main.cpp src/http_server.cpp
OBJ := $(patsubst %.cpp,$(BUILD)/%.o,$(SRC))
DEPS := $(OBJ:.o=.d)

.PHONY: all clean run selftest

all: $(BIN)

$(BIN): $(OBJ)
	$(CXX) $(LDFLAGS) -o $@ $(OBJ) $(LDLIBS)

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(DEPFLAGS) -c $< -o $@

-include $(DEPS)

run: $(BIN)
	./$(BIN) serve 8787

selftest: $(BIN)
	./scripts/selftest.sh

clean:
	rm -rf $(BUILD)
