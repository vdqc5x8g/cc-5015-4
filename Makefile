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

.PHONY: all clean run selftest assets rebuild-assets verify-assets ui-test

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

# Regenerate the terrain contour package from assets/source/*.json. Uses the
# same CDT kernel as the CLI, and rewrites the derived contours, models and
# previews; the recipes are never edited by this target.
assets: $(BIN)
	python3 -I scripts/make_terrain_assets.py

# Full reproduction: regenerate + verify hashes, elevation error and
# constraint-edge evidence. Fails if any derived file drifts.
rebuild-assets: $(BIN)
	python3 -I scripts/rebuild_assets.py

verify-assets:
	python3 -I scripts/verify_terrain_assets.py

# Drives the real UI in headless Chromium. Requires puppeteer and a browser;
# see README "Verifying the viewport".
ui-test: $(BIN)
	node scripts/ui_asset_test.js

clean:
	rm -rf $(BUILD)
