FQBN ?= arduino:avr:leonardo
PORT ?=
BUILD ?= build
CXXFLAGS ?= -std=c++17 -O0 -g -Wall -Wextra -Werror --coverage
SRC = sled/sled.ino sled/sled_core.h sled/test/test_core.cpp

.PHONY: all format test compile upload clean

all: format test compile

format:
	clang-format --dry-run -Werror $(SRC)

$(BUILD)/test_core: sled/test/test_core.cpp sled/sled_core.h
	mkdir -p $(BUILD)
	rm -f $(BUILD)/*.gcda
	$(CXX) $(CXXFLAGS) -DSLED_TRANSCRIPT='"$(CURDIR)/sled/test/protocol.txt"' -o $@ $<

test: $(BUILD)/test_core
	$(BUILD)/test_core
	gcovr -r . --filter sled/sled_core.h --fail-under-line 95 --fail-under-branch 85 $(BUILD)

compile:
	arduino-cli compile --fqbn $(FQBN) --warnings all --output-dir $(BUILD)/sled sled

upload:
	@test -n "$(PORT)" || (echo 'set PORT to the Leonardo (USB id 2341:8036)'; exit 1)
	arduino-cli upload --fqbn $(FQBN) -p $(PORT) --input-dir $(BUILD)/sled sled

clean:
	rm -rf $(BUILD)
