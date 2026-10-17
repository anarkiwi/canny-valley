# Radar controller core: sources and flags shared by the host test build and
# the XC16 image (included from firmware/Makefile, paths relative to it).
RADAR_DIR = radar
SCPI_DIR = $(RADAR_DIR)/lib/scpi-parser/libscpi
TUSB_DIR = $(RADAR_DIR)/lib/tinyusb/src
RADAR_CORE_SRC = $(wildcard $(RADAR_DIR)/core/*.c)
RADAR_CORE_HDR = $(wildcard $(RADAR_DIR)/core/*.h)
RADAR_TEST_SRC = $(wildcard $(RADAR_DIR)/test/*.cpp)
SCPI_SRC = $(wildcard $(SCPI_DIR)/src/*.c)
RADAR_DEFS = -DSCPI_USER_CONFIG=1
RADAR_INC = -I$(RADAR_DIR)/core -I$(SCPI_DIR)/inc
RADAR_FORMAT = $(RADAR_CORE_SRC) $(RADAR_CORE_HDR) $(RADAR_TEST_SRC)

RADAR_TEST_CFLAGS ?= -std=c99 -O0 -g -Wall -Wextra -Werror --coverage
RADAR_TEST_CXXFLAGS ?= -std=c++17 -O0 -g -Wall -Wextra -Werror --coverage

$(BUILD)/radar_test/scpi/%.o: $(SCPI_DIR)/src/%.c
	mkdir -p $(dir $@)
	$(CC) -std=c99 -O0 -w $(RADAR_DEFS) $(RADAR_INC) -c -o $@ $<

$(BUILD)/radar_test/core/%.o: $(RADAR_DIR)/core/%.c $(RADAR_CORE_HDR)
	mkdir -p $(dir $@)
	$(CC) $(RADAR_TEST_CFLAGS) $(RADAR_DEFS) $(RADAR_INC) -c -o $@ $<

$(BUILD)/radar_test/test_core: $(RADAR_TEST_SRC) $(RADAR_CORE_HDR) \
		$(patsubst $(RADAR_DIR)/core/%.c,$(BUILD)/radar_test/core/%.o,$(RADAR_CORE_SRC)) \
		$(patsubst $(SCPI_DIR)/src/%.c,$(BUILD)/radar_test/scpi/%.o,$(SCPI_SRC))
	rm -f $(BUILD)/radar_test/core/*.gcda $(BUILD)/radar_test/*.gcda
	$(CXX) $(RADAR_TEST_CXXFLAGS) $(RADAR_DEFS) $(RADAR_INC) -o $@ \
		$(RADAR_TEST_SRC) $(filter %.o,$^)

$(BUILD)/radar_test/core/cmds.o: RADAR_TEST_CFLAGS += -DRADAR_COMMIT='"0123456789ab"'

radar-test: $(BUILD)/radar_test/test_core
	timeout 120 $(BUILD)/radar_test/test_core
	gcovr -r . --filter '$(RADAR_DIR)/core/' --fail-under-line 90 \
		--fail-under-branch 80 $(BUILD)/radar_test

.PHONY: radar-test
