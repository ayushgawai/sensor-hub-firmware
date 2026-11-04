# sensor-hub-firmware: firmware core + host simulator, tests and bench.
# Requires a C++17 compiler (Apple clang is the tested one) and GNU make.

CXX := clang++

STD   := -std=c++17
WARN  := -Wall -Wextra -Wpedantic -Wshadow -Werror
INC   := -Iinclude -Isim
NOEXC := -fno-exceptions -fno-rtti

CORE_SRC  := $(wildcard src/*.cpp)
SIM_SRC   := $(wildcard sim/*.cpp)
TEST_SRC  := $(wildcard tests/*.cpp)
BENCH_SRC := $(wildcard bench/*.cpp)

# Build variants, each in its own directory:
#   fw   firmware core only, -Os: footprint and no-heap check
#   rel  -O2: bench
#   asan tests with AddressSanitizer + UndefinedBehaviorSanitizer
#   tsan ring buffer test with ThreadSanitizer
FW_DIR   := build/fw
REL_DIR  := build/rel
ASAN_DIR := build/asan
TSAN_DIR := build/tsan

FW_FLAGS   := $(STD) $(WARN) $(INC) $(NOEXC) -Os -fstack-usage
REL_FLAGS  := $(STD) $(WARN) $(INC) $(NOEXC) -O2 -g
ASAN_FLAGS := $(STD) $(WARN) $(INC) $(NOEXC) -O1 -g -fno-omit-frame-pointer \
              -fsanitize=address,undefined -fno-sanitize-recover=all
TSAN_FLAGS := $(STD) $(WARN) $(INC) $(NOEXC) -O1 -g -fsanitize=thread

objs = $(patsubst %.cpp,$(1)/%.o,$(2))

define compile_rule
$(1)/%.o: %.cpp
	@mkdir -p $$(@D)
	@echo "  CXX   $$@"
	@$$(CXX) $(2) -MMD -MP -c $$< -o $$@
endef
$(eval $(call compile_rule,$(FW_DIR),$(FW_FLAGS)))
$(eval $(call compile_rule,$(REL_DIR),$(REL_FLAGS)))
$(eval $(call compile_rule,$(ASAN_DIR),$(ASAN_FLAGS)))
$(eval $(call compile_rule,$(TSAN_DIR),$(TSAN_FLAGS)))

FW_OBJS    := $(call objs,$(FW_DIR),$(CORE_SRC))
TEST_OBJS  := $(call objs,$(ASAN_DIR),$(CORE_SRC) $(SIM_SRC) $(TEST_SRC))
TSAN_OBJS  := $(call objs,$(TSAN_DIR),tests/test_main.cpp tests/test_ring.cpp)
BENCH_OBJS := $(call objs,$(REL_DIR),$(CORE_SRC) $(SIM_SRC) $(BENCH_SRC))

TEST_BIN  := $(ASAN_DIR)/shf_tests
TSAN_BIN  := $(TSAN_DIR)/shf_ring_tsan
BENCH_BIN := $(REL_DIR)/shf_bench

.PHONY: all core test bench check-noheap footprint clean

all: core $(TEST_BIN) $(BENCH_BIN)

core: $(FW_OBJS)

$(TEST_BIN): $(TEST_OBJS)
	@echo "  LINK  $@"
	@$(CXX) $(ASAN_FLAGS) $^ -o $@

$(TSAN_BIN): $(TSAN_OBJS)
	@echo "  LINK  $@"
	@$(CXX) $(TSAN_FLAGS) $^ -o $@

$(BENCH_BIN): $(BENCH_OBJS)
	@echo "  LINK  $@"
	@$(CXX) $(REL_FLAGS) $^ -o $@

# The firmware core must not reference the heap, exceptions or
# thread-safe static initialization.
FORBIDDEN := _Znwm|_Znam|_ZdlPv|_ZdaPv|_malloc|_calloc|_realloc|_free|___cxa_throw|___cxa_allocate_exception|___cxa_guard_acquire
check-noheap: $(FW_OBJS)
	@if nm -u $(FW_OBJS) | grep -E '^ *($(FORBIDDEN))' ; then \
	  echo "FAIL: firmware core references heap/exception symbols (above)"; exit 1; \
	else echo "PASS: firmware core has no heap, exception or static-guard references"; fi

test: $(TEST_BIN) $(TSAN_BIN) check-noheap
	./$(TEST_BIN)
	./$(TSAN_BIN)

footprint: $(FW_OBJS)
	@sh tools/footprint.sh $(FW_DIR)

bench: $(BENCH_BIN) $(FW_OBJS)
	./$(BENCH_BIN)
	@sh tools/footprint.sh $(FW_DIR)

clean:
	rm -rf build

-include $(shell find build -name '*.d' 2>/dev/null)
