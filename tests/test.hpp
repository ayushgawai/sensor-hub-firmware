// Minimal test framework (the project uses only the standard library).
#pragma once

#include <cstdio>
#include <vector>

namespace t {

struct Case {
  const char* name;
  void (*fn)();
};
std::vector<Case>& registry();
struct Reg {
  Reg(const char* n, void (*f)()) { registry().push_back({n, f}); }
};
void fail(const char* file, int line, const char* expr);
// Prints an indented measurement line under the current test.
#define NOTE(...)            \
  do {                       \
    std::printf("    ");     \
    std::printf(__VA_ARGS__); \
    std::printf("\n");       \
  } while (0)

}  // namespace t

#define TEST(name)                                \
  static void name();                             \
  static const t::Reg reg_##name(#name, &name);   \
  static void name()

#define CHECK(cond)                                   \
  do {                                                \
    if (!(cond)) t::fail(__FILE__, __LINE__, #cond);  \
  } while (0)

#define REQUIRE(cond)                       \
  do {                                      \
    if (!(cond)) {                          \
      t::fail(__FILE__, __LINE__, #cond);   \
      return;                               \
    }                                       \
  } while (0)
