#include <chrono>
#include <cstring>

#include "test.hpp"

namespace t {

std::vector<Case>& registry() {
  static std::vector<Case> r;
  return r;
}

static int g_case_failures = 0;

void fail(const char* file, int line, const char* expr) {
  ++g_case_failures;
  if (g_case_failures <= 10) std::printf("    CHECK FAILED %s:%d: %s\n", file, line, expr);
}

}  // namespace t

int main(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int passed = 0;
  int failed = 0;
  for (const auto& c : t::registry()) {
    if (filter && std::strstr(c.name, filter) == nullptr) continue;
    t::g_case_failures = 0;
    std::printf("[ RUN  ] %s\n", c.name);
    std::fflush(stdout);
    const auto t0 = std::chrono::steady_clock::now();
    c.fn();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    if (t::g_case_failures == 0) {
      ++passed;
      std::printf("[  OK  ] %s (%lld ms)\n", c.name, static_cast<long long>(ms));
    } else {
      ++failed;
      std::printf("[ FAIL ] %s (%d failed checks)\n", c.name, t::g_case_failures);
    }
  }
  std::printf("\n%d passed, %d failed\n", passed, failed);
  return failed == 0 ? 0 : 1;
}
