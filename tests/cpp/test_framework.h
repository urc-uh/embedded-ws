// test_framework.h -- ~60 lines of test harness, no dependencies.
//
// Deliberately not GoogleTest or Unity: the Python suite this replaces needed
// no install step, and a firmware team should be able to run `make test` on a
// fresh laptop with nothing but a compiler. Same bargain here.

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <cstdio>
#include <cstring>
#include <cstdint>

namespace testing {

inline int& passCount() { static int n = 0; return n; }
inline int& failCount() { static int n = 0; return n; }
inline const char*& currentTest() { static const char* s = ""; return s; }
inline bool& currentFailed() { static bool b = false; return b; }

inline void reportFailure(const char* file, int line, const char* expr) {
    if (!currentFailed()) {
        std::printf("\n  FAIL  %s\n", currentTest());
        currentFailed() = true;
    }
    std::printf("        %s:%d: %s\n", file, line, expr);
    failCount()++;
}

inline void beginTest(const char* name) {
    currentTest() = name;
    currentFailed() = false;
}

inline void endTest() {
    if (!currentFailed()) { std::printf("  ok    %s\n", currentTest()); passCount()++; }
}

inline int summary(const char* suite) {
    std::printf("\n%s: %d passed, %d failed\n", suite, passCount(), failCount());
    return failCount() == 0 ? 0 : 1;
}

}  // namespace testing

#define RUN_TEST(fn)  do { testing::beginTest(#fn); fn(); testing::endTest(); } while (0)

#define CHECK(expr) \
    do { if (!(expr)) testing::reportFailure(__FILE__, __LINE__, #expr); } while (0)

#define CHECK_EQ(a, b)                                                        \
    do {                                                                      \
        auto _a = (a); auto _b = (b);                                         \
        if (!(_a == _b)) {                                                    \
            char _m[256];                                                     \
            std::snprintf(_m, sizeof(_m), "%s == %s  (got %lld, want %lld)",  \
                          #a, #b, (long long)_a, (long long)_b);              \
            testing::reportFailure(__FILE__, __LINE__, _m);                   \
        }                                                                     \
    } while (0)

#define CHECK_STREQ(a, b) \
    do { if (std::strcmp((a), (b)) != 0) testing::reportFailure(__FILE__, __LINE__, #a " == " #b); } while (0)

#endif  // TEST_FRAMEWORK_H
