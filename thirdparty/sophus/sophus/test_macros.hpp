#ifndef SOPUHS_TESTS_MACROS_HPP
#define SOPUHS_TESTS_MACROS_HPP

#include <sophus/common.hpp>

namespace Sophus {
namespace details {

template <typename Scalar>
class Metric {
 public:
  static Scalar impl(Scalar s0, Scalar s1) {
    using std::abs;
    return abs(s0 - s1);
  }
};

template <typename Scalar, int M, int N>
class Metric<Eigen::Matrix<Scalar, M, N>> {
 public:
  static Scalar impl(const Eigen::Matrix<Scalar, M, N>& v0,
                     const Eigen::Matrix<Scalar, M, N>& v1) {
    return (v0 - v1).norm();
  }
};

template <typename T>
auto metric(const T& v0, const T& v1) -> decltype(Metric<T>::impl(v0, v1)) {
  return Metric<T>::impl(v0, v1);
}

template <typename Scalar>
class Pretty {
 public:
  static std::string impl(Scalar s) { return FormatString("%", s); }
};

template <typename Scalar, int M, int N>
class Pretty<Eigen::Matrix<Scalar, M, N>> {
 public:
  static std::string impl(const Eigen::Matrix<Scalar, M, N>& v) {
    return FormatString("\n%\n", v);
  }
};

template <typename T>
std::string pretty(const T& v) {
  return Pretty<T>::impl(v);
}

template <typename... Args>
void testFailed(bool& passed, const char* func, const char* file, int line,
                const std::string& msg) {
  std::cerr << FormatString("Test failed in function %, file %, line %\n", func,
                            file, line);
  std::cerr << msg << "\n\n";
  passed = false;
}
}  // namespace details

void processTestResult(bool passed) {
  if (!passed) {
    std::cerr << "failed!" << std::endl << std::endl;
    exit(-1);
  }
  std::cerr << "passed." << std::endl << std::endl;
}
}  // namespace Sophus

#define SOPHUS_STRINGIFY(x) #x

// GenericTests whether left is equal to right given a threshold.
// The in-out parameter passed will be set to false if test fails.
#define SOPHUS_TEST_EQUAL(passed, left, right, ...)                            \
  do {                                                                         \
    if (left != right) {                                                       \
      std::string msg = Sophus::details::FormatString(                         \
          "% (=%) is not equal to % (=%)\n", SOPHUS_STRINGIFY(left),           \
          Sophus::details::pretty(left), SOPHUS_STRINGIFY(right),              \
          Sophus::details::pretty(right));                                     \
      msg += Sophus::details::FormatString(__VA_ARGS__);                       \
      Sophus::details::testFailed(passed, SOPHUS_FUNCTION, __FILE__, __LINE__, \
                                  msg);                                        \
    }                                                                          \
  } while (false)

// GenericTests whether left is equal to right given a threshold.
// The in-out parameter passed will be set to false if test fails.
#define SOPHUS_TEST_NEQ(passed, left, right, ...)                              \
  do {                                                                         \
    if (left == right) {                                                       \
      std::string msg = Sophus::details::FormatString(                         \
          "% (=%) shoudl not be equal to % (=%)\n", SOPHUS_STRINGIFY(left),    \
          Sophus::details::pretty(left), SOPHUS_STRINGIFY(right),              \
          Sophus::details::pretty(right));                                     \
      msg += Sophus::details::FormatString(__VA_ARGS__);                       \
      Sophus::details::testFailed(passed, SOPHUS_FUNCTION, __FILE__, __LINE__, \
                                  msg);                                        \
    }                                                                          \
  } while (false)

// GenericTests whether left is approximatly equal to right given a threshold.
// The in-out parameter passed will be set to false if test fails.
#define SOPHUS_TEST_APPROX(passed, left, right, thr, ...)                      \
  do {                                                                         \
    auto nrm = Sophus::details::metric((left), (right));                       \
    if (!(nrm < (thr))) {                                                      \
      std::string msg = Sophus::details::FormatString(                         \
          "% (=%) is not approx % (=%); % is % \n", SOPHUS_STRINGIFY(left),    \
          Sophus::details::pretty(left), SOPHUS_STRINGIFY(right),              \
          Sophus::details::pretty(right), SOPHUS_STRINGIFY(thr),               \
          Sophus::details::pretty(thr));                                       \
      msg += Sophus::details::FormatString(__VA_ARGS__);                       \
      Sophus::details::testFailed(passed, SOPHUS_FUNCTION, __FILE__, __LINE__, \
                                  msg);                                        \
    }                                                                          \
  } while (false)

#endif  // SOPUHS_TESTS_MACROS_HPP
