#ifndef INCLUDE_MATLAB_PLOT_CPLUSPLUS
#define INCLUDE_MATLAB_PLOT_CPLUSPLUS

#include <MatlabDataArray.hpp>
#include <MatlabEngine.hpp>
#include <array>
#include <cassert>
#include <codecvt>
#include <cstring>
#include <functional>
#include <iostream>
#include <locale>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mplt {

// ! singleton helper function
template <typename T>
struct HasShutdown {
  template <typename Class>
  static constexpr bool Test(decltype(&Class::Shutdown) *) {
    return true;
  }
  template <typename>
  static constexpr bool Test(...) {
    return false;
  }
  static constexpr bool value = Test<T>(nullptr);
};
template <typename T>
constexpr bool HasShutdown<T>::value;

template <typename T>
typename std::enable_if<HasShutdown<T>::value>::type CallShutdown(T *instance) {
  instance->Shutdown();
}

template <typename T>
typename std::enable_if<!HasShutdown<T>::value>::type CallShutdown(
    T *instance) {
  (void)instance;
}

// helper function
template <typename T>
inline std::vector<T> linspace(T start, T space, T end) {
  std::vector<T> result;
  while (start <= end) {
    result.push_back(start);
    start += space;
  }
  return result;
}

template <typename T>
inline std::vector<T> sin(const std::vector<T> &x) {
  std::vector<T> result(x.size());
  for (size_t i = 0; i < x.size(); ++i) {
    result[i] = std::sin(x[i]);
  }
  return result;
}

template <typename T>
inline std::vector<T> cos(const std::vector<T> &x) {
  std::vector<T> result(x.size());
  for (size_t i = 0; i < x.size(); ++i) {
    result[i] = std::cos(x[i]);
  }
  return result;
}

inline double convertFromString(const std::string &s) {
  double value = 0.0;
  std::stringstream ss(s);
  ss >> value;
  return value;
}

// helper definition
static std::unordered_set<std::string> line_spec_need_convert{"LineWidth",
                                                              "MarkerSize"};

//! matlab engine manager
class EngineManager {
 public:
  template <typename T>
  void exec(const std::string &cmd, T args) {
    matlab_engine_ptr_->feval(cmd, args);
  }

  void exec(const std::string &cmd) {
    matlab_engine_ptr_->eval(
        std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t>{}
            .from_bytes(cmd));
  }

  matlab::data::ArrayFactory *ArrayFactory() { return &factory_; }

 private:
  EngineManager() {
    try {
      matlab_engine_ptr_ =
          matlab::engine::connectMATLAB(u"plot_shared_session");
    } catch (matlab::engine::EngineException e) {
      std::cout << e.what() << "\n";
      std::cout << "cannot connect to shared session, now start matlab using "
                   "function call"
                << "\n";
      matlab_engine_ptr_ = matlab::engine::startMATLAB();
    }

    if (matlab_engine_ptr_ == nullptr) {
      std::cerr << "Can't start MATLAB engine!" << std::endl;
    } else {
      std::cout << "Start Matlab engine successfully!" << std::endl;
    }
  }

 private:
  std::unique_ptr<matlab::engine::MATLABEngine> matlab_engine_ptr_ = nullptr;
  matlab::data::ArrayFactory factory_;

  // sington syntax
 public:
  static EngineManager *Instance(bool create_if_needed = true) {
    static EngineManager *instance = nullptr;
    if (!instance && create_if_needed) {
      static std::once_flag flag;
      std::call_once(flag,
                     [&] { instance = new (std::nothrow) EngineManager(); });
    }
    return instance;
  }
  static void CleanUp() {
    auto instance = Instance(false);
    if (instance != nullptr) {
      CallShutdown(instance);
    }
  }

 private:
  EngineManager(const EngineManager &) = delete;
  EngineManager &operator=(const EngineManager &) = delete;
};

template <typename NumericX = double, typename NumericY = double>
void plot(const std::vector<NumericX> &x, const std::vector<NumericY> &y,
          const std::string &format = "") {
  assert(x.size() == y.size());
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createArray(
           {x.size(), 1}, x.begin(), x.end()),
       EngineManager::Instance()->ArrayFactory()->createArray(
           {y.size(), 1}, y.begin(), y.end()),
       EngineManager::Instance()->ArrayFactory()->createCharArray(format)});
  EngineManager::Instance()->exec("plot", args);
}

template <typename NumericX = double, typename NumericY = double>
void plot(const std::vector<NumericX> &x, const std::vector<NumericY> &y) {
  assert(x.size() == y.size());
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createArray(
           {x.size(), 1}, x.begin(), x.end()),
       EngineManager::Instance()->ArrayFactory()->createArray(
           {y.size(), 1}, y.begin(), y.end())});
  EngineManager::Instance()->exec("plot", args);
}

template <typename NumericX = double, typename NumericY = double>
void plot(const std::vector<NumericX> &x, const std::vector<NumericY> &y,
          const std::unordered_map<std::string, std::string> &keywords) {
  assert(x.size() == y.size());
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createArray(
           {x.size(), 1}, x.begin(), x.end()),
       EngineManager::Instance()->ArrayFactory()->createArray(
           {y.size(), 1}, y.begin(), y.end())});
  for (const auto &keyword : keywords) {
    args.push_back(EngineManager::Instance()->ArrayFactory()->createCharArray(
        keyword.first));
    if (line_spec_need_convert.find(keyword.first) !=
        line_spec_need_convert.end()) {
      args.push_back(EngineManager::Instance()->ArrayFactory()->createScalar(
          convertFromString(keyword.second)));
    } else {
      args.push_back(EngineManager::Instance()->ArrayFactory()->createCharArray(
          keyword.second));
    }
  }
  EngineManager::Instance()->exec("plot", args);
}

// template <typename Numeric>
// inline void plot(const std::vector<Numeric> &x, const std::vector<Numeric>
// &y,
//           const std::string &format,
//           const std::unordered_map<std::string, std::string> &keywords) {
//   assert(x.size() == y.size());
//   MatData stl_x, stl_y;
//   stl_x.copy_from_stl(x);
//   stl_x.put(EngineManager::Instance().getInstance(), "stl_x");
//   stl_y.copy_from_stl(y);
//   stl_y.put(EngineManager::Instance().getInstance(), "stl_y");

//   std::string exec_code = "plot";
//   exec_code += "(stl_x, stl_y, ";
//   exec_code += embrance(format);
//   for (const auto &keyword : keywords) {
//     exec_code += ",";
//     exec_code += embrance(keyword.first);
//     exec_code += ",";
//     exec_code += keyword.second;
//   }
//   exec_code += ")";
//    EngineManager::Instance()->exec(exec_code);
// }

// template <typename Numeric>
// void plot(const std::vector<Numeric> &y,
//           const std::unordered_map<std::string, std::string> &keywords = "")
//           {
//   std::vector<Numeric> x(y.size());
//   for (size_t i = 0; i < x.size(); ++i) {
//     x.at(i) = i;
//   }
//    plot(x, y, keywords);
// }

template <typename Numeric>
void plot(const std::vector<Numeric> &y, const std::string &format = "") {
  assert(!y.empty());
  std::vector<Numeric> x(y.size());
  for (size_t i = 0; i < x.size(); ++i) {
    x.at(i) = i;
  }
  plot(x, y, format);
}

// template <typename Numeric>
// inline void plot(const std::vector<std::vector<Numeric>> &xy,
//           const std::string &format = "") {
//   assert(xy.size() == 2 || xy[0].size() == 2);
//   std::vector<Numeric> x;
//   std::vector<Numeric> y;
//   if (xy.size() == 2) {
//     x.resize(xy[0].size());
//     y.resize(xy[0].size());
//     for (size_t i = 0; i < x.size(); ++i) {
//       x.at(i) = xy.at(0).at(i);
//       y.at(i) = xy.at(1).at(i);
//     };
//   } else {
//     x.resize(xy.size());
//     y.resize(xy.size());
//     for (size_t i = 0; i < x.size(); ++i) {
//       x.at(i) = xy.at(i).at(0);
//       y.at(i) = xy.at(i).at(1);
//     };
//   }
//    plot<Numeric>(x, y, format);
// }

// // one point plot
// template <typename NumericX = double, typename NumericY = double>
// inline void plot_point(const NumericX &x, const NumericY &y,
//                 const std::string &format = "") {
//   std::vector<NumericX> x_v;
//   x_v.push_back(x);
//   std::vector<NumericY> y_v;
//   y_v.push_back(y);
//    plot<NumericX, NumericY>(x_v, y_v, format);
// }

namespace detail {

template <typename T>
using is_function = typename std::is_function<
    std::remove_pointer<std::remove_reference<T>>>::type;

template <bool obj, typename T>
struct is_callable_impl;

template <typename T>
struct is_callable_impl<false, T> {
  typedef is_function<T> type;
};  // a non-object is callable iff it is a function

template <typename T>
struct is_callable_impl<true, T> {
  struct Fallback {
    inline void operator()();
  };
  struct Derived : T, Fallback {};

  template <typename U, U>
  struct Check;

  template <typename U>
  static std::true_type test(...);
  // use a variadic function to make sure (1) it accepts everything
  // and (2) its always the worst match

  template <typename U>
  static std::false_type test(Check<void (Fallback::*)(), &U::operator()> *);

 public:
  typedef decltype(test<Derived>(nullptr)) type;
  typedef decltype(&Fallback::operator()) dtype;
  static constexpr bool value = type::value;
};  // an object is callable iff it defines operator()

template <typename T>
struct is_callable {
  // dispatch to is_callable_impl<true, T> or is_callable_impl<false, T>
  // depending on whether T is of class type or not
  typedef typename is_callable_impl<std::is_class<T>::value, T>::type type;
};

template <typename IsYDataCallable>
struct plot_impl {};

template <>
struct plot_impl<std::false_type> {
  template <typename IterableX, typename IterableY>
  void operator()(const IterableX &x, const IterableY &y,
                  const std::string &format) {
    // 2-phase lookup for distance, begin, end
    using std::begin;
    using std::distance;
    using std::end;

    auto xs = distance(begin(x), end(x));
    auto ys = distance(begin(y), end(y));
    assert(xs == ys && "x and y data must have the same number of elements!");

    plot(x, y, format);
  }
};

template <>
struct plot_impl<std::true_type> {
  template <typename Iterable, typename Callable>
  void operator()(const Iterable &ticks, const Callable &f,
                  const std::string &format) {
    if (begin(ticks) == end(ticks)) return;

    // We could use additional meta-programming to deduce the correct element
    // type of y, but all values have to be convertible to double anyways
    std::vector<double> y;
    for (auto x : ticks) y.push_back(f(x));
    plot_impl<std::false_type>()(ticks, y, format);
  }
};

}  // end namespace detail

// template <typename T0, typename T1>
// inline void hold(const T0 &type, const T1 &cmd) {
//   EngineManager::Instance()->exec("hold " + cmd);
// }

inline void colormap(const std::string &param) {
  EngineManager::Instance()->exec("colormap(" + param + ")");
}

inline void shading(const std::string &param) {
  EngineManager::Instance()->exec("shading " + param);
}

inline void hold_on() { EngineManager::Instance()->exec("hold on"); }

inline void hold_off() { EngineManager::Instance()->exec("hold off"); }

// recursion stop for the above
template <typename... Args>
void plot() {
  hold_off();
}

template <typename A, typename B, typename... Args>
void plot(const A &a, const B &b, const std::string &format, Args... args) {
  hold_on();
  detail::plot_impl<typename detail::is_callable<B>::type>()(a, b, format);
  plot(args...);
}

/*
 * This group of plot() functions is needed to support initializer lists,
 i.e.
 * calling plot( {1,2,3,4} )
 */
inline void plot(const std::initializer_list<double> &x,
                 const std::initializer_list<double> &y,
                 const std::string &format = "") {
  plot<double, double>(x, y, format);
}

inline void plot(const std::initializer_list<double> &y,
                 const std::string &format = "") {
  plot<double>(y, format);
}

// template <typename NumericX = double, typename NumericY = double,
//           typename NumericZ = double>
// inline void plot3(const std::vector<NumericX> &x, const std::vector<NumericY>
// &y,
//            const std::vector<NumericZ> &z, const std::string &format = "") {
//   assert(x.size() == y.size() && y.size() == z.size());
//   MatData stl_x, stl_y, stl_z;
//   stl_x.copy_from_stl(x);
//   stl_x.put(EngineManager::Instance().getInstance(), "stl_x");
//   stl_y.copy_from_stl(y);
//   stl_y.put(EngineManager::Instance().getInstance(), "stl_y");
//   stl_y.copy_from_stl(z);
//   stl_y.put(EngineManager::Instance().getInstance(), "stl_z");

//   std::string exec_code = "plot3";
//   exec_code += "(stl_x, stl_y, stl_z, ";
//   exec_code += embrance(format);
//   exec_code += ")";
//    EngineManager::Instance()->exec(exec_code);
// }

// template <class NumericX = double, class NumericY = double,
//           class NumericZ = double>
// inline void surf(const std::vector<NumericX> &x, const std::vector<NumericY>
// &y,
//           const std::vector<std::vector<NumericZ>> &z) {
//   MatData stl_x, stl_y, stl_z;
//   stl_x.copy_from_stl(x);
//   stl_x.put(EngineManager::Instance().getInstance(), "stl_x");
//   stl_y.copy_from_stl(y);
//   stl_y.put(EngineManager::Instance().getInstance(), "stl_y");
//   stl_z.copy_from_stl(z);
//   stl_z.put(EngineManager::Instance().getInstance(), "stl_z");
//   std::string exec_code = "stl_x, stl_y, stl_z";
//   exec_code = "surf(" + exec_code + ");";
//    EngineManager::Instance()->exec(exec_code);
// }

// template <class NumericX = double, class NumericY = double,
//           class NumericZ = double>
// inline void surfc(const std::vector<NumericX> &x, const std::vector<NumericY>
// &y,
//            const std::vector<std::vector<NumericZ>> &z) {
//   MatData stl_x, stl_y, stl_z;
//   stl_x.copy_from_stl(x);
//   stl_x.put(EngineManager::Instance().getInstance(), "stl_x");
//   stl_y.copy_from_stl(y);
//   stl_y.put(EngineManager::Instance().getInstance(), "stl_y");
//   stl_z.copy_from_stl(z);
//   stl_z.put(EngineManager::Instance().getInstance(), "stl_z");
//   std::string exec_code = "stl_x, stl_y, stl_z";
//   exec_code = "surfc(" + exec_code + ");";
//    EngineManager::Instance()->exec(exec_code);
// }

// template <class NumericX = double, class NumericY = double,
//           class NumericZ = double>
// inline void mesh(const std::vector<NumericX> &x, const std::vector<NumericY>
// &y,
//           const std::vector<std::vector<NumericZ>> &z) {
//   MatData stl_x, stl_y, stl_z;
//   stl_x.copy_from_stl(x);
//   stl_x.put(EngineManager::Instance().getInstance(), "stl_x");
//   stl_y.copy_from_stl(y);
//   stl_y.put(EngineManager::Instance().getInstance(), "stl_y");
//   stl_z.copy_from_stl(z);
//   stl_z.put(EngineManager::Instance().getInstance(), "stl_z");
//   std::string exec_code = "stl_x, stl_y, stl_z";
//   exec_code = "mesh(" + exec_code + ");";
//    EngineManager::Instance()->exec(exec_code);
// }

inline void figure() { EngineManager::Instance()->exec("figure"); }

template <typename Numeric = double>
inline void figure_size(Numeric width, Numeric height) {
  std::vector<Numeric> postion_def{0, 0, width, height};
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createCharArray("Position"),
       EngineManager::Instance()->ArrayFactory()->createArray(
           {postion_def.size(), 1}, postion_def.begin(), postion_def.end())});
  EngineManager::Instance()->exec("figure", args);
}

// inline void figure(const std::string &cmd) {
//   std::string exec_code = "figure('Position', [515, 205, 960, 680], ";
//   // std::string exec_code = "figure('Name', '" + cmd + "')";
//   exec_code += "'Name', '" + cmd + "')";
//    EngineManager::Instance()->exec(exec_code);
// }

// template <typename Numeric = int>
// inline void figure(const std::vector<Numeric> &cmd) {
//   std::string exec_code = "figure('Position',[";
//   for (size_t i = 0; i < cmd.size() - 1; ++i) {
//     exec_code += std::to_string(cmd[i]);
//     exec_code += ",";
//   }
//   exec_code += std::to_string(cmd.back());
//   exec_code += ")";
//    EngineManager::Instance()->exec(exec_code);
// }

// inline void figure(const std::vector<std::string> &cmd) {
//   // std::string exec_code = "figure(";
//   std::string exec_code = "figure('Position', [515, 205, 960, 680], ";
//   for (size_t i = 0; i < cmd.size() - 1; ++i) {
//     if ((i & 1) == 0) {
//       exec_code += embrance(cmd[i]);
//       exec_code += ",";
//     } else {
//       exec_code += cmd[i];
//       exec_code += ",";
//     }
//   }
//   exec_code += cmd.back();
//   exec_code += ")";
//    EngineManager::Instance()->exec(exec_code);
// }

inline void subplot(int m, int n, int p) {
  std::string exec_code = "subplot(" + std::to_string(m) + ", " +
                          std::to_string(n) + ", " + std::to_string(p) + ")";
  EngineManager::Instance()->exec(exec_code);
}

inline void subplot(int m, int n, const std::vector<int> &region) {
  assert(!region.empty());
  std::string exec_code =
      "subplot(" + std::to_string(m) + ", " + std::to_string(n) + ", [";
  for (size_t i = 0; i < region.size() - 1; i++) {
    exec_code += std::to_string(region[i]);
    exec_code += ",";
  }
  exec_code += std::to_string(region.back());
  exec_code += "])";
  EngineManager::Instance()->exec(exec_code);
}

// inline void close() {  EngineManager::Instance()->exec("close"); }

// inline void clear() {  EngineManager::Instance()->exec("clear"); }

inline void close_all() { EngineManager::Instance()->exec("close all"); }

inline void axis(const std::string &cmd) {
  EngineManager::Instance()->exec("axis " + cmd);
}

// template <typename Numeric1 = double, typename Numeric2 = double,
//           typename Numeric3 = double, typename Numeric4 = double>
// inline void axis(Numeric1 x1, Numeric2 x2, Numeric3 y1, Numeric4 y2) {
//   std::string exec_code = "axis([";
//   exec_code += std::to_string(x1);
//   exec_code += ",";
//   exec_code += std::to_string(x2);
//   exec_code += ",";
//   exec_code += std::to_string(y1);
//   exec_code += ",";
//   exec_code += std::to_string(y2);
//   exec_code += "])";
//    EngineManager::Instance()->exec(exec_code);
// }

inline void axis_equal() { axis("equal"); }
inline void axis_off() { axis("off"); }
inline void axis_tight() { axis("tight"); }

inline void grid(const std::string &cmd) {
  EngineManager::Instance()->exec("grid " + cmd);
}

inline void grid_on() { grid("on"); }

inline void grid_off() { grid("off"); }

inline void legend() { EngineManager::Instance()->exec("legend"); }

// inline void legend(const std::vector<std::string> &cmd) {
//   std::string exec_code = "legend(";
//   for (size_t i = 0; i < cmd.size() - 1; ++i) {
//     exec_code += embrance(cmd[i]);
//     exec_code += ",";
//   }
//   exec_code += embrance(cmd.back());
//   exec_code += ")";
//    EngineManager::Instance()->exec(exec_code);
// }

// template <typename NumericX = double, typename NumericY = double>
// inline void text(NumericX x, NumericY y, const std::string &cmd) {
//   std::string exec_code = "text(";
//   exec_code += std::to_string(x);
//   exec_code += ",";
//   exec_code += std::to_string(y);
//   exec_code += ",";
//   exec_code += embrance(cmd);
//   exec_code += ")";
//    EngineManager::Instance()->exec(exec_code);
// }

// template <typename Numeric = double>
// inline void xlim(Numeric x, Numeric y) {
//   std::string exec_code =
//       "xlim([" + std::to_string(x) + ", " + std::to_string(y) + "])";

//    EngineManager::Instance()->exec(exec_code);
// }

// template <typename Numeric = double>
// inline void ylim(Numeric x, Numeric y) {
//   std::string exec_code =
//       "ylim([" + std::to_string(x) + ", " + std::to_string(y) + "])";

//    EngineManager::Instance()->exec(exec_code);
// }

template <typename Numeric = double>
inline void zlim(Numeric x, Numeric y) {
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createScalar(x),
       EngineManager::Instance()->ArrayFactory()->createScalar(y)});
  return EngineManager::Instance()->exec("zlim", args);
}

// template <typename Numeric = double>
// inline void axis_lim(const Numeric &x, const Numeric &y, const Numeric &h,
//               const Numeric &w) {
//   std::string exec_code = "axis([" + std::to_string(x) + ", " +
//                           std::to_string(y) + ", " + std::to_string(h) + ", "
//                           + std::to_string(w) + "])";
//    EngineManager::Instance()->exec(exec_code);
// }

inline void xlabel(const std::string &label) {
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createCharArray(label)});
  EngineManager::Instance()->exec("xlabel", args);
}

inline void ylabel(const std::string &label) {
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createCharArray(label)});
  EngineManager::Instance()->exec("ylabel", args);
}

inline void zlabel(const std::string &label) {
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createCharArray(label)});
  EngineManager::Instance()->exec("zlabel", args);
}

inline void title(const std::string &label) {
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createCharArray(label)});
  EngineManager::Instance()->exec("title", args);
}

// // line([x_LR, x_RR],[y_LR,y_RR]);
// template <typename Numeric = double>
// inline void line(const Numeric &x1, const Numeric &x2, const Numeric &y1,
//           const Numeric &y2) {
//   std::string exec_code = "line([" + std::to_string(x1) + ", " +
//                           std::to_string(x2) + "],[" + std::to_string(y1) +
//                           ", " + std::to_string(y2) + "])";
//    EngineManager::Instance()->exec(exec_code);
// }

// // set(gca, 'xTick', minAxis_x:gridResolution:maxAxis_x);
// inline void x_tick(double x, double y, double h) {
//   std::string exec_code = "set(gca, \'xTick\'," + std::to_string(x) + ":" +
//                           std::to_string(y) + ": " + std::to_string(h) + ")";
//    EngineManager::Instance()->exec(exec_code);
// }

// inline void y_tick(double x, double y, double h) {
//   std::string exec_code = "set(gca, \'yTick\'," + std::to_string(x) + ":" +
//                           std::to_string(y) + ": " + std::to_string(h) + ")";

//    EngineManager::Instance()->exec(exec_code);
// }

// inline void rectangle(double x, double y, double h, double w) {
//   std::string exec_code = "rectangle( \'position\', [" + std::to_string(x) +
//                           ", " + std::to_string(y) + ", " + std::to_string(h)
//                           +
//                           ", " + std::to_string(w) +
//                           "] ,\'FaceColor\',[0,0,0] );";
//    EngineManager::Instance()->exec(exec_code);
// }

inline void colorbar() { EngineManager::Instance()->exec("colorbar"); }

// template <typename Numeric = double>
// inline void view(Numeric az, Numeric el) {
//   std::vector<Numeric> view_point{az, el};
//   matlab::data::Array args(
//       {EngineManager::Instance()->ArrayFactory()->createArray(
//           {view_point.size(), 1}, view_point.begin(), view_point.end())});
//   // matlab::data::Array args =
//   //     EngineManager::Instance()->ArrayFactory()->createArray(
//   //         {view_point.size(), 1}, view_point.begin(), view_point.end());
//   // std::vector<matlab::data::Array> args(
//   // {EngineManager::Instance()->ArrayFactory()->createScalar<Numeric>(az),
//   // EngineManager::Instance()->ArrayFactory()->createScalar<Numeric>(el)});
//   EngineManager::Instance()->exec("view", args);
// }

template <typename Numeric = double>
inline void view(Numeric axis) {
  matlab::data::Array args =
      EngineManager::Instance()->ArrayFactory()->createScalar<Numeric>(axis);
  EngineManager::Instance()->exec("view", args);
}

inline void view_from_z() { view(2); }

// // print(gcf,'MySavedPlot','-opengl','-r600','-dsvg')
// inline void save_to_svg(const std::string &name, int resolution = 300) {
//   std::string exec_code = "print(gcf,";
//   exec_code += embrance(name);
//   exec_code += ",";
//   // exec_code += embrance("-opengl");
//   // exec_code += ",";
//   exec_code += embrance("-r" + std::to_string(resolution));
//   exec_code += ",";
//   exec_code += embrance("-dsvg");
//   exec_code += ")";
//    EngineManager::Instance()->exec(exec_code);
// }

// // other useful functions
// std::vector<double> linstep(double start, double end, double step) {
//   std::vector<double> res;
//   for (double i = start; i <= end; i += step) {
//     res.push_back(i);
//   }
//    res;
// }

// std::vector<double> linspace(double start, double end, int space) {
//   double step = (start - end) / space;
//   return linstep(start, end, step);
// }

template <class NumericX = double, class NumericY = double,
          class NumericZ = double>
inline void Creat3DInterface(const std::vector<NumericX> &x,
                             const std::vector<NumericY> &y,
                             const std::vector<std::vector<NumericZ>> &z,
                             const std::string &func) {
  // note c++ is row-major, matlab is column-major
  std::vector<NumericZ> z_one_dim(z[0].size() * z.size());
  auto it = z_one_dim.begin();
  for (size_t i = 0; i < z.size(); i++) {
    std::copy(z[i].begin(), z[i].end(), it);
    it += z[i].size();
  }
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createArray(
           {x.size(), 1}, x.begin(), x.end()),
       EngineManager::Instance()->ArrayFactory()->createArray(
           {y.size(), 1}, y.begin(), y.end()),
       EngineManager::Instance()->ArrayFactory()->createArray(
           {z[0].size(), z.size()}, z_one_dim.begin(), z_one_dim.end())});
  EngineManager::Instance()->exec(func, args);
}

template <class NumericZ = double>
inline void Creat3DInterface(const std::vector<std::vector<NumericZ>> &z,
                             const std::string &func) {
  std::vector<NumericZ> z_one_dim(z.size() * z[0].size());
  auto it = z_one_dim.begin();
  for (size_t i = 0; i < z.size(); i++) {
    std::copy(z[i].begin(), z[i].end(), it);
    it += z[i].size();
  }

  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createArray(
          {z[0].size(), z.size()}, z_one_dim.begin(), z_one_dim.end())});
  EngineManager::Instance()->exec(func, args);
}

template <class NumericX = double, class NumericY = double,
          class NumericZ = double>
inline void surf(const std::vector<NumericX> &x, const std::vector<NumericY> &y,
                 const std::vector<std::vector<NumericZ>> &z) {
  Creat3DInterface(x, y, z, __FUNCTION__);
}

template <class NumericZ = double>
inline void surf(const std::vector<std::vector<NumericZ>> &z) {
  Creat3DInterface(z, __FUNCTION__);
}

template <class NumericX = double, class NumericY = double,
          class NumericZ = double>
inline void surfc(const std::vector<NumericX> &x,
                  const std::vector<NumericY> &y,
                  const std::vector<std::vector<NumericZ>> &z) {
  Creat3DInterface(x, y, z, __FUNCTION__);
}

template <class NumericZ = double>
inline void surfc(const std::vector<std::vector<NumericZ>> &z) {
  Creat3DInterface(z, __FUNCTION__);
}

template <class NumericX = double, class NumericY = double,
          class NumericZ = double>
inline void mesh(const std::vector<NumericX> &x, const std::vector<NumericY> &y,
                 const std::vector<std::vector<NumericZ>> &z) {
  Creat3DInterface(x, y, z, __FUNCTION__);
}

template <class NumericZ = double>
inline void mesh(const std::vector<std::vector<NumericZ>> &z) {
  Creat3DInterface(z, __FUNCTION__);
}

template <class NumericX = double, class NumericY = double,
          class NumericZ = double>
inline void quiver3(NumericX x0, NumericX x1, NumericY y0, NumericY y1,
                    NumericZ z0, NumericZ z1) {
  std::vector<matlab::data::Array> args(
      {EngineManager::Instance()->ArrayFactory()->createScalar<NumericX>(x0),
       EngineManager::Instance()->ArrayFactory()->createScalar<NumericX>(x1),
       EngineManager::Instance()->ArrayFactory()->createScalar<NumericY>(y0),
       EngineManager::Instance()->ArrayFactory()->createScalar<NumericY>(y1),
       EngineManager::Instance()->ArrayFactory()->createScalar<NumericZ>(z0),
       EngineManager::Instance()->ArrayFactory()->createScalar<NumericZ>(z1),
       EngineManager::Instance()->ArrayFactory()->createScalar<double>(0)});
  EngineManager::Instance()->exec("quiver3", args);
}

}  // namespace mplt

#endif  // INCLUDE_MATLAB_PLOT_CPLUSPLUS