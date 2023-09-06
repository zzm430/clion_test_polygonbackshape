#ifndef SINGLETON_HPP
#define SINGLETON_HPP

namespace common {

class Singleton {
public:
    template <typename T, typename... Args>
    static T& GetInstance(Args... args) {
      static T value(std::forward<Args>(args)...);
      return value;
    }

    template <typename U>
    static U& GetInstance() {
      static U value;
      return value;
    }

    Singleton(Singleton const&) = delete;
    Singleton& operator=(Singleton const&) = delete;

    Singleton(Singleton&&) = delete;
    Singleton& operator=(Singleton&&) = delete;

protected:
    Singleton();
    ~Singleton();
};

}  // namespace common

#endif
