#ifndef  OBJECT_HPP
#define  OBJECT_HPP
//用于数据的基础属性

namespace common {

class Object {
public:
  Object() : valid_(false), repeat_(false) {}

  virtual bool is_valid() const {
    return valid_;
  }

  virtual bool is_invalid() const {
    return !valid_;
  }

  virtual void set_valid() {
    valid_ = true;
  }

  virtual void set_invalid() {
    valid_ = false;
  }

  virtual bool is_repeat() const {
    return repeat_;
  }

  virtual bool is_unrepeat() const {
    return !repeat_;
  }

  virtual void set_repeat() {
    repeat_ = true;
  }

  virtual void set_unrepeat() {
    repeat_ = false;
  }

  virtual bool operator== (const Object &other) const {
    return valid_ == other.valid_;
  }

  bool valid_;
  bool repeat_;
};

}


