#ifndef ABSTRACTARGUMENT_H
#define ABSTRACTARGUMENT_H

#include <exception>
#include <memory>
#include <cxxabi.h>

/**
 * @brief The AbstractArgument class
 */
namespace Constraints {

/**
 * @brief demangle
 * @param name
 * @return
 */
std::string demangle(const char* name) {
    int status;
    // enable c++11 by passing the flag -std=c++11 to g++
    std::unique_ptr<char, void(*)(void*)> res {
        abi::__cxa_demangle(name, NULL, NULL, &status),
        std::free
    };
    return (status==0) ? res.get() : name;
}

/**
 *
 */
template<typename ArgType, typename Constraint>
bool assertValid(const ArgType& arg, const Constraint& constraint) {
  if (!constraint(arg)) {
    throw std::invalid_argument(std::string("failed constraint ") + demangle(typeid(Constraint).name()));
  }
  return true;
}

/**
 *
 */
template <typename ArgType, typename Constraint, typename... Constraints>
bool assertValid(const ArgType& arg, const Constraint& constraint, const Constraints&... args) {
  return constraint(arg) && assertValid(arg, args...);
}

/**
 *
 */
template<typename ArgType, typename Constraint>
bool isValid(const ArgType& arg, const Constraint& constraint) {
  return constraint(arg);
}

/**
 *
 */
template <typename ArgType, typename Constraint, typename... Constraints>
bool isValid(const ArgType& arg, const Constraint& constraint, const Constraints&... args) {
  return constraint(arg) && isValid(arg, args...);
}

/**
 *
 */
template<typename Arg1Type, typename Arg2Type, typename Constraint>
bool assertCompatible(const Arg1Type& arg1, const Arg2Type& arg2, const Constraint& constraint) {
  if (!constraint(arg1, arg2)) {
    throw std::invalid_argument(std::string("failed constraint ") + demangle(typeid(Constraint).name()));
  }
  return true;
}

/**
 *
 */
template <typename Arg1Type, typename Arg2Type, typename Constraint, typename... Constraints>
bool assertCompatible(const Arg1Type& arg1, const Arg2Type& arg2, const Constraint& constraint, const Constraints&... args) {
  return constraint(arg1,arg2) && assertCompatible(arg1, arg2, args...);
}

/**
 *
 */
template<typename Arg1Type, typename Arg2Type, typename Constraint>
bool isCompatible(const Arg1Type& arg1, const Arg2Type& arg2, const Constraint& constraint) {
  return constraint(arg1, arg2);
}

/**
 *
 */
template <typename Arg1Type, typename Arg2Type, typename Constraint, typename... Constraints>
bool isCompatible(const Arg1Type& arg1, const Arg2Type& arg2, const Constraint& constraint, const Constraints&... args) {
  return constraint(arg1, arg2) && isCompatible(arg1, arg2, args...);
}

};

#endif // ABSTRACTARGUMENT_H
