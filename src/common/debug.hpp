#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include <stdexcept>

// 不要用assert,因為assert在release mode中無法工作
inline void ensure(bool cond, const char* msg) {
	if(!cond)
	{
			throw std::runtime_error(std::string("[ERROR]") + " " + msg);
	}
}
#endif  // DEBUG_HPP_