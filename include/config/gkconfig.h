/*
 * gkconfig.h
 *
 *  Created on: 2014/03/07
 *      Author: Takuya Makimoto
 */

#ifndef GKCONFIG_H_
#define GKCONFIG_H_

#include <cstdlib>
#ifdef GK_DEBUG
#include <iostream>
#endif

/*
 * Debug
 */

/*
 * Int Type
 */
#ifndef GK_SIZEOF_INT
#	define GK_SIZEOF_INT 4
#endif

/*
 * Floating Type
 */
#ifndef GK_SIZEOF_FLOAT
#	define GK_SIZEOF_FLOAT 8
#endif

#ifndef GK_FUNCTION_NAME
#	if defined(__PRETTY_FUNCTION__)
#		define __PRETTY_FUNCTION__ GK_FUNCTION_NAME
#	else
#		define __func__ GK_FUNCTION_NAME
#	endif
#endif

#ifndef GK_DEBUG
#	define gk_static_assert(condition)
#else
namespace gk {

	inline void gk_replacement_assert(const char* file, int line,
			const char* function, const char* condition) {
		std::cout << file << ":" << line << ":" << " Assertion " << condition
		<< "failed." << std::endl;
		std::abort();
	}

#	define gk_static_assert(condition) \
	do { \
		if (! (condition)) {\
			gk::gk_replacement_assert(__FILE__, __LINE__, GK_FUNCTION_NAME, #condition); \
		} \
	}while (false)

}  // namespace gk
#endif

#endif /* GKCONFIG_H_ */
