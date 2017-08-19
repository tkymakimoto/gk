/*
 * version.h
 *
 *  Created on: 2013/04/02
 *      Author: Takuya Makimoto
 */

#ifndef VERSION_H_
#define VERSION_H_

#include <string>
#include <sstream>

#define GK_API_MAJOR 0
#define GK_API_MINOR 0
#define GK_API_PATCH 0

namespace gk {

/**
 * @brief The version of gk.
 * @author Takuya Makimoto
 * @date 2015/12/14
 */
struct version {
	static size_t major() {
		return GK_API_MAJOR;
	}

	static size_t minor() {
		return GK_API_MINOR;
	}

	static size_t patch() {
		return GK_API_PATCH;
	}

	static std::string version() {
		std::stringstream ss;
		ss << GK_API_MAJOR << '.' << GK_API_MINOR << '.' << GK_API_PATCH;
		return ss.str();
	}

	static bool isLeast(size_t major, size_t minor, size_t patch) {
		return (GK_API_MAJOR == major) && (GK_API_MINOR == minor)
				&& (GK_API_PATCH == patch);
	}
};

}  // namespace gk

#endif /* VERSION_H_ */
