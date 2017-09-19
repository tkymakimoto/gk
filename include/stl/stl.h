/*
 * stl.h
 *
 *  Created on: 2017/09/19
 *      Author: J0115775
 */

#ifndef STL_STL_H_
#define STL_STL_H_

#include "../primitive/triangle.h"

#include <fstream>
#include <sstream>
#include <vector>

namespace gk {

#define GK_STL_HEADER_BYTES 80
#define GK_STL_VALUE_BYTES 4

template<typename OutputIterator>
void read_stl(const char* filename, OutputIterator begin, OutputIterator end) {

#if __SIZEOF_FLOAT__ == GK_STL_VALUE_Bytes
	typedef float value_type;

#else __SIZEOF_DOUBLE__ == GK_STL_VALUE_Bytes
	typedef double value_type;
#endif

	std::ifstream ifs(filename, std::ios_base::binary);

	std::stringstream ss;

	char buffer_comment[GK_STL_HEADER_BYTES];
	ifs.read(buffer_comment, sizeof(buffer_comment));
	const char* STL_Comment = buffer_comment;

	char buffer_size[sizeof(uint32_t)];
	ifs.read(buffer_size, sizeof(uint32_t));
	ss.str(buffer_size);
	const std::uint32_t Size_of_Cells = ss;
}

}  // namespace gk

#endif /* STL_STL_H_ */
