/*
 * stl.h
 *
 *  Created on: 2017/09/19
 *      Author: J0115775
 */

#ifndef STL_STL_H_
#define STL_STL_H_

#include "../primitive/triangle.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "../primitive/triangle.h"

namespace gk {

#define GK_STL_HEADER_BYTES 80
#define GK_STL_VALUE_BYTES 4

//template<typename OutputIterator>
void read_stl(const char* filename) {

	std::ifstream ifs(filename, std::ios_base::in | std::ios_base::binary);

	std::stringstream ss;

	char buffer_comment[GK_STL_HEADER_BYTES];
	ifs.read(buffer_comment, sizeof(buffer_comment));

	std::cout << buffer_comment << std::endl;

	std::basic_ifstream<uint32_t> ifs_uint32_t(filename,
			std::ios_base::in | std::ios_base::binary);

//	std::cout << "ifs.tellg() = " << ifs.tellg() << std::endl;

	uint32_t size_of_cells;
	ifs.read(reinterpret_cast<char*>(&size_of_cells), sizeof(uint32_t));

	std::cout << "The number of cells : " << size_of_cells << std::endl;

#if __SIZEOF_DOUBLE__ == GK_STL_VALUE_BYTES
	typedef double value_type;
#else
	typedef float value_type;
#endif
	for (uint32_t i = 0; i < size_of_cells; ++i) {
		value_type dx;
		value_type dy;
		value_type dz;
		ifs.read(reinterpret_cast<char*>(&dx), sizeof(value_type));
		ifs.read(reinterpret_cast<char*>(&dy), sizeof(value_type));
		ifs.read(reinterpret_cast<char*>(&dz), sizeof(value_type));

		value_type x1;
		value_type y1;
		value_type z1;
		ifs.read(reinterpret_cast<char*>(&x1), sizeof(value_type));
		ifs.read(reinterpret_cast<char*>(&y1), sizeof(value_type));
		ifs.read(reinterpret_cast<char*>(&z1), sizeof(value_type));
		vector<value_type, GK::GK_3D> v1(x1, y1, z1);

		value_type x2;
		value_type y2;
		value_type z2;
		ifs.read(reinterpret_cast<char*>(&x2), sizeof(value_type));
		ifs.read(reinterpret_cast<char*>(&y2), sizeof(value_type));
		ifs.read(reinterpret_cast<char*>(&z2), sizeof(value_type));
		vector<value_type, GK::GK_3D> v2(x2, y2, z2);

		value_type x3;
		value_type y3;
		value_type z3;
		ifs.read(reinterpret_cast<char*>(&x3), sizeof(value_type));
		ifs.read(reinterpret_cast<char*>(&y3), sizeof(value_type));
		ifs.read(reinterpret_cast<char*>(&z3), sizeof(value_type));
		vector<value_type, GK::GK_3D> v3(x3, y3, z3);

		uint16_t color;
		ifs.read(reinterpret_cast<char*>(&color), sizeof(uint16_t));

		std::cout << "n(" << dx << ", " << dy << ", " << dz << ")" << std::endl;
		std::cout << "p1(" << x1 << ", " << y1 << ", " << z1 << ")"
				<< std::endl;
		std::cout << "p2(" << x2 << ", " << y2 << ", " << z2 << ")"
				<< std::endl;
		std::cout << "p3(" << x3 << ", " << y3 << ", " << z3 << ")"
				<< std::endl;
		std::cout << "color = " << color << std::endl;
		std::cout << std::endl;

		triangle<value_type, GK::GK_3D> cell(v1, v2, v3);
	}

	ifs.close();
}

}  // namespace gk

#endif /* STL_STL_H_ */
