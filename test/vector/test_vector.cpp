/*
 * test_vector.cpp
 *
 *  Created on: 2017/08/14
 *      Author: Takuya
 */

#include <cstdlib>
#include <iostream>

#include <gk.h>

int main(int argc, char** argv) {

	gk::vector<double, gk::GK::GK_3D> u(2.0, 3.0, -5.0);
	gk::vector<double, gk::GK::GK_3D> v(-1.5, -4.5, 2.5);
	gk::vector<double, gk::GK::GK_3D> w = u + v;

	std::cout << w << std::endl;

	gk::version::major_value();

	gk::product<double, float> t_double = float(4.3);
	std::cout << t_double << std::endl;

	exit(EXIT_SUCCESS);
}

