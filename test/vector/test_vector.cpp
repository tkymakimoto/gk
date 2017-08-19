/*
 * test_vector.cpp
 *
 *  Created on: 2017/08/14
 *      Author: Takuya
 */

#include <cstdlib>
#include <iostream>
#include <gkvector.h>

int main(int argc, char **argv) {

	gk::vector<double, gk::GK::GK_3D> u(2.0, 3.0, -5.0);
	gk::vector<double, gk::GK::GK_3D> v(-1.5, -4.5, 2.5);
	gk::vector<double, gk::GK::GK_3D> w = u + v;

	std::cout << w << std::endl;

//	gk::product<double, double> a = gk::dot(u, v);

	exit(EXIT_SUCCESS);
}

