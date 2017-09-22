/*
 * test_stl.cpp
 *
 *  Created on: 2017/09/20
 *      Author: J0115775
 */

#include <cstdlib>
#include <iostream>
#include <vector>
#include <primitive/triangle.h>
#include <stl/stl.h>

int main(int argc, char** argv) {

	std::vector<gk::triangle<float, gk::GK::GK_3D> > V;

	gk::read_stl("./sample.stl");

	std::cout << "Finish this program." << std::endl;

	exit(EXIT_SUCCESS);
}
