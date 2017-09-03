# gk - geometry kernel

gk is a C++ library to compute some geometries.

# Installation & Usage

It is very easy that you install gk in your programs. gk is written in the header files "*.h" so that you need not compile the source files of gk. 

When you actually use gk, you only include the header file **gk.h** following the code.

```cpp
#include <cstdlib>
#include <iostream>

#include <gk.h>

int main(int argc, char** argv) {

	gk::vector<double, gk::GK::GK_3D> u(2.0, 3.0, -5.0);
	gk::vector<double, gk::GK::GK_3D> v(-1.5, -4.5, 2.5);
	gk::vector<double, gk::GK::GK_3D> w = u + v;

	std::cout << w << std::endl;

	exit(EXIT_SUCCESS);
}
```
