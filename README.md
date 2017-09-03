# gk - geometry kernel

gk is a C++ library to compute some geometries.

# Installation & Usage

It is very easy that you install gk in your programs. gk is written in the header files "*.h" so that you need not compile the source files of gk. 

When you actually use gk, you only include the header file **gk.h** following the code.

```cpp
#include <gk.h>

#include <cstdlib>

int main(int argc, char** argv) {
	gk::vector<double, gk::GK::GK_3D> p(0.0, 0.0, 0.0);
	gk::vector<double, gk::GK::GK_3D> q(1, 0, 1.0, 2.0);

	gk::line<double, gk::GK::GK_3D> l(p, q);

	exit(EXIT_SUCCESS);
}
```
