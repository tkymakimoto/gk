#include <gkprimitive.h>

#include <cstdlib>

int main(int argc, char** argv) {
	gk::vector<double, gk::GK::GK_3D> p(0.0, 0.0, 0.0);
	gk::vector<double, gk::GK::GK_3D> q(1, 0, 1.0, 2.0);

	gk::line<double, gk::GK::GK_3D> l(p, q);

	exit(EXIT_SUCCESS);
}

