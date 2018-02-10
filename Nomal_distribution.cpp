
#include<cmath>

extern double unirv(void);

double Normal(void) {
	int k;
	double x, y, z;

	y = 0;
	for (k = 1; k <= 100; k++) {
		x = unirv();
		y += x;
	}
	z = sqrt(3.0) * (y - 50) / 5;
	return z;
}
