#include<cmath>

double unirv(void) {
	/* function to return uniform randon variable [0,1] */
	return rand() / ((double)RAND_MAX + 1);
}