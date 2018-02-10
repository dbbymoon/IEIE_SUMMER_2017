#include<cmath>

using namespace std;
extern double Normal(void);

struct Point {
	double x, y;
};

const double PI = std::acos(-1);		// = 3.141592....

// The distance the sensor node traveled during unit time
Point brown(const Point& p1, double distance) {
	Point a;
	double sigma;

	sigma = distance / sqrt(PI / 2);	// from Rayleigh distribution mean
	a.x = p1.x + Normal() * sigma;
	a.y = p1.y + Normal() * sigma;
	
	return a;
}