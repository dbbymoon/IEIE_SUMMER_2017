#include<cmath>

using namespace std;

struct Point {
	double x, y;
};

const double PI = std::acos(-1);							// = 3.141592....

double u_dist(const Point& p1, const Point& p2) {
	double distance;
	Point a, b;

	a.x = p1.x * cos(p1.y * PI / 180);
	a.y = p1.x * sin(p1.y * PI / 180);
	b.x = p2.x * cos(p2.y * PI / 180);
	b.y = p2.x * sin(p2.y * PI / 180);

	distance = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));	 // uclidian distance
	return distance;
}