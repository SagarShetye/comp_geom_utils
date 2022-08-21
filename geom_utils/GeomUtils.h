#ifndef __MESH_UTILS__
#define __MESH_UTILS__

#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Simple_cartesian.h>

#include <math.h>
#include <algorithm>

// Useful aliases
using CGAL_double = CGAL::Simple_cartesian<double>;
using Point_3 = CGAL_double::Point_3;
using Vector_3 = CGAL::Vector_3<CGAL_double>;
using Ray_3 = CGAL_double::Ray_3;
//using reference = CGAL::Surface_mesh<Point_3>::reference;

namespace GeomUtils {
	// Tolerance for 3D distance purposes
	static double dist_tol = 1.0e-6;

	// Tolerance for checking zeros
	static double zero_tol = 1.0e-11;

	inline void normalise(Vector_3 &vec) {
		double sq_length = vec.squared_length();
		if (sq_length > 0)
			vec = vec / std::sqrt(sq_length);
	}

	// TODO: Add this in a more appropriate location
	// Helper functions to manipulate CGAL::Point_3s
	Point_3 operator+(const Point_3 &pt_1, const Point_3 &pt_2);

	Point_3& operator+=(Point_3 &pt_1, const Point_3 &pt_2);

	Point_3 operator*(double scalar, const Point_3 &pt_1);

	Point_3 operator*(const Point_3 &pt_1, double scalar);

	Point_3 operator/(const Point_3 &pt_1, double scalar);

	// Clamp a value between its legal bounds
	template <typename T>
	T clamp(const T& n, const T& lower, const T& upper) {
		return std::max(lower, std::min(n, upper));
	}
}

#endif // __MESH_UTILS__
