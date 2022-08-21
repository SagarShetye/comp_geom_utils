#include "GeomUtils.h"

namespace GeomUtils{
	// Adding 2 position vectors (CGAL::Point_3)
	Point_3 operator+(const Point_3 &pt_1, const Point_3 &pt_2) {
		return Point_3(pt_1.x() + pt_2.x(), pt_1.y() + pt_2.y(), pt_1.z() + pt_2.z());
	}

	// Adding 2 position vectors (CGAL::Point_3)
	Point_3& operator+=(Point_3 &pt_1, const Point_3 &pt_2) {
		//Point_3 res(pt_1.x() + pt_2.x(), pt_1.y() + pt_2.y(), pt_1.z() + pt_2.z());
		//return res;

		pt_1 = Point_3(pt_1.x() + pt_2.x(), pt_1.y() + pt_2.y(), pt_1.z() + pt_2.z());
		return pt_1;
	}

	//// Subtracting 2 position vectors (CGAL::Point_3)
	//Point_3 MeshUtils::operator-(const Point_3 &pt_1, const Point_3 &pt_2) {
	//	return Point_3(pt_1.x() - pt_2.x(), pt_1.y() - pt_2.y(), pt_1.z() - pt_2.z());
	//}
	//
	//// Subtracting 2 position vectors (CGAL::Point_3)
	//Point_3 MeshUtils::operator-=(const Point_3 &pt_1, const Point_3 &pt_2) {
	//	return Point_3(pt_1.x() - pt_2.x(), pt_1.y() - pt_2.y(), pt_1.z() - pt_2.z());
	//}

	// Pre-Multiplying a position vector with a scalar
	Point_3 operator*(double scalar, const Point_3 &pt_1) {
		return Point_3(scalar * pt_1.x(), scalar * pt_1.y(), scalar * pt_1.z());
	}

	// Post-Multiplying a position vector with a scalar
	Point_3 operator*(const Point_3 &pt_1, double scalar) {
		return Point_3(scalar * pt_1.x(), scalar * pt_1.y(), scalar * pt_1.z());
	}

	// Divide a position vector with a scalar
	Point_3 operator/(const Point_3 &pt_1, double scalar) {
		return Point_3(pt_1.x() / scalar, pt_1.y() / scalar, pt_1.z() / scalar);
	}
}