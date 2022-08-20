//#pragma once
// For Delaunay triangulation via incremental construction
#ifndef __DELAUNAY_TRIANGULATION__
#define __DELAUNAY_TRIANGULATION__
#include <CGAL/boost/graph/Euler_operations.h>

namespace delaunay {
	bool triangulate_incremental(MeshUtils::Halfedge_mesh &mesh2d, const std::vector<Point_3> &points);
	// TODO: Implement the pure flipping approach and compare the 2 algorithms
}

#endif