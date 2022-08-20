//#pragma once

#ifndef _HALFEDGE_MESH_
#define _HALFEDGE_MESH_

#include <CGAL/Surface_mesh.h>
#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Bbox_3.h>
#include <Eigen/Dense>

#include <unordered_map>

// Useful aliases
using CGAL_double = CGAL::Simple_cartesian<double>;
using Point_3 = CGAL_double::Point_3;
using Vector_3 = CGAL::Vector_3<CGAL_double>;
using Ray_3 = CGAL_double::Ray_3;
using vertex_descriptor = CGAL::Surface_mesh<Point_3>::Vertex_index;
//using reference = CGAL::Surface_mesh<Point_3>::reference;

namespace MeshUtils {
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

	// A convenient halfedge mesh container built upon CGALs 'Surface_mesh' type
	// Halfedge mesh class that also stores vertex and face normals
	// TODO: Check if CGAL inherently has something to do this
	//			Maybe make new vertex, edge, face containers inheriting from the CGAL bases
	class Halfedge_mesh : public CGAL::Surface_mesh<Point_3> {
	public:
		// Centroid of the mesh
		Point_3 centroid = Point_3(0.0, 0.0, 0.0);

		// Face normals
		std::unordered_map<Face_index, Vector_3> faceNormals;
		// Vertex normals
		std::unordered_map<Vertex_index, Vector_3> vertexNormals;

		// Some helpful functions
		// Get the faces around a vertex
		void FacesAroundVertex(Vertex_index vertIdx, std::vector<Face_index> &neighbourFaceIdxs) const;

		// Get all the connected vertices to the given vertex
		void VerticesConnectedToVertex(Vertex_index vertIdx, std::vector<Vertex_index> &connectedVertIdxs) const;

		// Get the normal of a face
		// TODO: Check if CGAL can do this on its own
		Vector_3 GetFaceNormal(Face_index faceIdx);

		// Get the vertex normal
		Vector_3 GetVertexNormal(Vertex_index vertIdx);

		// Get the centroid of a face
		Point_3 GetFaceCentroid(Face_index faceIdx);

		// Recompute the face normals
		void RecomputeFaceNormals();

		// Recompute the vertex normals
		void RecomputeVertexNormals();

		// Local operators
		// Triangulate a face, use the centroid of the face by default as the new vertex to be added
		// The new vertex position can also be passed in as an argument
		Vertex_index SubdivideFace(Face_index faceIdx, Point_3 *newPt = nullptr);

		// Split an edge and create a vertex at the mid-pt of the edge (For triangular meshes only)
		// Return the new vertex index
		// The default behaviour is to add the new vertex at the mid-pt of the old edge
		Vertex_index SplitEdge(Edge_index edgeIdx, Point_3 *newSplitPoint = nullptr);

		// Flip an edge within a face
		Edge_index FlipEdge(Edge_index edgeIdx, double areaTol = 1e-6);

		// TODO
		// Collapse an edge into a vertex
		Vertex_index CollapseEdge(Edge_index edgeIdx);
	};

}

namespace boost {
	template<>
	struct boost::graph_traits<MeshUtils::Halfedge_mesh> :
		public boost::graph_traits<CGAL::Surface_mesh<Point_3>>
	{};
} // namespace boost

#endif
