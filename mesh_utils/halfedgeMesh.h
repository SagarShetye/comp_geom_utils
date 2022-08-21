//#pragma once

#ifndef _HALFEDGE_MESH_
#define _HALFEDGE_MESH_

#include "GeomUtils.h"

#include <CGAL/Surface_mesh.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Bbox_3.h>
#include <Eigen/Dense>

#include <unordered_map>

namespace MeshUtils {
	using vertex_descriptor = CGAL::Surface_mesh<Point_3>::Vertex_index;

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
