#pragma once

#ifndef __BVH__
#define __BVH__
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>

#include "halfedgeMesh.h"

// For fast ray-primitive intersections
typedef CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<Point_3>> Face_Primitive;
typedef CGAL::AABB_traits<CGAL_double, Face_Primitive> Face_Traits;
typedef CGAL::AABB_tree<Face_Traits> Face_Tree;
typedef boost::optional<Face_Tree::Intersection_and_primitive_id<Ray_3>::Type> Ray_face_intersection;

// To find the closest primitive to the given point
typedef boost::optional<Face_Tree::Point_and_primitive_id> Closest_pt_face_query;

typedef CGAL::AABB_halfedge_graph_segment_primitive<CGAL::Surface_mesh<Point_3>> Edge_Primitive;
typedef CGAL::AABB_traits<CGAL_double, Edge_Primitive> Edge_Traits;
typedef CGAL::AABB_tree<Edge_Traits> Edge_Tree;
typedef boost::optional<Edge_Tree::Intersection_and_primitive_id<Ray_3>::Type> Ray_edge_intersection;

// TODO: Add the ray-primitive intersection stuff to a separate file
// For fast computation of ray-face intersections
class faceBVH {
private:
	// For fast ray-face intersection
	Face_Tree faceTree;

public:
	faceBVH() {};
	faceBVH(const MeshUtils::Halfedge_mesh &mesh);
	// Query the first intersected face (for picking)
	MeshUtils::Halfedge_mesh::Face_index getFirstIntersectedFace(const Ray_3 &ray);
	// TODO: Find the closest primitive to the ray
	MeshUtils::Halfedge_mesh::Face_index findClosestFace(const Point_3 &queryPt);
	void rebuild(const MeshUtils::Halfedge_mesh &mesh);
};

// For fast ray-edge intersections
class edgeBVH {
private:
	Edge_Tree edgeTree;

public:
	edgeBVH() {};
	edgeBVH(const MeshUtils::Halfedge_mesh &mesh);

	// Query the first intersected edge (for picking)
	MeshUtils::Halfedge_mesh::Edge_index getFirstIntersectedEdge(const Ray_3 &ray);

	// Reconstruct the BVH on modification of the mesh
	void rebuild(const MeshUtils::Halfedge_mesh &mesh);
};

// BVH containers for fast ray-primitive intersections

#endif // !__BVH__
