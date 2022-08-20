#include "bvh.h"

// Build the BVH according to faces
faceBVH::faceBVH(const MeshUtils::Halfedge_mesh &mesh) {
	faceTree.insert(mesh.faces_begin(), mesh.faces_end(), mesh);
	// Set up the k-D tree for the faces
	faceTree.accelerate_distance_queries();
}

// Get the first intersected face
MeshUtils::Halfedge_mesh::Face_index faceBVH::getFirstIntersectedFace(const Ray_3 &ray) {
	// Get the first intersected face by this ray
	Ray_face_intersection intersection = faceTree.first_intersection(ray);
	if (intersection)
		return intersection->second;

	return MeshUtils::Halfedge_mesh::null_face();
}

// Find the closest face to the ray
MeshUtils::Halfedge_mesh::Face_index faceBVH::findClosestFace(const Point_3 &queryPt) {
	Closest_pt_face_query closest = faceTree.closest_point_and_primitive(queryPt);

	return closest->second;
}

// Reconstruct the BVH
void faceBVH::rebuild(const MeshUtils::Halfedge_mesh &mesh) {
	faceTree.rebuild(mesh.faces_begin(), mesh.faces_end(), mesh);
	// Set up the k-D tree for the faces
	faceTree.accelerate_distance_queries();
}

// Build the bvh based on the edges
edgeBVH::edgeBVH(const MeshUtils::Halfedge_mesh &mesh) {
	edgeTree.insert(mesh.edges_begin(), mesh.edges_end(), mesh);
}

// Get the first intersected edge
MeshUtils::Halfedge_mesh::Edge_index edgeBVH::getFirstIntersectedEdge(const Ray_3 &ray) {
	boost::optional<MeshUtils::Halfedge_mesh::Edge_index> edgePrimId = edgeTree.first_intersected_primitive(ray);
	// Get the first intersected edge by this ray
	Ray_edge_intersection intersection = edgeTree.first_intersection(ray);
	if (intersection)
		return intersection->second;

	return MeshUtils::Halfedge_mesh::null_edge();
}

// Reconstruct the BVH
void edgeBVH::rebuild(const MeshUtils::Halfedge_mesh &mesh) {
	edgeTree.rebuild(mesh.edges_begin(), mesh.edges_end(), mesh);
}


