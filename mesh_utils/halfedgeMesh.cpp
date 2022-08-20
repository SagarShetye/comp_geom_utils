#include "halfedgeMesh.h"

using namespace MeshUtils;


// Adding 2 position vectors (CGAL::Point_3)
Point_3 MeshUtils::operator+(const Point_3 &pt_1, const Point_3 &pt_2) {
	return Point_3(pt_1.x() + pt_2.x(), pt_1.y() + pt_2.y(), pt_1.z() + pt_2.z());
}

// Adding 2 position vectors (CGAL::Point_3)
Point_3& MeshUtils::operator+=(Point_3 &pt_1, const Point_3 &pt_2) {
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
Point_3 MeshUtils::operator*(double scalar, const Point_3 &pt_1) {
	return Point_3(scalar * pt_1.x(), scalar * pt_1.y(), scalar * pt_1.z());
}

// Post-Multiplying a position vector with a scalar
Point_3 MeshUtils::operator*(const Point_3 &pt_1, double scalar) {
	return Point_3(scalar * pt_1.x(), scalar * pt_1.y(), scalar * pt_1.z());
}

// Divide a position vector with a scalar
Point_3 MeshUtils::operator/(const Point_3 &pt_1, double scalar) {
	return Point_3(pt_1.x() / scalar, pt_1.y() / scalar, pt_1.z() / scalar);
}

//// Normalize a vector
//inline void MeshUtils::normalise(Vector_3 &vec) {
//	double sq_length = vec.squared_length();
//	if (sq_length > 0)
//		vec = vec / std::sqrt(sq_length);
//}


// Subdivide a face by triangulation, use the centroid of the face by default as the new vertex to be added
// The new vertex position can also be passed in as an argument
Halfedge_mesh::Vertex_index Halfedge_mesh::SubdivideFace(Face_index faceIdx, Point_3 *newPt) {
	// The position of the new vertex to be added (default is the centroid of the face)
	Point_3 newPos(0.0, 0.0, 0.0);
	if (newPt == nullptr) {
		// Calculate the centroid of the face
		newPos = GetFaceCentroid(faceIdx);
	}
	else
		newPos = *newPt;

	// Make a new vertex corr. to the new position
	Vertex_index newVert = add_vertex(newPos);

	// The new faces
	std::vector<Face_index> newFaces;
	// Halfedges on the new edges
	std::vector<Halfedge_index> newHalfedges;
	// Collecting the halfedges and vertices in the original face
	std::vector<Halfedge_index> oldHalfedges;
	std::vector<Vertex_index> vertices;
	Halfedge_index hf = halfedge(faceIdx);
	Halfedge_index currhf = hf;
	do {
		Vertex_index currVert = source(currhf);
		oldHalfedges.push_back(currhf);
		vertices.push_back(currVert);
		newFaces.push_back(add_face());						// make a new face here, assign halfedges and everything else later on
		newHalfedges.push_back(add_edge(newVert, currVert));
		currhf = next(currhf);
	} while (currhf != hf);

	// Connecting the new halfedges with the old ones
	size_t numVerts = vertices.size();
	for (size_t i = 0; i < numVerts; ++i) {
		Halfedge_index newHf = newHalfedges[i];
		Halfedge_index oldHf = oldHalfedges[i];
		Face_index newFace = newFaces[i];

		// Old halfedge - New halfedge
		set_next(newHf, oldHf);
		// New halfedge - New face
		set_face(newHf, newFace);
		set_halfedge(newFace, newHf);

		// Old halfedge - New face
		set_face(oldHf, newFace);

		// New-halfedge - New face
		set_face(opposite(newHalfedges[(i + 1) % numVerts]), newFace);

		// New halfedge - Old halfedge
		set_next(oldHf, opposite(newHalfedges[(i + 1) % numVerts]));
		// New halfedge - New halfedge
		set_next(opposite(newHalfedges[(i + 1) % numVerts]), newHf);
		// New halfedge - Old vertex
		set_halfedge(vertices[i], newHf);
		set_target(newHf, vertices[i]);
	}

	//// Set the incoming halfedge for the new vertex
	//set_target(opposite(newHalfedges[0]), newVert);
	// Se the correct incoming halfedge to the new vertex
	set_halfedge(newVert, opposite(newHalfedges[0]));

//// TEMP
//	auto hhf = halfedge(newVert);
//	for (auto hf : oldHalfedges) {
//		auto fc = face(hf);
//		bool isBorder = is_border(opposite(hf));
//		auto oppFace = face(opposite(hf));
//
//		int r = 0;
//	}

	// Add new normal information to the face normals map
	for (auto newFace : newFaces) {
		this->faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(newFace, this->GetFaceNormal(newFace)));
	}

	// Remove the face from the normals map
	this->faceNormals.erase(faceIdx);

	// Delete the old face
	remove_face(faceIdx);

	// Adding the new vertex normal information
	this->vertexNormals.insert(std::pair<Halfedge_mesh::Vertex_index, Vector_3>(newVert, this->GetVertexNormal(newVert)));

	return newVert;
}


// Split an edge and create a vertex at the mid-pt of the edge (For triangular meshes only)
// Return the new vertex index
Halfedge_mesh::Vertex_index Halfedge_mesh::SplitEdge(Edge_index edgeIdx, Point_3 *newSplitPoint) {
	Halfedge_index hfIdx = this->halfedge(edgeIdx);			// An halfedge corr. to the edge
	Halfedge_index oppHfIdx = this->opposite(hfIdx);		// Opposite HF

															// Check to make sure that the degree of the adjacent faces is 3
	if ((this->degree(this->face(hfIdx)) > 3) || (this->degree(this->face(oppHfIdx)) > 3))
		return null_vertex();

	// The start and end vertices of the edge
	const Point_3 startPos = this->point(this->vertex(edgeIdx, 0));
	const Point_3 endPos = this->point(this->vertex(edgeIdx, 1));
	// Get the mid-pt of the edge, this will result in the new vertex
	Point_3 midPoint((startPos.x() + endPos.x()) / 2.0, (startPos.y() + endPos.y()) / 2.0, (startPos.z() + endPos.z()) / 2.0);

	// Collecting the current mesh elements
	// Half-edges
	Halfedge_index h0 = this->halfedge(edgeIdx);
	Halfedge_index h1 = this->next(h0);
	Halfedge_index h2 = this->next(h1);
	Halfedge_index h3 = this->opposite(h0);
	Halfedge_index h4 = this->next(h3);
	Halfedge_index h5 = this->next(h4);

	// Faces
	Face_index f0 = this->face(h0);
	Face_index f1 = this->face(h3);

	// Vertices 
	Vertex_index v0 = this->source(h3);			// On edge to split
	Vertex_index v1 = this->source(h2);
	Vertex_index v2 = this->source(h0);			// On edge to split
	Vertex_index v3 = this->source(h5);

	// Creating new vertex
	// Check if a position for the new vertex has been provided
	Vertex_index v4 = (newSplitPoint == nullptr) ? this->add_vertex(Point_3((startPos.x() + endPos.x()) / 2.0, (startPos.y() + endPos.y()) / 2.0, (startPos.z() + endPos.z()) / 2.0))
													: this->add_vertex(*newSplitPoint);

	// Creating new edges (and half-edges)
	Halfedge_index hn1 = this->add_edge(v1, v4);
	Halfedge_index hn4 = this->opposite(hn1);

	Halfedge_index hn2 = this->add_edge(v4, v0);
	Halfedge_index hn7 = this->opposite(hn2);

	Halfedge_index hn3 = this->add_edge(v2, v4);
	Halfedge_index hn6 = this->opposite(hn3);

	Halfedge_index hn5 = this->add_edge(v3, v4);
	Halfedge_index hn8 = this->opposite(hn5);

	// Creating new faces
	Face_index fn1 = this->add_face();
	Face_index fn2 = this->add_face();
	Face_index fn3 = this->add_face();
	Face_index fn4 = this->add_face();

	// Reassigning the mesh elements
	// Half-edges
	this->set_face(hn1, fn1);
	this->set_next(hn1, hn2);

	this->set_face(hn2, fn1);
	this->set_next(hn2, h1);

	this->set_face(hn3, fn2);
	this->set_next(hn3, hn4);

	this->set_face(hn4, fn2);
	this->set_next(hn4, h2);

	this->set_face(hn5, fn3);
	this->set_next(hn5, hn6);

	this->set_face(hn6, fn3);
	this->set_next(hn6, h4);

	this->set_face(hn7, fn4);
	this->set_next(hn7, hn8);

	this->set_face(hn8, fn4);
	this->set_next(hn8, h5);

	// Original edges
	this->set_next(h1, hn1);
	this->set_face(h1, fn1);

	this->set_next(h2, hn3);
	this->set_face(h2, fn2);

	this->set_next(h4, hn5);
	this->set_face(h4, fn3);

	this->set_next(h5, hn7);
	this->set_face(h5, fn4);

	// Setting the halfedges for the vertices
	// The halfedge is the incoming one for the vertex
	// i.e. the halfedge ends at the vertex
	this->set_halfedge(v4, this->opposite(hn2));
	this->set_halfedge(v0, this->opposite(hn7));
	this->set_halfedge(v1, this->opposite(hn1));
	this->set_halfedge(v2, this->opposite(hn3));
	this->set_halfedge(v3, this->opposite(hn5));

	// Assigning the correct halfedges to the faces
	this->set_halfedge(fn1, hn1);
	this->set_halfedge(fn2, hn3);
	this->set_halfedge(fn3, hn5);
	this->set_halfedge(fn4, hn7);

	// Deleting old elements

	// Deleting the edge that is split
	// (It also deletes the corresponding halfedges)
	this->remove_edge(edgeIdx);

	// Faces
	// Remove the faces from the normals map
	this->faceNormals.erase(f0);
	this->faceNormals.erase(f1);
	this->remove_face(f0);
	this->remove_face(f1);

	// Adding the normal information for the new faces
	this->faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(fn1, this->GetFaceNormal(fn1)));
	this->faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(fn2, this->GetFaceNormal(fn2)));
	this->faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(fn3, this->GetFaceNormal(fn3)));
	this->faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(fn4, this->GetFaceNormal(fn4)));

	// Adding the new vertex normal information
	this->vertexNormals.insert(std::pair<Halfedge_mesh::Vertex_index, Vector_3>(v4, this->GetVertexNormal(v4)));

	return v4;

	return null_vertex();
}

// Flip an edge within a face
// Default area tolerance is 1e-6
Halfedge_mesh::Edge_index Halfedge_mesh::FlipEdge(Edge_index edgeIdx, double areaTol) {

	// Dont do anything if the edge is on the boundary
	if (this->is_border(edgeIdx))
		return null_edge();

	// Check if the flip would invalidate the mesh (i.e. flip orientation of the faces)
	Halfedge_mesh::Halfedge_index hf = this->halfedge(edgeIdx);
	Point_3 p1 = this->point(this->source(hf)), p2 = this->point(this->target(hf));
	Point_3 p3 = this->point(this->source(this->prev(hf))), p4 = this->point(this->target(this->next(this->opposite(hf))));
	Vector_3 normal1 = CGAL::unit_normal(p1, p2, p3);
	Vector_3 normal2 = CGAL::unit_normal(p4, p2, p3);
	if (CGAL::scalar_product(normal1, normal2) < 0)
		// The normals are in the opposite directions, the mesh would get invalidated, so do nothing
		return null_edge();

	// General mesh (any polygon)
	std::vector<Halfedge_index>halfEdges;
	std::vector<Halfedge_index>twins;
	std::vector<Edge_index>Edges;
	std::vector<Vertex_index>Vertices;

	Halfedge_index h_prev;					// Halfedge that has next as edgeIdx->halfedge()
	
	// Iterating over the first face
	Halfedge_index h = this->halfedge(edgeIdx);
	halfEdges.push_back(h);
	Edges.push_back(this->edge(h));
	h = this->next(h);
	do
	{
		halfEdges.push_back(h);
		Edges.push_back(this->edge(h));
		twins.push_back(this->opposite(h));
		Vertices.push_back(this->source(h));
		if (this->next(h) == this->halfedge(edgeIdx))
			h_prev = h;
		h = this->next(h);
	} while (h != this->halfedge(edgeIdx));

	// Iterating over the second face
	h = this->opposite(this->halfedge(edgeIdx));
	halfEdges.push_back(h);
	h = this->next(h);
	do
	{
		halfEdges.push_back(h);
		Edges.push_back(this->edge(h));
		Vertices.push_back(this->source(h));
		twins.push_back(this->opposite(h));
		h = this->next(h);
	} while (h != this->opposite(this->halfedge(edgeIdx)));


	// Collecting the current mesh elements
	// Half-edges
	Halfedge_index h0 = this->halfedge(edgeIdx);
	Halfedge_index h5 = this->opposite(h0);

	// Vertices
	// Current end points of the edge
	Vertex_index v4 = this->source(this->next(h0));
	Vertex_index v0 = this->source(this->next(h5));
	// New end points of the edge
	Vertex_index v5 = this->source(this->next(this->next(h0)));
	Vertex_index v1 = this->source(this->next(this->next(h5)));

	// Faces
	Face_index f0 = this->face(h0);
	Face_index f1 = this->face(h5);

	// TODO
	// Checking if the flip would cause 'sliver' triangles (almost coincident edges)
	if ((this->degree(f0) == 3) && (this->degree(f1) == 3)) {
		const Point_3 A = this->point(this->source(this->halfedge(edgeIdx))), B = this->point(this->source(this->next(this->next(this->opposite(this->halfedge(edgeIdx)))))),
			C = this->point(this->source(h_prev)), D = this->point(this->source(this->next(this->next(this->next(this->opposite(this->halfedge(edgeIdx)))))));
		Vector_3 BC = C - B;																		// New position of edge
		Vector_3 originalArea1 = CGAL::cross_product(C - D, A - D);
		Vector_3 originalArea2 = CGAL::cross_product(A - D, B - D);
		double area = originalArea1.squared_length() < originalArea2.squared_length() ? originalArea1.squared_length() : originalArea2.squared_length();
		//std::cout << "AREA: " << area << "\n";
		for (int i = 0; i < Vertices.size() - 1; i++) {
			Vector_3 side = this->point(Vertices[i + 1]) - this->point(Vertices[i]);				// Vector along side of polygon
			// Check if the flipped edge would be coincident with an existing edge
			double newArea_sq = CGAL::cross_product(side, BC).squared_length();
			/*if (newArea_sq < (0.1*area*0.1*area)) {*/
			if (newArea_sq < areaTol) {
				//std::cout << "EDGE NOT FLIPPED DUE TO SLIVER\n\n";
				return null_edge();
			}
		}
	}



	// Reassigning the mesh elements
	// Half-edges
	this->set_target(h0, v5);
	this->set_target(h5, v1);


	// Assigning the next of h0
	h = this->halfedge(edgeIdx);
	Halfedge_index h0_prev, h0_next, h5_prev, h5_next;	// half-edges adjacent to h0 and h5
	Halfedge_index h0_next_next = this->next(this->next(h0));
	Halfedge_index h5_next_next = this->next(this->next(h5));
	h0_next = this->next(h0); h5_next = this->next(h5);
	do {
		if (this->next(h) == h0)
			h0_prev = h;
		h = this->next(h);
	} while (h != h0);
	h = h5;
	do {
		if (this->next(h) == h5)
			h5_prev = h;
		h = this->next(h);
	} while (h != h5);

	// Assigning the next of h5, h0 and halfedges adjacent to it in the old configuration
	this->set_next(h5_prev, h0_next);
	this->set_next(h0_next, h5);
	this->set_next(h5_next, h0);
	this->set_next(h0_prev, h5_next);

	this->set_next(h5, h5_next_next);
	this->set_next(h0, h0_next_next);

	// Face f0
	this->set_halfedge(f0, h0);
	Halfedge_index h0_temp = h0;
	do {
		this->set_face(h0_temp, f0);
		h0_temp = this->next(h0_temp);
	} while (h0_temp != h0);


	// Face f1
	this->set_halfedge(f1, h5);
	Halfedge_index h5_temp = h5;
	do {
		this->set_face(h5_temp, f1);
		h5_temp = this->next(h5_temp);
	} while (h5_temp != h5);

	// Update the normals for both the faces
	this->faceNormals[f0] = this->GetFaceNormal(f0);
	this->faceNormals[f1] = this->GetFaceNormal(f1);

	return edgeIdx;

	return null_edge();
}

// TODO:
// Collapse an edge into a vertex
Halfedge_mesh::Vertex_index Halfedge_mesh::CollapseEdge(Edge_index edgeIdx) {
	// Collecting the current mesh elements
	Halfedge_index h0 = halfedge(edgeIdx);
	Halfedge_index h1 = opposite(halfedge(edgeIdx));

	Vertex_index v0 = source(h0);
	Vertex_index v1 = source(h1);


	Edge_index E0 = edgeIdx;						// Edge to be collapsed

	// These edges & faces(in case where the faces connected to the edge are triangular)
	// will be deleted after all the new elements have been made
	// and all the connectivity has been established
	std::vector<Edge_index> edgesToDelete;
	std::vector<Face_index> facesToDelete;
	edgesToDelete.push_back(edgeIdx);

	Face_index F0 = face(h0);
	Face_index F1 = face(h1);

	// Half-edges associated with the half-edges of the given edge
	Halfedge_index h0_prev, h0_next, h1_prev, h1_next;
	std::vector<Vertex_index>neighbour_v0;					// Neighbouring vertices of v0
	std::vector<Vertex_index>neighbour_v1;					// Neighbouring vertices of v1

	// Halfedges eminating from the vertex v0 and neighbour vertices of v0
	std::vector<Halfedge_index>halfEdges0;
	Halfedge_index h = next(h1);
	h1_next = h;
	int count = 0;
	do
	{
		halfEdges0.push_back(h);
		neighbour_v0.push_back(target(h));
		h = next(opposite(h));
		if (next(opposite(h)) == h0)
			h0_prev = opposite(h);
		count++;
	} while (h != h0);
// TEMP
	int vert0_degree = degree(source(h0));

	// Halfedges eminating from the vertex v1 and neighbour vertices of v1
	std::vector<Halfedge_index>halfEdges1;
	h = next(h0);
	h0_next = h;
	count = 0;
	do
	{
		halfEdges1.push_back(h);
		neighbour_v1.push_back(target(h));
		h = next(opposite(h));
		if (next(opposite(h)) == h1)
			h1_prev = opposite(h);
		count++;
	} while (h != h1);
// TEMP
	int vert1_degree = degree(source(h1));

	// Edges associated with the half-edges adjacent to h0 and h1
	Edge_index e_h0_prev = edge(h0_prev);
	Edge_index e_h0_next = edge(h0_next);
	Edge_index e_h1_prev = edge(h1_prev);
	Edge_index e_h1_next = edge(h1_next);

	// Checking for proper connectivity of the mesh (Vertices of the edge have only 2 joint neighbours)
	//int min_size = neighbour_v0.size() < neighbour_v1.size() ? neighbour_v0.size() : neighbour_v1.size();
	int jointNeighbours = 0;
	for (int i = 0; i < neighbour_v0.size(); i++) {
		for (int j = 0; j < neighbour_v1.size(); j++) {
			if (neighbour_v0[i] == neighbour_v1[j])
				jointNeighbours++;
		}
	}
	//std::cout << "JOINT NEIGHBOURS: " << jointNeighbours << "\n";
	if (jointNeighbours > 2)
		return v0;													// Returning the old vertex as edge is not to be collapsed

	// Check if new faces are flipped (normal points in opposite directions)
	Point_3 newPosition = (point(v0) + point(v1)) / 2.0;
	// Faces containing v0
	for (int i = 0; i < neighbour_v0.size() - 1; i++) {
		// Area vector of the old face
		Vector_3 AB = point(neighbour_v0[i]) - point(v0);
		Vector_3 AC = point(neighbour_v0[i + 1]) - point(v0);
		Vector_3 oldArea = CGAL::cross_product(AB, AC);

		// Area vector of the new face
		AB = point(neighbour_v0[i]) - newPosition;
		AC = point(neighbour_v0[i + 1]) - newPosition;
		Vector_3 newArea = CGAL::cross_product(AB, AC);

		if (CGAL::scalar_product(oldArea, newArea) < 0) {
			//std::cout << "FACE FLIPPED\n";
			return v0;
		}
	}
	// Faces containing v1
	for (int i = 0; i < neighbour_v1.size() - 1; i++) {
		// Area vector of the old face
		Vector_3 AB = point(neighbour_v1[i]) - point(v1);
		Vector_3 AC = point(neighbour_v1[i + 1]) - point(v1);
		Vector_3 oldArea = CGAL::cross_product(AB, AC);

		// Area vector of the new face
		AB = point(neighbour_v1[i]) - newPosition;
		AC = point(neighbour_v1[i + 1]) - newPosition;
		Vector_3 newArea = CGAL::cross_product(AB, AC);

		if (CGAL::scalar_product(oldArea, newArea) < 0) {
			//std::cout << "FACE FLIPPED\n";
			return v0;
		}
	}

	// Making the new elements
	// The new vertex after collapsing the edge
	Vertex_index v2 = add_vertex(newPosition);

	// Reassigning the mesh elements
	// Half-edges surrounding the half-edges of the edge to collapse
	// Checking if the adjacent faces are triangular
	bool h0_degree3 = degree(face(h0)) == 3;
	bool h1_degree3 = degree(face(h1)) == 3;

	// Faces
	//// For non-triangular faces
	//if (!h0_degree3)
	//{
	//	//halfedge(f0) = h0_prev;
	//	//Halfedge_index h_face_0 = next(h0);
	//	//do {
	//	//	//h_face_0->face() = f0;
	//	//	set_face(h_face_0, f0);
	//	//	h_face_0 = next(h_face_0);
	//	//} while (h_face_0 != h0);

	//	// Reassigning the halfedges
	//	Halfedge_index prevOfCurr = prev(h0),
	//		nextOfCurr = next(h0);					// Previous and next halfedges of the halfedge of this edge that is in the non-tiangular face
	//	set_next(prevOfCurr, nextOfCurr);

	//	// Assigning half-edge to the new vertex
	//	set_halfedge(v2, opposite(next(opposite(next(h0)))));
	//	//v2->halfedge() = h0->next()->twin()->next();
	//}

	//if (!h1_degree3)
	//{
	//	halfedge(f1) = h1_prev;
	//	Halfedge_index h_face_1 = next(h1);
	//	do {
	//		set_face(h_face_1, f1);
	//		h_face_1 = next(h_face_1);
	//	} while (h_face_1 != h1);
	//	// Assigning half-edge to the new vertex
	//	set_halfedge(v2, opposite(next(opposite(next(h1)))));
	//	//v2->halfedge() = h0->next()->twin()->next();
	//}

	if (h0_degree3){
		// If the face is triangular, then the 2 edges connected to the edge to be collapsed,
		// would get combined into a single edge, say, E0.
		// So, the halfedges belonging to those 2 edges that are not in the face that would be collapsed,
		// would now become opposites of each other in the new configuration
		// Since, CGAL doesn't allow for setting opposites explicitly,
		// Make the new edge E0 (the combined edge), and set the next and prev halfedges appropriately
		
		Halfedge_index temp_prev = opposite(h0_prev), temp_next = opposite(h0_next);
		Halfedge_index prev_of_temp_prev = prev(temp_prev), next_of_temp_prev = next(temp_prev);
		Halfedge_index prev_of_temp_next = prev(temp_next), next_of_temp_next = next(temp_next);

		//e_h0_new->halfedge() = temp_prev;
		Vertex_index oppVert = source(next_of_temp_prev);
		Halfedge_index hf_h0_new = add_edge(v2, oppVert);						// New edge that will replace the triangular face
		Edge_index e_h0_new = edge(hf_h0_new);
		// Note that the set_next fn sets the prev halfedge also correctly
		set_next(hf_h0_new, next_of_temp_prev);
		set_next(prev_of_temp_prev, hf_h0_new);

		set_next(opposite(hf_h0_new), next_of_temp_next);
		set_next(prev_of_temp_next, opposite(hf_h0_new));

		// Set the vertex
		set_halfedge(oppVert, hf_h0_new);
		set_halfedge(v2, opposite(hf_h0_new));
		set_target(hf_h0_new, oppVert);
		set_target(opposite(hf_h0_new), v2);

		// Set the face for the new halfedges
		set_face(hf_h0_new, face(prev_of_temp_prev));
		set_face(opposite(hf_h0_new), face(prev_of_temp_next));

		// Set the halfedges for the adjacent faces
		set_halfedge(face(next_of_temp_prev), hf_h0_new);
		set_halfedge(face(next_of_temp_next), opposite(hf_h0_new));

		// Delete the old edges
		edgesToDelete.push_back(edge(temp_prev));
		edgesToDelete.push_back(edge(temp_next));

		// Delete the face
		facesToDelete.push_back(face(h0));
	}
	else
		set_next(h0_prev, h0_next);


	if (h1_degree3){
		// If the face is triangular, then the 2 edges connected to the edge to be collapsed,
		// would get combined into a single edge, say, E0.
		// So, the halfedges belonging to those 2 edges that are not in the face that would be collapsed,
		// would now become opposites of each other in the new configuration
		// Since, CGAL doesn't allow for setting opposites explicitly,
		// Make the new edge E0 (the combined edge), and set the next and prev halfedges appropriately

		Halfedge_index temp_prev = opposite(h1_prev), temp_next = opposite(h1_next);
		Halfedge_index prev_of_temp_prev = prev(temp_prev), next_of_temp_prev = next(temp_prev);
		Halfedge_index prev_of_temp_next = prev(temp_next), next_of_temp_next = next(temp_next);

		//e_h0_new->halfedge() = temp_prev;
		Vertex_index oppVert = source(next_of_temp_prev);

		Halfedge_index hf_h1_new = add_edge(v2, oppVert);									// New edge that will replace the triangular face
		Edge_index e_h1_new = edge(hf_h1_new);									// New edge that will replace the triangular face

		// Note that the set_next fn sets the prev halfedge also correctly
		set_next(hf_h1_new, next_of_temp_prev);
		set_next(prev_of_temp_prev, hf_h1_new);

		set_next(opposite(hf_h1_new), next_of_temp_next);
		set_next(prev_of_temp_next, opposite(hf_h1_new));

		// Set the vertex
		set_halfedge(oppVert, hf_h1_new);
		set_target(hf_h1_new, oppVert);
		set_target(opposite(hf_h1_new), v2);

		// Set the faces for the new halfedges
		set_face(hf_h1_new, face(prev_of_temp_prev));
		set_face(opposite(hf_h1_new), face(prev_of_temp_next));

		// Set the halfedges for the adjacent faces
		set_halfedge(face(next_of_temp_prev), hf_h1_new);
		set_halfedge(face(next_of_temp_next), opposite(hf_h1_new));

		// Delete the old
		edgesToDelete.push_back(edge(temp_prev));
		edgesToDelete.push_back(edge(temp_next));

		// Delete the face
		facesToDelete.push_back(face(h1));
	}
	else
		set_next(h1_prev, h1_next);

	// Assigning the new vertex to the appropriate half-edges
	for (int i = 0; i < halfEdges0.size(); i++)
	{
		if (!h1_degree3)
			set_target(opposite(halfEdges0[i]), v2);
		else if (halfEdges0[i] != h1_next)
			set_target(opposite(halfEdges0[i]), v2);
	}

	for (int i = 0; i < halfEdges1.size(); i++)
	{
		if (!h0_degree3)
			set_target(opposite(halfEdges1[i]), v2);
		else if (halfEdges1[i] != h0_next)
			set_target(opposite(halfEdges1[i]), v2);
	}


	// Deleting the old elements
	// Delete edges
	size_t numEdgesToDelete = edgesToDelete.size();
	size_t edgeCount = 0;
	auto edgeIter = edgesToDelete.begin();
	while(edgeCount < numEdgesToDelete){
		auto nextEdgeIter = edgeIter;
		nextEdgeIter++;
		remove_edge(*edgeIter);
		edgeIter = nextEdgeIter;
		edgeCount++;
	}
	// Delete the faces, if needed
	size_t numFacesToDelete = facesToDelete.size();
	if (numFacesToDelete > 0) {
		size_t faceCount = 0;
		auto faceIter = facesToDelete.begin();
		while (faceCount < numFacesToDelete) {
			auto nextFaceIter = faceIter;
			nextFaceIter++;
			faceNormals.erase(*faceIter);	// Remove the face from the normal map
			remove_face(*faceIter);
			faceIter = nextFaceIter;
			faceCount++;
		}
	}

	// Delete the vertices connected to the edge to be collapsed
	remove_vertex(v0);
	remove_vertex(v1);

	// Calculate the vertex normal
	Halfedge_index newVertHf = halfedge(v2);
	// Recompute the face & vertex normals
	Halfedge_index hf = halfedge(v2);
	do {
		Face_index fIdx = face(hf);
		if (faceNormals.find(fIdx) != faceNormals.end()) {
			// Update the face area
			faceNormals[fIdx] = GetFaceNormal(fIdx);
		}
		else {
			faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(fIdx, GetFaceNormal(fIdx)));
		}
		hf = opposite(next(hf));
	} while (hf != halfedge(v2));
	vertexNormals.insert(std::pair<Halfedge_mesh::Vertex_index, Vector_3>(v2, GetVertexNormal(v2)));
	return v2;


	return null_vertex();
}

// Get the normal of a face
// TODO: Check if CGAL can do this on its own
// TODO: Store the normals after computing once
Vector_3 Halfedge_mesh::GetFaceNormal(Halfedge_mesh::Face_index faceIdx) {
	Halfedge_mesh::Halfedge_index halfEdgeIdx = this->halfedge(faceIdx);
	const Point_3 pt1 = this->point(this->source(halfEdgeIdx));
	const Point_3 pt2 = this->point(this->source(this->next(halfEdgeIdx)));
	const Point_3 pt3 = this->point(this->source(this->prev(halfEdgeIdx)));

	return CGAL::unit_normal(pt1, pt2, pt3);
}

// Get the normal at a vertex, by taking the average of the faces containing the vertex
Vector_3 Halfedge_mesh::GetVertexNormal(Halfedge_mesh::Vertex_index vertIdx) {
	std::vector<Halfedge_mesh::Face_index> neighbourFaceIdxs;
	this->FacesAroundVertex(vertIdx, neighbourFaceIdxs);
	Vector_3 vertNormal(0.0, 0.0, 0.0);
	// Going over the faces that contain the vertex to get the average normal
	for (auto faceIdx : neighbourFaceIdxs) {
		vertNormal += this->faceNormals.at(faceIdx);
	}
	vertNormal = vertNormal / (double)neighbourFaceIdxs.size();
	// Normalize the normal
	vertNormal = vertNormal / sqrt(vertNormal.squared_length());
	return vertNormal;
}


// Get the face indices of all the faces containing a given vertex
// Clears the vector first, caller is responsible
void Halfedge_mesh::FacesAroundVertex(Halfedge_mesh::Vertex_index vertIdx, std::vector<Halfedge_mesh::Face_index> &neighbourFaceIdxs) const {
	neighbourFaceIdxs.clear();
	Halfedge_mesh::Halfedge_index startHalfEdgeIdx = this->halfedge(vertIdx);
	Halfedge_mesh::Halfedge_index currHalfEdgeIdx = startHalfEdgeIdx;
	do {
		Face_index face = this->face(currHalfEdgeIdx);
		if(face != null_face())
			neighbourFaceIdxs.push_back(face);
		currHalfEdgeIdx = this->prev(this->opposite(currHalfEdgeIdx));
	} while (currHalfEdgeIdx != startHalfEdgeIdx);
}

// Get all the vertices that are connected to the given vertex
void Halfedge_mesh::VerticesConnectedToVertex(Vertex_index vertIdx, std::vector<Vertex_index> &connectedVertIdxs) const {
	connectedVertIdxs.clear();
	Halfedge_mesh::Halfedge_index startHalfEdgeIdx = this->halfedge(vertIdx);
	Halfedge_mesh::Halfedge_index currHalfEdgeIdx = startHalfEdgeIdx;
	do {
		connectedVertIdxs.push_back(this->source(currHalfEdgeIdx));
		currHalfEdgeIdx = this->prev(this->opposite(currHalfEdgeIdx));
	} while (currHalfEdgeIdx != startHalfEdgeIdx);
}

// Get the centroid of a face
Point_3 Halfedge_mesh::GetFaceCentroid(Face_index faceIdx) {
	Halfedge_index hf = halfedge(faceIdx);
	Halfedge_index currhf = hf;
	Point_3 centroid(0.0, 0.0, 0.0);
	int numVerts = 0;
	do {
		centroid += point(source(currhf));
		++numVerts;
		currhf = next(currhf);
	} while (currhf != hf);

	return centroid / numVerts;
}

// Recompute the face normals
void Halfedge_mesh::RecomputeFaceNormals(){
	for(auto faceIter = this->faces_begin(); faceIter != this->faces_end(); ++faceIter)
		this->faceNormals[*faceIter] = this->GetFaceNormal(*faceIter);
}

// Recompute the vertex normals
void Halfedge_mesh::RecomputeVertexNormals() {
	for (auto vertIter = this->vertices_begin(); vertIter != this->vertices_end(); ++vertIter)
		this->vertexNormals[*vertIter] = this->GetVertexNormal(*vertIter);
}
