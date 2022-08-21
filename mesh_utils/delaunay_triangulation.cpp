// Implementation of Delaunay triangulation
// Using an incremental building algorithm to do this

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <fstream>
#include <math.h>
#include <cstdlib>     /* srand, rand */
#include <ctime>       /* time */
#include "meshRenderer.h"
#include "bvh.h"
#include "fssimplewindow.h"
#include "meshIO.h"
#include "delaunay_triangulation.h"

using namespace GeomUtils;
using namespace MeshUtils;

//// Generate a random number between 2 doubles
//float RandomDouble(const double a, const double b) {
//	double random = ((double)rand()) / (double)RAND_MAX;
//	double range = b - a;
//	double r = random * range;
//	return a + r;
//}
//
//// Generate a set of random vertices (2D)
//void GenerateRandomVerts(std::vector<Point_3> &verts, const int numVerts, const double xMin, const double xMax,
//							const double yMin, const double yMax) {
//	verts.clear();
//	srand(time(NULL));		// Seed for RNG
//	for (int i = 0; i < numVerts; ++i)
//		verts.push_back(Point_3(RandomDouble(xMin, xMax), RandomDouble(yMin, yMax), 0.0));
//}

// Compute barycentric coordinates (u, v, w) for
// point p with respect to triangle (a, b, c)
// Uses Cramer's rule
void Barycentric(Point_3 p, Point_3 a, Point_3 b, Point_3 c, double &u, double &v, double &w){
	Vector_3 v0 = b - a, v1 = c - a, v2 = p - a;
	double d00 = CGAL::scalar_product(v0, v0);
	double d01 = CGAL::scalar_product(v0, v1);
	double d11 = CGAL::scalar_product(v1, v1);
	double d20 = CGAL::scalar_product(v2, v0);
	double d21 = CGAL::scalar_product(v2, v1);
	double denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}

// Check if an edge in the triangulation is illegal (for delaunay)
// An edge is illegal if the circumcircle of either triangle containing the edge
// contains the opposite vertex (vertex that is not contained in that triangle)
// (If an edge is illegal, flip it to get a more delaunay configuration)
bool isEdgeIllegal(const MeshUtils::Halfedge_mesh &mesh2d, MeshUtils::Halfedge_mesh::Edge_index edgeIdx) {
	// Check if the edge is on the border
	// i.e. if an incident face to the edge is null_face()
	if (mesh2d.is_border(edgeIdx))
		return false;

	MeshUtils::Halfedge_mesh::halfedge_index hf = mesh2d.halfedge(edgeIdx);
	// Check the degrees of the incident faces
	if ((mesh2d.degree(mesh2d.face(hf)) != 3) || (mesh2d.degree(mesh2d.face(mesh2d.opposite(hf))) != 3))
		return false;
	
	// First triangle (ABC) containing given edge
	Point_3 A = mesh2d.point(mesh2d.source(hf));
	Point_3 B = mesh2d.point(mesh2d.source(mesh2d.next(hf)));
	Point_3 C = mesh2d.point(mesh2d.source(mesh2d.prev(hf)));
	// Point opposite to the edge and not in this triangle
	Point_3 D = mesh2d.point(mesh2d.target(mesh2d.next(mesh2d.opposite(hf))));



// OLD METHOD (DOES NOT WORK!!!) -->
	//// Check if the triangle is delaunay
	//double diff_ADx = A.x() - D.x(), diff_ADy = A.y() - D.y();
	//double diff_BDx = B.x() - D.x(), diff_BDy = B.y() - D.y();
	//double diff_CDx = C.x() - D.x(), diff_CDy = C.y() - D.y();

	//Eigen::Matrix3d mat;
	//mat(0, 0) = diff_ADx; mat(0, 1) = diff_ADy; mat(0, 2) = diff_ADx * diff_ADx - diff_ADy * diff_ADy;
	//mat(1, 0) = diff_BDx; mat(1, 1) = diff_BDy; mat(1, 2) = diff_BDx * diff_BDx - diff_BDy * diff_BDy;
	//mat(2, 0) = diff_CDx; mat(2, 1) = diff_CDy; mat(2, 2) = diff_CDx * diff_CDx - diff_CDy * diff_CDy;

	//if (mat.determinant() > 0)
	//	return true;

// NEW METHOD
	// Get the circumcircle of the triangle, and check if the other pt lies in it
	// Getting this uses the fact that the circumcenter is equidistant from the triangle verts
	// This simplifies to a system of 2 equations. Solving it by Cramer's rule, we get the center
	// NOTE: Assume that (a1, b1), (a2, b2) are the coefficients and (c1, c2) are the constants in the equations
	double xa = A.x(), xb = B.x(), xc = C.x();
	double ya = A.y(), yb = B.y(), yc = C.y();
	double diffx_ab = xa - xb, diffx_bc = xb - xc, diffx_ca = xc - xa;
	double diffy_ab = ya - yb, diffy_bc = yb - yc, diffy_ca = yc - ya;

	double a1 = 2 * xa * diffx_ab + 2 * xb * diffx_bc + 2 * xc * diffx_ca,
		b1 = 2 * ya * diffx_ab + 2 * yb * diffx_bc + 2 * yc * diffx_ca;
	double a2 = 2 * xa * diffy_ab + 2 * xb * diffy_bc + 2 * xc * diffy_ca,
		b2 = 2 * ya * diffy_ab + 2 * yb * diffy_bc + 2 * yc * diffy_ca;
	double c1 = diffx_ab * (xa * xa + ya * ya) + diffx_bc * (xb * xb + yb * yb) + diffx_ca * (xc * xc + yc * yc),
		c2 = diffy_ab * (xa * xa + ya * ya) + diffy_bc * (xb * xb + yb * yb) + diffy_ca * (xc * xc + yc * yc);

	// Let the circumcenter be (h, k)
	double Det = a1 * b2 - b1 * a2, Dh = c1 * b2 - b1 * c2, Dk = a1 * c2 - c1 * a2;
	double h = Dh / Det, k = Dk / Det;
	double rad_sq = (A.x() - h) * (A.x() - h) + (A.y() - k) * (A.y() - k);
	if ((D.x() - h) * (D.x() - h) + (D.y() - k) * (D.y() - k) <= rad_sq)
		// Pt D lies inside the circumcircle, so illegal
		return true;

	// Other triangle (ABC) containing given edge
	hf = mesh2d.opposite(hf);
	A = mesh2d.point(mesh2d.source(hf));
	B = mesh2d.point(mesh2d.source(mesh2d.next(hf)));
	C = mesh2d.point(mesh2d.source(mesh2d.prev(hf)));
	// Point opposite to the edge and not in this triangle
	D = mesh2d.point(mesh2d.target(mesh2d.next(mesh2d.opposite(hf))));

// OLD METHOD (DOES NOT WORK!!!) -->
	//// Check if the triangle is delaunay
	//diff_ADx = A.x() - D.x(), diff_ADy = A.y() - D.y();
	//diff_BDx = B.x() - D.x(), diff_BDy = B.y() - D.y();
	//diff_CDx = C.x() - D.x(), diff_CDy = C.y() - D.y();

	//mat(0, 0) = diff_ADx; mat(0, 1) = diff_ADy; mat(0, 2) = diff_ADx * diff_ADx - diff_ADy * diff_ADy;
	//mat(1, 0) = diff_BDx; mat(1, 1) = diff_BDy; mat(1, 2) = diff_BDx * diff_BDx - diff_BDy * diff_BDy;
	//mat(2, 0) = diff_CDx; mat(2, 1) = diff_CDy; mat(2, 2) = diff_CDx * diff_CDx - diff_CDy * diff_CDy;

	//if (mat.determinant() > 0)
	//	return true;

// NEW METHOD
	// Get the circumcircle of the triangle, and check if the other pt lies in it
	// Getting this uses the fact that the circumcenter is equidistant from the triangle verts
	// This simplifies to a system of 2 equations. Solving it by Cramer's rule, we get the center
	// NOTE: Assume that (a1, b1), (a2, b2) are the coefficients and (c1, c2) are the constants in the equations
	xa = A.x(); xb = B.x(); xc = C.x();
	ya = A.y(); yb = B.y(); yc = C.y();
	diffx_ab = xa - xb; diffx_bc = xb - xc; diffx_ca = xc - xa;
	diffy_ab = ya - yb; diffy_bc = yb - yc; diffy_ca = yc - ya;

	a1 = 2 * xa * diffx_ab + 2 * xb * diffx_bc + 2 * xc * diffx_ca;
	b1 = 2 * ya * diffx_ab + 2 * yb * diffx_bc + 2 * yc * diffx_ca;
	a2 = 2 * xa * diffy_ab + 2 * xb * diffy_bc + 2 * xc * diffy_ca;
	b2 = 2 * ya * diffy_ab + 2 * yb * diffy_bc + 2 * yc * diffy_ca;
	c1 = diffx_ab * (xa * xa + ya * ya) + diffx_bc * (xb * xb + yb * yb) + diffx_ca * (xc * xc + yc * yc);
	c2 = diffy_ab * (xa * xa + ya * ya) + diffy_bc * (xb * xb + yb * yb) + diffy_ca * (xc * xc + yc * yc);

	// Let the circumcenter be (h, k)
	Det = a1 * b2 - b1 * a2; Dh = c1 * b2 - b1 * c2; Dk = a1 * c2 - c1 * a2;
	h = Dh / Det; k = Dk / Det;
	rad_sq = (A.x() - h) * (A.x() - h) + (A.y() - k) * (A.y() - k);
	if ((D.x() - h) * (D.x() - h) + (D.y() - k) * (D.y() - k) <= rad_sq)
		// Pt D lies inside the circumcircle, so illegal
		return true;

	// Edge is legal
	return false;
}

// Check if the face has an edge(s) that are on the boundary
bool isFaceOnBoundary(const MeshUtils::Halfedge_mesh &mesh, MeshUtils::Halfedge_mesh::Face_index faceIdx) {
	// Halfedge in the face
	MeshUtils::Halfedge_mesh::Halfedge_index hf = mesh.halfedge(faceIdx);
	MeshUtils::Halfedge_mesh::Halfedge_index currhf = hf;
	do {
		if (mesh.is_border(mesh.edge(currhf)))
			return true;
		currhf = mesh.next(currhf);
	} while (currhf != hf);
	// The face is not on the boundary
	return false;
}


// Create a dummy convex triangle based on the point set so that the triangulation is easier to do
// These points, and their incident edges would be deleted at the end of the triangulation
void makeDummyConvexTriangle(MeshUtils::Halfedge_mesh &mesh2d, const std::vector<Point_3> &verts, std::vector<MeshUtils::Halfedge_mesh::Vertex_index> &dummyVerts) {
	// TODO
	// Get the BBox for the set of points
	CGAL::Iso_cuboid_3<CGAL_double> bbox = CGAL::bounding_box(verts.begin(), verts.end());

	// Construct 2 triangles using this rectangle (Considering triangulation of 2D point set for now)
	double xMin = bbox.xmin(), xMax = bbox.xmax();
	double yMin = bbox.ymin(), yMax = bbox.ymax();
	double z = bbox.zmax();

	// Size of the bbox
	double xSize = fabs(xMax - xMin), ySize = fabs(yMax - yMin);;
	double scaleFactor = 0.001;

	// Consider a quad ABCD (numbered from the lower left corner, clockwise)
	// This quad is slightly bigger than the og Bbox
	// Make triangle ABD
	Point_3 ptA(xMin - scaleFactor * xSize, yMin - scaleFactor * ySize, z), ptB(xMin - scaleFactor * xSize, yMax + scaleFactor * ySize, z),
		ptC(xMax + scaleFactor * xSize, yMax + scaleFactor * ySize, z), ptD(xMax + scaleFactor * xSize, yMin - scaleFactor * ySize, z);
	// Make new vertices for the dummy convex traingles
	MeshUtils::Halfedge_mesh::Vertex_index vertA = mesh2d.add_vertex(ptA);
	MeshUtils::Halfedge_mesh::Vertex_index vertB = mesh2d.add_vertex(ptB);
	MeshUtils::Halfedge_mesh::Vertex_index vertC = mesh2d.add_vertex(ptC);
	MeshUtils::Halfedge_mesh::Vertex_index vertD = mesh2d.add_vertex(ptD);
	// This record would have to be kept in order to remove them at the end of triangulation
	dummyVerts.push_back(vertA);
	dummyVerts.push_back(vertB);
	dummyVerts.push_back(vertC);
	dummyVerts.push_back(vertD);

	// Add the 2 triangles to the mesh
	MeshUtils::Halfedge_mesh::Face_index newFace1 = mesh2d.add_face(vertA, vertB, vertD);
	MeshUtils::Halfedge_mesh::Face_index newFace2 = mesh2d.add_face(vertB, vertC, vertD);
}

// The new point to be added to the triangulation would be contained inside 'closestFace'
// Split that face into 3. Then check if original edges of the triangle need to be flipped
// to maintain Delaunay conditions. If a flip occurs, then also check the edges that were
// connected to the original face, but not incident to the new pt
void MakeNewTriangle(MeshUtils::Halfedge_mesh &mesh2d, Point_3 newPt, MeshUtils::Halfedge_mesh::Face_index closestFace) {
	MeshUtils::Halfedge_mesh::Halfedge_index hf = mesh2d.halfedge(closestFace);

	// Get the edges of the triangle that contains this new point
	std::queue<MeshUtils::Halfedge_mesh::Edge_index> edgesToCheck;
	edgesToCheck.push(mesh2d.edge(hf));
	edgesToCheck.push(mesh2d.edge(mesh2d.next(hf)));
	edgesToCheck.push(mesh2d.edge(mesh2d.prev(hf)));

	//std::vector<Halfedge_mesh::Edge_index> edges{ mesh2d.edge(hf), mesh2d.edge(mesh2d.next(hf)), mesh2d.edge(mesh2d.prev(hf)) };

	// Check if the new pt lies on an edge of the closest face, in that case, the edge would
	// just be split making 4 total triangles instead of 3
	MeshUtils::Halfedge_mesh::Edge_index edgeToSplit = MeshUtils::Halfedge_mesh::null_edge();
	MeshUtils::Halfedge_mesh::Halfedge_index currHf = hf;
	do {
		MeshUtils::Halfedge_mesh::Edge_index edge = mesh2d.edge(currHf);
		// Ends of the edge
		Point_3 startPt = mesh2d.point(mesh2d.source(currHf)),
			endPt = mesh2d.point(mesh2d.target(currHf));
		Vector_3 edgeDir = (endPt - startPt),
			newPtDir = (newPt - startPt);
		Vector_3 crossProd = CGAL::cross_product(edgeDir, newPtDir);
		if (crossProd.squared_length() < zero_tol * zero_tol) {
			// The new pt lies on this edge, record it for splitting later
			edgeToSplit = edge;
			// If the edge is split, then even the other face containing the edge
			// would have to be adjust as those edges may be affected
			edgesToCheck.push(mesh2d.edge(mesh2d.next(mesh2d.opposite(hf))));
			edgesToCheck.push(mesh2d.edge(mesh2d.prev(mesh2d.opposite(hf))));

			break;
		}
		// Move onto the next edge
		currHf = mesh2d.next(currHf);
	} while (currHf != hf);

	// Make the new triangles
	MeshUtils::Halfedge_mesh::Vertex_index newVert;
	if (edgeToSplit != MeshUtils::Halfedge_mesh::null_edge())
		// New pt is on an existing edge, so split it
		newVert = mesh2d.SplitEdge(edgeToSplit, &newPt);
	else
		// The new point is inside the triangle
		// So, split the triangle into 3
		newVert = mesh2d.SubdivideFace(closestFace, &(newPt));

	// Check if edges need to be flipped
	// Initially, the only edges that need to checked are the edges of triangle ABC (the triangle that contains the new pt)
	// If any of these edges are flipped, then the other 2 edges of the quadrilateral in which the flip has taken place
	// (these edges are not incident to the new pt), also need to be checked.
	// For eg, if the diagonal QS of quad PQRS is to be flipped (P being the new pt), then after the flip, edges QR and RS
	// would have to be checked for Delaunay configuration
	std::vector<MeshUtils::Halfedge_mesh::Edge_index> newEdgesToCheck;
	while (!edgesToCheck.empty()) {
		MeshUtils::Halfedge_mesh::Edge_index edIdx = edgesToCheck.front();
		// Check if the edge violates Delaunay condition (provided it is not a boundary edge)
		if (!mesh2d.is_border(edIdx) && isEdgeIllegal(mesh2d, edIdx)) {
			// If a flip is not performed for whatever reason, a null_edge is returned
			if (mesh2d.FlipEdge(edIdx) != Halfedge_mesh::null_edge()) {
				// Edge is to be flipped so, more edges to check as explained before
				// Check in the first face containing this edge
				MeshUtils::Halfedge_mesh::Halfedge_index startHF = mesh2d.halfedge(edIdx);
				MeshUtils::Halfedge_mesh::Halfedge_index hf = mesh2d.next(startHF);
				while (hf != startHF) {
					// Make sure that the new edge that would have to be checked is not incident to the new vertex
					if ((mesh2d.source(hf) != newVert) && (mesh2d.target(hf) != newVert))
						newEdgesToCheck.push_back(mesh2d.edge(hf));
					hf = mesh2d.next(hf);
				}
				// Check in the second face containing this edge
				startHF = mesh2d.opposite(mesh2d.halfedge(edIdx));
				hf = mesh2d.next(startHF);
				while (hf != startHF) {
					// Make sure that the new edge that would have to be checked is not incident to the new vertex
					if ((mesh2d.source(hf) != newVert) && (mesh2d.target(hf) != newVert))
						newEdgesToCheck.push_back(mesh2d.edge(hf));
					hf = mesh2d.next(hf);
				}
			}
		}

		// Remove the edge from the queue
		edgesToCheck.pop();
	}

	// Also check the new edges to be flipped
	for(auto edIdx : newEdgesToCheck)
		if (!mesh2d.is_border(edIdx) && isEdgeIllegal(mesh2d, edIdx))
			mesh2d.FlipEdge(edIdx);

}

// Remove the dummy vertices that were added to enclose the point set. Also remove the incident edges to those points
void removeDummyPoints(MeshUtils::Halfedge_mesh &mesh2d, std::vector<MeshUtils::Halfedge_mesh::Vertex_index> &dummyVerts) {
	std::set<MeshUtils::Halfedge_mesh::Edge_index> edgesToRemove;
	std::set<MeshUtils::Halfedge_mesh::Face_index> facesToRemove;
	for (size_t idx = 0; idx < dummyVerts.size(); idx++) {
		MeshUtils::Halfedge_mesh::Vertex_index vert = dummyVerts[idx];
		// TODO: Check if faces are automatically removed
		// Remove the incident edges for the dummy vertices
		MeshUtils::Halfedge_mesh::Halfedge_index startHF = mesh2d.opposite(mesh2d.halfedge(vert));
		// Traverse over all the incident edges and collect them
		MeshUtils::Halfedge_mesh::Halfedge_index hf = startHF;
		do{
			edgesToRemove.insert(mesh2d.edge(hf));
			MeshUtils::Halfedge_mesh::Face_index face = mesh2d.face(hf);
			if(face != MeshUtils::Halfedge_mesh::null_face())
				facesToRemove.insert(face);
			hf = mesh2d.opposite(mesh2d.prev(hf));
		} while (hf != startHF);
	}

	////// TODO: Check if remove_edge() removes isolated vertices too
	//// Remove the Edges
	//for (auto setIter = edgesToRemove.begin(); setIter != edgesToRemove.end(); ++setIter)
	//	mesh2d.remove_edge(*setIter);

	//// Remove the vertices
	//for (size_t idx = 0; idx < dummyVerts.size(); idx++)
	//	mesh2d.remove_vertex(dummyVerts[idx]);
	//	//CGAL::Euler::remove_center_vertex(mesh2d.halfedge(dummyVerts[idx]), static_cast<CGAL::Surface_mesh<Point_3>>(mesh2d));

	// TODO: CHECK face removal issue
	// Remove the faces
	std::cout << "FACES BEFORE: " << mesh2d.number_of_faces()<<"\n";
	for (auto setIter = facesToRemove.begin(); setIter != facesToRemove.end(); ++setIter)
		//mesh2d.remove_face(*setIter);
		CGAL::Euler::remove_face(mesh2d.halfedge(*setIter), static_cast<CGAL::Surface_mesh<Point_3>>(mesh2d));

	mesh2d.collect_garbage();
	std::cout << "FACES AFTER: " << mesh2d.number_of_faces();
}


// TEMP function to add a point to the triangulation - FOR DEBUGGING ONLY
void addPoint(MeshUtils::Halfedge_mesh &mesh2d, Point_3 newPt) {
	int numFaces = mesh2d.number_of_faces();

	faceBVH bvh(mesh2d);

	// Some triangles already exit, so add to the triangulation
	// STEP 1) Find the closest triangle to the new point
	MeshUtils::Halfedge_mesh::Face_index closestFace = bvh.findClosestFace(newPt);

	// Make a new triangle using the new vertex and an existing edge in the triangulation
	MakeNewTriangle(mesh2d, newPt, closestFace);

	//// Rebuild the BVH
	//bvh.rebuild(mesh2d);
}


// TODO: If the new pt added lies on an edge, then that edge would have to be split
// and would ultimately result in 4 triangles
// Perform delaunay triangulation (Incremental construction)
// The algorithm involves adding new points into the triangulation serially. The face that contains
// the new pt is split. A series of edge flips is performed locally (as the rest of the mesh is already Delaunay)
// in order to make the mesh Delaunay.
bool delaunay::triangulate_incremental(MeshUtils::Halfedge_mesh &mesh2d, const std::vector<Point_3> &points) {
	if (points.size() < 3)
		return false;

	// STEP I)
	// Create a dummy outer rectangle (2 triangles to be specific). All the points
	// in the set will be contained within either of these triangles. This proves to
	// be a good starting point for inserting the points into the triangulation.
	// At the end the dummy vertices will be removed along with their incident edges
	std::vector<MeshUtils::Halfedge_mesh::Vertex_index> dummyVerts;
	makeDummyConvexTriangle(mesh2d, points, dummyVerts);

	// Make a BVH for the mesh for fast closest face computations
	faceBVH bvh(mesh2d);

	// STEP II)
	// Add the points into the triangulation serially, by finding which existing triangle
	// contains each and then performing a series of local flips as needed to make the mesh Delaunay
	for (auto pointIter = points.begin(); pointIter != points.end(); ++pointIter) {
		// Find the closest triangle to the new point
		MeshUtils::Halfedge_mesh::Face_index closestFace = bvh.findClosestFace(*pointIter);

		// Add the new pt into the triangulation, by splitting an existing face
		MakeNewTriangle(mesh2d, *pointIter, closestFace);

		// TODO: Rebuilding the BVH after addition of each new pt seems extra; see if there is more efficient way
		// Rebuild the BVH
		bvh.rebuild(mesh2d);
	}

	// STEP III)
	// Remove the dummy vertices from he triangulation along with incident edges
	removeDummyPoints(mesh2d, dummyVerts);

	return true;
}
