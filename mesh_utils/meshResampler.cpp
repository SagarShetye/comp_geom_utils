#include "meshResampler.h"

using namespace GeomUtils;
using namespace MeshUtils;

namespace MeshUtils {
	// Perform loop subdivision on the input mesh
	// Only for triangular meshes
	void meshResampler::LoopSubdivision(Halfedge_mesh& mesh) {
		// TODO: Triangulate the mesh (if needed) before doing anything

		// To keep track of the new positions (according to the subdivision rule)
		std::unordered_map<Halfedge_mesh::Vertex_index, Point_3> newPosVerts;		// New positions for the vertices of the subdivided mesh
		std::unordered_set<Halfedge_mesh::Vertex_index> oldVertices;				// Vertices in the old mesh

		// STEP 1:
		// Calculate the vertex positions for the old vertices in the mesh
		for (auto vertIter = mesh.vertices_begin(); vertIter != mesh.vertices_end(); ++vertIter) {
			// Get all the connected vertices
			std::vector<Halfedge_mesh::Vertex_index> vertsAroundCurr;
			mesh.VerticesConnectedToVertex(*vertIter, vertsAroundCurr);
			Point_3 sumNeighbourPos(0.0, 0.0, 0.0);
			// Summation of the neighbour vertex positions
			for (auto vIdx : vertsAroundCurr)
				sumNeighbourPos += mesh.point(vIdx);
			//const size_t vertDegree = vertsAroundCurr.size();
			const size_t vertDegree = mesh.degree(*vertIter);
			double weight = 3.0 / (8.0 * vertDegree);
			if (vertDegree == 3)
				weight = 3.0 / 16.0;
			// Getting the new vertex position
			// newPos = (1 - n * weight) * oldPos + weight * (sum_of_neighbour_pos)
			// where 'n' is the vertex degree
			const Point_3 oldVertPos = mesh.point(*vertIter);
			Point_3 newVertPos = (1.0 - vertDegree * weight) * oldVertPos + weight * (sumNeighbourPos);
			newPosVerts.insert(std::pair<Halfedge_mesh::Vertex_index, Point_3>(*vertIter, newVertPos));
			oldVertices.insert(*vertIter);
		}


		// STEP 2:
		// Get the edges in the old mesh, only split these
		std::unordered_set<Halfedge_mesh::Edge_index> oldEdges;					// Edges in the old mesh
		std::unordered_set<Halfedge_mesh::Edge_index> newEdges;					// Edges that, after splitting, are not along an old edge
		std::unordered_map<Halfedge_mesh::Edge_index, Point_3> splitEdgePos;	// New positions for the split vertices

		// Calculate the positions to place the new vertex after splitting an edge of the original mesh
		for (auto edgeIter = mesh.edges_begin(); edgeIter != mesh.edges_end(); ++edgeIter) {
			// Calculate the position for the new vertex after splitting the edge
			Halfedge_mesh::Halfedge_index hfIdx = mesh.halfedge(*edgeIter);
			// Vertices opposite to the edge to be split
			Halfedge_mesh::Vertex_index oppVert_1 = mesh.source(mesh.prev(hfIdx));
			Halfedge_mesh::Vertex_index oppVert_2 = mesh.source(mesh.prev(mesh.opposite(hfIdx)));

			// Positions of vertices opposite to the edge
			const Point_3 oppPos_1 = mesh.point(oppVert_1);
			const Point_3 oppPos_2 = mesh.point(oppVert_2);

			// New position for the split vertex is given by
			// newPos = (3/8) * (sum of pos connected to edge) + (1/8) * (sum of pos opposite to the edge)
			Point_3 newPos = (3.0 / 8.0) * (mesh.point(mesh.source(hfIdx)) + mesh.point(mesh.target(hfIdx)))
				+ (1.0 / 8.0) * (oppPos_1 + oppPos_2);

			// Keep track of the new position to be updated for this vertex at the last step of the subdivision
			splitEdgePos.insert(std::pair<Halfedge_mesh::Edge_index, Point_3>(*edgeIter, newPos));

			oldEdges.insert(*edgeIter);
		}


		// Split all the edges in the original mesh
		const size_t numEdges = mesh.number_of_edges();
		auto edgeIter = mesh.edges_begin();
		//for (auto edgeIter = oldEdges.begin(); edgeIter != oldEdges.end(); ++edgeIter) {
		for (size_t edgeNum = 0; edgeNum < numEdges;) {
			auto nextEdgeIter = edgeIter;
			nextEdgeIter++;
			if (oldEdges.find(*edgeIter) != oldEdges.end()) {
				// This is an edge of the original mesh, can safely split this
				// Calculate the position for the new vertex after splitting the edge
				Halfedge_mesh::Halfedge_index hfIdx = mesh.halfedge(*edgeIter);
				// Vertices opposite to the edge to be split
				Halfedge_mesh::Vertex_index oppVert_1 = mesh.source(mesh.prev(hfIdx));
				Halfedge_mesh::Vertex_index oppVert_2 = mesh.source(mesh.prev(mesh.opposite(hfIdx)));

				// Positions of vertices opposite to the edge
				const Point_3 oppPos_1 = mesh.point(oppVert_1);
				const Point_3 oppPos_2 = mesh.point(oppVert_2);

				//// New position for the split vertex is given by
				//// newPos = (3/8) * (sum of pos connected to edge) + (1/8) * (sum of pos opposite to the edge)
				//Point_3 newVertPos = (3.0 / 8.0) * (mesh.point(mesh.source(hfIdx)) + mesh.point(mesh.target(hfIdx)))
				//	+ (1.0 / 8.0) * (oppPos_1 + oppPos_2);

				// New position for the split vertex
				Point_3 newVertPos = splitEdgePos.at(*edgeIter);

				// Split the edge
				Halfedge_mesh::Vertex_index newVertIdx = mesh.SplitEdge(*edgeIter);
				// Keep track of the desired position for the new split vertex (update at the last stage of the subdivision)
				newPosVerts.insert(std::pair<Halfedge_mesh::Vertex_index, Point_3>(newVertIdx, newVertPos));

				// Add all the edges that are not along the old edges
				// The new edges that are along edges of the original mesh would not be flipped!
				Halfedge_mesh::Halfedge_index newHfIdx = mesh.halfedge(newVertIdx);
				do {
					if ((mesh.source(newHfIdx) == oppVert_1) ||
						(mesh.source(newHfIdx) == oppVert_2))
						newEdges.insert(mesh.edge(newHfIdx));
					newHfIdx = mesh.prev(mesh.opposite(newHfIdx));
				} while (newHfIdx != mesh.halfedge(newVertIdx));
				edgeNum++;
			}
			edgeIter = nextEdgeIter;
		}


		// STEP 3:
		// Flip all the edges that are connected to one old vertex and one new vertex
		// The edge to flip should not be a part of an edge of the original mesh
		for (auto edgeIter = mesh.edges_begin(); edgeIter != mesh.edges_end(); ++edgeIter) {
			Halfedge_mesh::Vertex_index startVertIdx = mesh.source(mesh.halfedge(*edgeIter));
			Halfedge_mesh::Vertex_index endVertIdx = mesh.target(mesh.halfedge(*edgeIter));

			// Check if the edge formed a part of an edge of the old mesh
			bool alongNewEdge = (newEdges.find(*edgeIter) != newEdges.end());

			// Flags to check if the vertices of the edge are old or new (i.e. after split)
			bool startIsOld = false, endIsOld = false;
			if (oldVertices.find(startVertIdx) != oldVertices.end())
				startIsOld = true;
			if (oldVertices.find(endVertIdx) != oldVertices.end())
				endIsOld = true;

			// Flip the edge if it is connected to an old and a new vertex
			if (alongNewEdge && (startIsOld != endIsOld))
				mesh.FlipEdge(*edgeIter);
		}


		// STEP 4:
		// Update the positions of all the old vertices & the new ones
		for (auto mapIter = newPosVerts.begin(); mapIter != newPosVerts.end(); ++mapIter) {
			Halfedge_mesh::Vertex_index vertIdx = mapIter->first;
			mesh.point(vertIdx) = mapIter->second;
		}
	}


	// Perform isotropic remeshing of a triangular mesh
	void meshResampler::IsotropicRemeshing(Halfedge_mesh& mesh) {
		std::cout << "----------------------PRE-REMESHING---------------------------\n";
		std::cout << "NUM_EDGES: " << mesh.number_of_edges() << "\n";
		std::cout << "NUM_VERTS: " << mesh.number_of_vertices() << "\n";
		std::cout << "NUM_FACES: " << mesh.number_of_faces() << "\n\n\n";

		std::unordered_set<Halfedge_mesh::Edge_index> oldEdges;
		// Get the average edge length
		double avgEdgeLen = 0.0;
		size_t numEdges = 0;
		for (auto edgeIter = mesh.edges_begin(); edgeIter != mesh.edges_end(); ++edgeIter) {
			Halfedge_mesh::Halfedge_index hf = mesh.halfedge(*edgeIter);
			oldEdges.insert(*edgeIter);
			Point_3 edgeStartPos = mesh.point(mesh.source(hf));
			Point_3 edgeEndPos = mesh.point(mesh.target(hf));
			avgEdgeLen += sqrt((edgeEndPos - edgeStartPos).squared_length());
			++numEdges;
		}
		avgEdgeLen /= (double)numEdges;

		// Split all edges that have a length greater than 4L/3 (L - avg edge length)
		auto edgeIter = oldEdges.begin();
		for (; edgeIter != oldEdges.end();) {
			auto nextEdgeIter = edgeIter;
			++nextEdgeIter;
			Halfedge_mesh::Halfedge_index hf = mesh.halfedge(*edgeIter);
			Point_3 edgeStartPos = mesh.point(mesh.source(hf));
			Point_3 edgeEndPos = mesh.point(mesh.target(hf));
			double edgeLen = sqrt((edgeEndPos - edgeStartPos).squared_length());
			if (edgeLen > (4.0 * avgEdgeLen / 3.0)) {
				// Split the edge
				mesh.SplitEdge(*edgeIter);
			}
			edgeIter = nextEdgeIter;
		}
		std::cout << "1) EDGES SPLIT\n";
		// Collapse all edges that have a length less than 4L/5 (L - avg edge length)
		for (auto edgeIter = mesh.edges_begin(); edgeIter != mesh.edges_end(); ++edgeIter) {
			//auto newEdgeIter = edgeIter;
			//++newEdgeIter;
			Halfedge_mesh::Halfedge_index hf = mesh.halfedge(*edgeIter);
			Point_3 edgeStartPos = mesh.point(mesh.source(hf));
			Point_3 edgeEndPos = mesh.point(mesh.target(hf));
			double edgeLen = sqrt((edgeEndPos - edgeStartPos).squared_length());
			if (edgeLen < (4.0 * avgEdgeLen / 5.0)) {
				mesh.CollapseEdge(*edgeIter);
			}
		}
		std::cout << "2) EDGES COLLAPSED\n";

		// Flip edges if it improves the degree of the neighbouring vertices
		// Calculate the deviation from the ideal degree i.e. 6
		// Get the degrees of the vertices of the edge and the opposite vertices opposite
		// to the edge. If flipping the edge decreases the total deviation, then flip the edge.
		for (auto edgeIter = mesh.edges_begin(); edgeIter != mesh.edges_end(); ++edgeIter) {
			Halfedge_mesh::Halfedge_index hf = mesh.halfedge(*edgeIter);
			Halfedge_mesh::Vertex_index edgeVert0 = mesh.source(hf), edgeVert1 = mesh.target(hf);
			Halfedge_mesh::Vertex_index oppVert0 = mesh.source(mesh.prev(hf)), oppVert1 = mesh.source(mesh.prev(mesh.opposite(hf)));

			int totalDegreeDeviation = abs((int)mesh.degree(edgeVert0) - 6) + abs((int)mesh.degree(edgeVert1) - 6)
				+ abs((int)mesh.degree(oppVert0) - 6) + abs((int)mesh.degree(oppVert1) - 6);
			// Calculate the vertex degree deviations that you would get after flipping the edge
			int totalDegreeDeviation_post = abs(((int)mesh.degree(edgeVert0) - 1) - 6) + abs(((int)mesh.degree(edgeVert1) - 1) - 6)
				+ abs(((int)mesh.degree(oppVert0) + 1) - 6) + abs(((int)mesh.degree(oppVert1) + 1) - 6);
			// If flipping the edge improves the total degree of the vertices. then flip the edge
			if (totalDegreeDeviation_post < totalDegreeDeviation)
				mesh.FlipEdge(*edgeIter);
		}
		std::cout << "3) EDGES FLIPPED\n";

		// Move vertices to the average of the neighbour vertices
		std::unordered_map<Halfedge_mesh::Vertex_index, Point_3> avgVertPositions;
		for (auto vertIter = mesh.vertices_begin(); vertIter != mesh.vertices_end(); ++vertIter) {
			// Calculate the average vertex position
			Point_3 avgVertPos(0.0, 0.0, 0.0);
			std::vector<Halfedge_mesh::Vertex_index> neighbourVertices;
			// Get the vertices around this vertex
			mesh.VerticesConnectedToVertex(*vertIter, neighbourVertices);
			for (auto neighbourIter : neighbourVertices) {
				avgVertPos += mesh.point(neighbourIter);
			}
			avgVertPos = avgVertPos / (double)neighbourVertices.size();

			avgVertPositions.insert(std::pair<Halfedge_mesh::Vertex_index, Point_3>(*vertIter, avgVertPos));
		}
		for (auto vertPosIter = avgVertPositions.begin(); vertPosIter != avgVertPositions.end(); ++vertPosIter) {
			Halfedge_mesh::Vertex_index vertIdx = vertPosIter->first;
			mesh.point(vertIdx) = vertPosIter->second;
		}


	}


	// Perform linear subdivision of the input mesh
	// Divides a n-sided polygon into n quads
	// By default does a linear subdivision, pass in a flag to do Catmull-Clark
	// The mesh is entirely rebuilt from a polygon soup after calculating the new vertex positions
	void meshResampler::QuadSubdivision(Halfedge_mesh& mesh, bool linear) {
		Halfedge_mesh newMesh;			// The new mesh to be built
		// 1) Create new vertices corr. to each edge in the original mesh
		std::unordered_map<Halfedge_mesh::Edge_index, Halfedge_mesh::Vertex_index> newEdgeVerts;
		for (auto edgeIter = mesh.edges_begin(); edgeIter != mesh.edges_end(); ++edgeIter) {
			Halfedge_mesh::Halfedge_index hf = mesh.halfedge(*edgeIter);
			Point_3 startPos = mesh.point(mesh.source(hf));
			Point_3 endPos = mesh.point(mesh.target(hf));
			// A new vertex is added at the center of old edges
			Point_3 newPos = (startPos + endPos) / 2.0;
			Halfedge_mesh::Vertex_index newVert = newMesh.add_vertex(newPos);
			newEdgeVerts.insert(std::pair<Halfedge_mesh::Edge_index, Halfedge_mesh::Vertex_index>(*edgeIter, newVert));
		}

		// 2) Create new vertices corr. to each face in the original mesh
		// A new vertex is added at the centroid of the face
		std::unordered_map<Halfedge_mesh::Face_index, Halfedge_mesh::Vertex_index> newFaceVerts;
		for (auto faceIter = mesh.faces_begin(); faceIter != mesh.faces_end(); ++faceIter) {
			Halfedge_mesh::Halfedge_index hf = mesh.halfedge(*faceIter);		// A halfedge belonging to the face
			Point_3 newPos(0.0, 0.0, 0.0);
			size_t numVerts = 0;
			// Calculate the centroid of the face
			Halfedge_mesh::Halfedge_index currHf = hf;
			do {
				Point_3 startPos = mesh.point(mesh.source(hf));
				newPos += startPos;
				++numVerts;
				// Move onto the next edge
				currHf = mesh.next(currHf);
			} while (currHf != hf);
			// Average of the vertices in the face
			newPos = newPos / (double)numVerts;
			Halfedge_mesh::Vertex_index newVert = newMesh.add_vertex(newPos);
			newFaceVerts.insert(std::pair<Halfedge_mesh::Face_index, Halfedge_mesh::Vertex_index>(*faceIter, newVert));
		}

		// 3) Create new vertices corr. to the original vertices of the mesh
		std::unordered_map<Halfedge_mesh::Vertex_index, Halfedge_mesh::Vertex_index> newVerts;
		for (auto vertIter = mesh.vertices_begin(); vertIter != mesh.vertices_end(); ++vertIter) {
			Point_3 newPos = mesh.point(*vertIter);
			if (!linear) {
				// TODO
				// Catmull-Clark subdivision to be done
			}
			Halfedge_mesh::Vertex_index newVert = newMesh.add_vertex(newPos);
			newVerts.insert(std::pair<Halfedge_mesh::Vertex_index, Halfedge_mesh::Vertex_index>(*vertIter, newVert));
		}

		// 4) Construct new quads from the new vertex positions calculated
		// A new quad is constructed from the following -
		// a) 1 new vertex corr. to an edge of the original mesh
		// b) 1 new vertex corr. to an old vertex of the original mesh
		// c) 1 new vertex corr. to an edge of the original mesh
		// d) 1 new vertex corr. to the face of the original mesh
		int newFaces = 0;
		for (auto faceIter = mesh.faces_begin(); faceIter != mesh.faces_end(); ++faceIter) {
			Halfedge_mesh::Halfedge_index hf = mesh.halfedge(*faceIter);		// Halfedge in the face
			Halfedge_mesh::Halfedge_index currHf = hf;
			do {
				// Container for the vertices in the new mesh for constructing a new quad face
				std::vector<Halfedge_mesh::Vertex_index> vertsInNewFace(4);
				vertsInNewFace[0] = newEdgeVerts.at(mesh.edge(currHf));				// New vert corr. to an old edge
				vertsInNewFace[1] = newVerts.at(mesh.target(currHf));				// New vert corr. to an old vert
				vertsInNewFace[2] = newEdgeVerts.at(mesh.edge(mesh.next(currHf)));	// New vert corr. to an old edge
				vertsInNewFace[3] = newFaceVerts.at(*faceIter);						// New vert corr. to an old face

				newMesh.add_face(vertsInNewFace);
				++newFaces;
				//newMesh.add_face(newEdgeVerts.at(mesh.edge(hf)), newVerts.at(mesh.target(hf)),
				//	newEdgeVerts.at(mesh.edge(mesh.next(hf))), newFaceVerts.at(*faceIter));

				// Move onto the next edge
				currHf = mesh.next(currHf);
			} while (currHf != hf);
		}

		// Calculate the face and vertex normals
		newMesh.faceNormals.clear();
		for (auto faceIter = newMesh.faces_begin(); faceIter != newMesh.faces_end(); ++faceIter) {
			Vector_3 faceNorm = newMesh.GetFaceNormal(*faceIter);
			newMesh.faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(*faceIter, faceNorm));
		}

		newMesh.vertexNormals.clear();
		for (auto vertIter = newMesh.vertices_end(); vertIter != newMesh.vertices_end(); ++vertIter) {
			Vector_3 vertNorm = newMesh.GetVertexNormal(*vertIter);
			newMesh.vertexNormals.insert(std::pair<Halfedge_mesh::Vertex_index, Vector_3>(*vertIter, vertNorm));
		}

		std::cout << "\nPRE-SUBDIVISION\n";
		std::cout << "NUM_VERTS: " << mesh.number_of_vertices() << "\n";
		std::cout << "NUM_EDGES: " << mesh.number_of_edges() << "\n";
		std::cout << "NUM_FACES: " << mesh.number_of_faces() << "\n\n";

		std::cout << "\nPOST-SUBDIVISION\n";
		std::cout << "NUM_VERTS: " << newMesh.number_of_vertices() << "\n";
		std::cout << "NUM_EDGES: " << newMesh.number_of_edges() << "\n";
		std::cout << "NUM_FACES: " << newMesh.number_of_faces() << "\n\n";

		// TODO: Check if the elements of the old mesh needs to be deleted
		// Assign the new mesh to the old mesh
		mesh = newMesh;

		std::cout << "\nPOST-POST-SUBDIVISION\n";
		std::cout << "NUM_VERTS: " << mesh.number_of_vertices() << "\n";
		std::cout << "NUM_EDGES: " << mesh.number_of_edges() << "\n";
		std::cout << "NUM_FACES: " << mesh.number_of_faces() << "\n\n";
	}
}
