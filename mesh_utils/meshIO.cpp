#include "meshIO.h"

namespace MeshUtils {

	// Reads an input .off file and gets the Surface_mesh
	// Uses the CGAL-provided reader that directly populates the Surface_mesh
	/* static */ void meshIO::ReadOff(Halfedge_mesh& mesh, std::string offFilePath, bool useCGALReader) {
		std::ifstream offFile(offFilePath, std::ifstream::in);
		//if(false && useCGALReader)
		//	CGAL::read_off(offFile, mesh);
		if (true) {
			std::string line;
			std::getline(offFile, line);
			if (line != std::string("OFF"))
				return;
			// Reading the number of vertices and faces
			int numVerts, numFaces, numEdges;
			std::getline(offFile, line);
			std::istringstream iss(line);
			iss >> numVerts >> numFaces >> numEdges;
			// Reading the vertices
			std::vector<Halfedge_mesh::Vertex_index> vertices;
			int vertCount = 0;
			while (vertCount < numVerts) {
				std::getline(offFile, line);
				double xPos, yPos, zPos;
				std::istringstream iss(line);
				iss >> xPos >> yPos >> zPos;
				Halfedge_mesh::Vertex_index vertIdx = mesh.add_vertex(Point_3(xPos, yPos, zPos));
				vertices.push_back(vertIdx);
				vertCount++;
			}

			// Reading the faces
			int faceCount = 0;
			while (faceCount < numFaces) {
				int numSides;				// number of sides in the polygon
				std::getline(offFile, line);
				std::istringstream iss(line);
				iss >> numSides;
				std::vector<Halfedge_mesh::Vertex_index> vertsInFace(numSides);
				for (int i = 0; i < numSides; i++) {
					int vertIdx;
					iss >> vertIdx;
					vertsInFace[i] = vertices[vertIdx];
				}
				// Add the new face
				// It assigns the required halfedges or adjust them if already existing
				mesh.add_face(vertsInFace);
				faceCount++;
			}
		}

		// Get face normals
		for (auto Face_index = mesh.faces_begin(); Face_index != mesh.faces_end(); ++Face_index) {
			mesh.faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(*Face_index, mesh.GetFaceNormal(*Face_index)));
		}

		// Get the vertex normals
		for (auto vertIdx = mesh.vertices_begin(); vertIdx != mesh.vertices_end(); ++vertIdx) {
			mesh.vertexNormals.insert(std::pair<Halfedge_mesh::Vertex_index, Vector_3>(*vertIdx, mesh.GetVertexNormal(*vertIdx)));
		}
	}

#if 0
	// Write the halfedge mesh to an .off file
	/*static*/ void meshIO::WriteOff(Halfedge_mesh& mesh, std::string outOffFilePath) {
		std::ofstream outOffFile(outOffFilePath, std::ofstream::out);
		CGAL::write_off(outOffFile, mesh);
	}
#endif

	// Reads an input .ply file and gets the Surface_mesh
	// Uses the CGAL-provided reader that directly populates the Surface_mesh
	/* static */ void meshIO::ReadPly(Halfedge_mesh& mesh, std::string plyFilePath) {
		std::ifstream plyFile(plyFilePath, std::ifstream::in);
		//CGAL::read_PLY(plyFile, mesh);

		// Get face normals
		for (auto Face_index = mesh.faces_begin(); Face_index != mesh.faces_end(); ++Face_index) {
			mesh.faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(*Face_index, mesh.GetFaceNormal(*Face_index)));
		}

		// Get the vertex normals
		for (auto vertIdx = mesh.vertices_begin(); vertIdx != mesh.vertices_end(); ++vertIdx) {
			mesh.vertexNormals.insert(std::pair<Halfedge_mesh::Vertex_index, Vector_3>(*vertIdx, mesh.GetVertexNormal(*vertIdx)));
		}
	}

	// TODO: Work out the STL reader properly
	// Reads an input STL file and populates the halfedge DS
	/* static */ void meshIO::ReadSTL(Halfedge_mesh& mesh, std::string stlFilePath) {

		// Reading the STL file (Using CGAL function)
		std::ifstream stlFile(stlFilePath, std::ifstream::in);
		std::vector<std::array<double, 3>> points;
		std::vector<std::array<int, 3>> triangles;	// Stores indices of verts for each triangle

		// Using binary stl parser (https://github.com/dillonhuff/stl_parser)
		auto stlInfo = stl::parse_stl(stlFilePath);


		// Populating the halfedge mesh
		// Add new vertices

		// Iterating over each triangle
		for (const auto& triangle : stlInfo.triangles) {

			Halfedge_mesh::Vertex_index v1 = mesh.add_vertex(Point_3(triangle.v1.x, triangle.v1.y, triangle.v1.z));
			Halfedge_mesh::Vertex_index v2 = mesh.add_vertex(Point_3(triangle.v2.x, triangle.v2.y, triangle.v2.z));
			Halfedge_mesh::Vertex_index v3 = mesh.add_vertex(Point_3(triangle.v3.x, triangle.v3.y, triangle.v3.z));

			// Adding the face to the mesh
			Halfedge_mesh::Face_index faceIdx = mesh.add_face(v1, v2, v3);
			double normalX = triangle.normal.x;
			double normalY = triangle.normal.y;
			double normalZ = triangle.normal.z;

			// Adding the face normal to the property map
			mesh.faceNormals.insert(std::pair<Halfedge_mesh::Face_index, Vector_3>(faceIdx, Vector_3(triangle.normal.x, triangle.normal.y, triangle.normal.z)));
		}

		// Calculating the vertex normals
		for (auto Vertex_index = mesh.vertices_begin(); Vertex_index != mesh.vertices_end(); ++Vertex_index) {
			std::vector<Halfedge_mesh::Face_index> neighbourFaceIdxs;
			mesh.FacesAroundVertex(*Vertex_index, neighbourFaceIdxs);
			Vector_3 vertNormal(0.0, 0.0, 0.0);
			for (auto faceIdx : neighbourFaceIdxs) {
				vertNormal += mesh.faceNormals.at(faceIdx);
			}
			vertNormal /= (double)neighbourFaceIdxs.size();
			mesh.vertexNormals.insert(std::pair<Halfedge_mesh::Vertex_index, Vector_3>(*Vertex_index, vertNormal));
		}
	}
}
