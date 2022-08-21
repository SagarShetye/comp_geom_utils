#pragma once
#ifndef __MESH_RENDERER__
#define __MESH_RENDERER__

#define _USE_MATH_DEFINES

#include "halfedgeMesh.h"
#include "shader.h"
#include "RenderingUtils.h"

// TODO: Update the drawing methods to use vertex buffers (using glDrawArrays)
namespace MeshUtils {
	// TODO: Give an option to render without using shaders
	// TODO: Texture related stuff
	// Helper to draw the mesh on the screen
	// and handle the logistics of picking edges/faces
	class meshRenderer {
	public:
		// Some flags to determine rendering related options
		bool useShader = true;
		bool renderFaces = true, renderEdges = true;
		bool useTextures = false;

		// OpenGL vertex arrays and buffers objects
		unsigned int VBO1, VBO2, VAO1, VAO2, EBO1, EBO2;

		// To store the vertices + indices for rendering faces/edges
		std::vector<float> faceVertices, edgeVertices;
		std::vector<unsigned int> faceIndices, edgeIndices;

		// Reference to the shader object. The caller will be responsible
		// for loading/compiling/linking the shader. The renderer object
		// will just use it.
		Shader* shader;

		// The model, view and projection matrices
		Eigen::Matrix4f* modelMat, * viewMat, * projectionMat;

		RenderingUtils::colour faceColour, edgeColour;
		RenderingUtils::colour selectedFaceColour, selectedEdgeColour;
		// Indices of the selected entities to draw in a different style/colour
		Halfedge_mesh::Face_index selectedFaceIdx;
		Halfedge_mesh::Edge_index selectedEdgeIdx;
		Halfedge_mesh* mesh;

		// Constructors
		meshRenderer();
		//meshRenderer(Halfedge_mesh*, Shader*);
		meshRenderer(Halfedge_mesh*, Eigen::Matrix4f*, Eigen::Matrix4f*, 
			Eigen::Matrix4f*, Shader*, RenderingUtils::colour = RenderingUtils::colour(255, 255, 255), 
			RenderingUtils::colour = RenderingUtils::colour(0, 0, 0),
			RenderingUtils::colour = RenderingUtils::colour(153, 255, 255), RenderingUtils::colour = RenderingUtils::colour(128, 0, 128));

		// Set up the Vertex arrays and Buffer objects that OpenGL would require for rendering
		void Initialize();

		void DrawMesh() const;
		// Render the edges of the mesh
		void DrawEdges() const;
		// Render the faces of the mesh
		void DrawFaces(bool smooth) const;
		// Draw an arrow representing an halfedge
		void DrawHalfedgeArrow(const Halfedge_mesh::Halfedge_index h);

		// Turn off Face rendering
		inline void toggleFaceRendering() {
			renderFaces = !renderFaces;
		}

		// Turn off Edge rendering
		inline void toggleEdgeRendering() {
			renderEdges = !renderEdges;
		}

	private:
		size_t numFaces, numEdges;
		// This function would populate the vertex arrays for faces & edges for rendering.
		// Would also generate indices that would be used by glDrawElements()
		void generateVertexArraysForRendering();
	};
}

#endif // !__MESH_RENDERER__
