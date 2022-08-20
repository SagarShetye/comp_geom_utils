#pragma once
#ifndef __MESH_RENDERER__
#define __MESH_RENDERER__

#define _USE_MATH_DEFINES
#include <math.h>
#include "halfedgeMesh.h"
#include <Eigen/Dense>
#include "shader.h"
#include "fssimplewindow.h"

// TODO: Update the drawing methods to use vertex buffers (using glDrawArrays)

// TODO: Finish this and use this as the default camera
// A trackball camera class
class TrackballCamera
{
public:
	Vector_3 pos;						// Camera position

	Vector_3 targetPos, upVector;		// Specifies the camera orientation
										// useful while rotating the scene

	float phi = M_PI_2, theta = M_PI_2;	// Spherical coords (for rotation)
										// 'phi' is the angle wrt +y
										// 'theta' is the angle wrt +x
	float radius = 1.0f;
	bool viewNeedsUpdate = false;
	//bool up = 1.0f;						// Component along the Y-axis (i.e. 'up' direction)
	Eigen::Matrix4f viewMatrix;

private:
	TrackballCamera();

	void updateCartesianCoords();

public:
	TrackballCamera(float, float, float);		// initialize the camera at a certain position

	//const Vector_3& GetForwardVector() const;
	//const Vector_3& GetUpVector() const;

	//bool moveView(Eigen::Matrix4f &modelMat, Eigen::Matrix4f &viewMat);

	void rotate(float dTheta, float dPhi);

	void pan(float dx, float dy);

	void zoom(float distance);

	void updateViewMatrix();

	//void Reset();
};


class camera
{
public:
	float x, y, z;
	//double h, p, b;
	float vFOV = 5.0;

	Vector_3 forwardVector, upVector;	// Specifies the camera orientation
										// useful while rotating the scene

	bool isDragging = false;			// For moving the model when the mouse is pressed
	float prevMouse_x, prevMouse_y;
	float phi = 0.0f, theta = 0.0f;
	float radius = 1.0f;

public:
	camera();

	camera(float, float, float);		// initialize the camera at a certain position
	//void Initialize(void);
	//void SetUpCameraProjection(void);
	//void SetUpCameraTransformation(void);

	const Vector_3& GetForwardVector() const;
	const Vector_3& GetUpVector() const;

	bool moveView(Eigen::Matrix4f& modelMat, Eigen::Matrix4f& viewMat);

	//void Reset();
};


struct colour {
	size_t r, g, b;
	colour() : r(0), g(0), b(0) {};
	colour(size_t red, size_t green, size_t blue) {
		this->r = red;
		this->g = green;
		this->b = blue;
	};
};

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

		colour faceColour, edgeColour;
		colour selectedFaceColour, selectedEdgeColour;
		// Indices of the selected entities to draw in a different style/colour
		Halfedge_mesh::Face_index selectedFaceIdx;
		Halfedge_mesh::Edge_index selectedEdgeIdx;
		Halfedge_mesh* mesh;

		// Constructors
		meshRenderer();
		//meshRenderer(Halfedge_mesh*, Shader*);
		meshRenderer(Halfedge_mesh*, Eigen::Matrix4f*, Eigen::Matrix4f*, Eigen::Matrix4f*, Shader*, colour = colour(255, 255, 255), colour = colour(0, 0, 0),
			colour = colour(153, 255, 255), colour = colour(128, 0, 128));

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
