#ifndef __RENDERING_UTILS__
#define __RENDERING_UTILS__

#define _USE_MATH_DEFINES

#define NOMINMAX		// To disable pre-existing macros for min, max, thanks WINDOWS!

#include "fssimplewindow.h"
#include "GeomUtils.h"

#include <Eigen/Dense>

#include <math.h>

namespace RenderingUtils{
	void setProjectionMatrix(const float& angleOfView, const float& nearClip, const float& farClip,
		const float& aspectRatio, Eigen::Matrix4f& mat);

	// TODO: Finish this and use this as the default camera
	// A trackball camera class
	class TrackballCamera
	{
	public:
		TrackballCamera() = delete;

		Vector_3 pos = Vector_3(0.0f, 0.0f, -5.0f);	// Camera position

		Vector_3 targetPos = Vector_3(0.0f, 0.0f, 0.0f),
			upVector = Vector_3(0.0f, 1.0f, 0.0f);		// Specifies the camera orientation
														// useful while rotating the scene

		float phi = M_PI_2, theta = M_PI_2;	// Spherical coords (for rotation)
											// 'phi' is the angle wrt +y
											// 'theta' is the angle wrt +x
		float radius = 1.0f;
		bool viewNeedsUpdate = false;
		//bool up = 1.0f;						// Component along the Y-axis (i.e. 'up' direction)
		Eigen::Matrix4f viewMatrix = Eigen::Matrix4f::Identity();

	private:
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
		float x = 0.0f, y = 0.0f, z = 0.0f;
		//double h, p, b;
		float vFOV = 5.0;

		Vector_3 forwardVector = Vector_3(0.0f, 0.0f, -1.0f),
			upVector = Vector_3(0.0f, 1.0f, 0.0f);	// Specifies the camera orientation
													// useful while rotating the scene

		bool isDragging = false;			// For moving the model when the mouse is pressed
		float prevMouse_x, prevMouse_y;
		float phi = 0.0f, theta = 0.0f;
		float radius = 1.0f;

	public:
		camera() = delete;

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

	// Keeps track of mouse movements and changes camera view accordingly
	class MouseHandler {
	private:
		float mouseLastPos_x, mouseLastPos_y;
		float scrollFactor;						// Resolution for scrolling (useful for zooming view)
		float rotationFactor;					// Resolution for rotation
		float panFactor;					// Resolution for panning
		bool lbPressed = false;
	public:
		MouseHandler() {
			mouseLastPos_x = 0.0f;
			mouseLastPos_y = 0.0f;
			scrollFactor = 0.25f;
			rotationFactor = 0.01f;
			panFactor = 0.05f;
		};
		void checkForUpdate(TrackballCamera &cam);

		//// Determine if the cam is to be moved, 
		//void moveMouse(float lb, float mb, float mx, float my);
	};
}
#endif	// __RENDERING_UTILS__
