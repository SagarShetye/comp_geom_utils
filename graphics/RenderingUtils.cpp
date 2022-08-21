#include "RenderingUtils.h"


namespace RenderingUtils{
	// Camera set up at a certain position
	// (Make it necessary to call this cstor)
	camera::camera(float posx, float posy, float posz){
		x = posx;
		y = posy;
		z = posz;
	}

	const Vector_3& camera::GetForwardVector() const {
		return forwardVector;
	}

	const Vector_3& camera::GetUpVector() const {
		return upVector;
	}

	// To move the view
	bool camera::moveView(Eigen::Matrix4f& modelMat, Eigen::Matrix4f& viewMat) {
		double oldCam_x = x, oldCam_y = y, oldCam_z = z;
		bool isMoved = false;

		Vector_3 yawAxis = GetUpVector();
		Vector_3 forwardVec = GetForwardVector();
		Vector_3 pitchAxis = CGAL::cross_product(forwardVec, yawAxis);

		float pitchRotAngle = 0.0f, yawRotAngle = 0.0f;
		float transX = 0.0f, transY = 0.0f, transZ = 0.0f;		// This corr. to the movement of the camera
																// For modifying the view matrix, inverse of this would be used

		if (0 != FsGetKeyState(FSKEY_LEFT)) {
			// Rotation about the 'up' vector
			yawRotAngle = -M_PI / 180.0;
			isMoved = true;
		}
		if (0 != FsGetKeyState(FSKEY_RIGHT)) {
			// Rotation about the 'up' vector
			yawRotAngle = M_PI / 180.0;
			isMoved = true;
		}
		if (0 != FsGetKeyState(FSKEY_UP)) {
			// Rotation about the vector perp to the 'up' & 'forward' vectors
			pitchRotAngle = -M_PI / 180.0;
			isMoved = true;
		}
		if (0 != FsGetKeyState(FSKEY_DOWN)) {
			// Rotation about the vector perp to the 'up' & 'forward' vectors
			pitchRotAngle = M_PI / 180.0;
			isMoved = true;
		}
		if (0 != FsGetKeyState(FSKEY_W)) {
			Vector_3 forwardVec = GetForwardVector();
			transX += forwardVec.x() * 0.7;
			transY += forwardVec.y() * 0.7;
			transZ += forwardVec.z() * 0.7;
			isMoved = true;
		}
		if (0 != FsGetKeyState(FSKEY_S)) {
			Vector_3 forwardVec = GetForwardVector();
			transX -= forwardVec.x() * 0.7;
			transY -= forwardVec.y() * 0.7;
			transZ -= forwardVec.z() * 0.7;
			isMoved = true;
		}
		// Move left
		if (0 != FsGetKeyState(FSKEY_A)) {
			Vector_3 upVec = GetUpVector();
			Vector_3 forwardVec = GetForwardVector();
			Vector_3 leftVec = CGAL::cross_product(upVec, forwardVec);

			transX += leftVec.x() * 0.7;
			transY += leftVec.y() * 0.7;
			transZ += leftVec.z() * 0.7;
			isMoved = true;
		}
		// Move Right
		if (0 != FsGetKeyState(FSKEY_D)) {
			Vector_3 upVec = GetUpVector();
			Vector_3 forwardVec = GetForwardVector();
			Vector_3 rightVec = CGAL::cross_product(forwardVec, upVec);

			transX += rightVec.x() * 0.7;
			transY += rightVec.y() * 0.7;
			transZ += rightVec.z() * 0.7;
			isMoved = true;
		}
		// Move Up
		if (0 != FsGetKeyState(FSKEY_R)) {
			Vector_3 upVec = GetUpVector();

			transX += upVec.x() * 0.7;
			transY += upVec.y() * 0.7;
			transZ += upVec.z() * 0.7;
			isMoved = true;
		}
		// Move down
		if (0 != FsGetKeyState(FSKEY_F)) {
			double upx, upy, upz;
			Vector_3 upVec = GetUpVector();

			transX -= upVec.x() * 0.7;
			transY -= upVec.y() * 0.7;
			transZ -= upVec.z() * 0.7;
			isMoved = true;
		}

		if (isMoved) {
			// Rotate the scene wrt the camera
			Eigen::AngleAxisf pitchAngle(pitchRotAngle, Eigen::Vector3f(pitchAxis.x(), pitchAxis.y(), pitchAxis.z()));
			Eigen::AngleAxisf yawAngle(yawRotAngle, Eigen::Vector3f(yawAxis.x(), yawAxis.y(), yawAxis.z()));

			// Using Quaternions
			Eigen::Quaternion<float> q = yawAngle * pitchAngle;
			Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
			rotationMatrix.block<3, 3>(0, 0) = q.matrix();

			Eigen::Matrix4f cameraTransMat = Eigen::Matrix4f::Identity();
			cameraTransMat(0, 3) = x;	cameraTransMat(1, 3) = y;	cameraTransMat(2, 3) = z;
			rotationMatrix = cameraTransMat.inverse() * rotationMatrix * cameraTransMat;

			// Move the scene relative to the camera
			Eigen::Matrix4f transMat = Eigen::Matrix4f::Identity();
			transMat(0, 3) = -transX;	transMat(1, 3) = -transY;	transMat(2, 3) = -transZ;
			viewMat = rotationMatrix * transMat * viewMat;
		}

		return isMoved;
	}


	// TODO
	// TEMP - Trackball camera related stuff
	// Camera related definitions
	// TrackballCamera set up at a certain position
	// (Make it necessary to call this cstor)
	TrackballCamera::TrackballCamera(float posx, float posy, float posz){
		pos = Vector_3(posx, posy, posz);
		radius = sqrt(posx * posx + posy * posy + posz * posz);
	}

	// TODO: Check after pan
	void TrackballCamera::rotate(float dTheta, float dPhi) {
		viewNeedsUpdate = true;
		float up = upVector.y();

		if (up > 0.0f) {
			theta += dTheta;
		}
		else {
			theta -= dTheta;
		}

		phi += dPhi;

		// Keep phi within -2PI to +2PI for easy 'up' comparison
		if (phi > 2.0 * M_PI) {
			phi -= 2.0f * M_PI;
		}
		else if (phi < -2.0f * M_PI) {
			phi += 2.0f * M_PI;
		}

		// If phi is between 0 to PI or -PI to -2PI, make 'up' be positive Y, other wise make it negative Y
		if ((phi > 0 && phi < M_PI) || (phi < -M_PI && phi > -2.0f * M_PI)) {
			up = 1.0f;
		}
		else {
			up = -1.0f;
		}

		upVector = Vector_3(upVector.x(), up, upVector.z());
	}

	void TrackballCamera::zoom(float distance) {
		viewNeedsUpdate = true;

		radius -= distance;

		// Don't let the radius go negative
		// If it does, re-project our target down the look vector
		if (radius <= 0.0f) {
			radius = 30.0f;
			Vector_3 look = targetPos - pos;
			look = look / sqrt(look.squared_length());
			targetPos = targetPos + 30.f * look;
		}
	}

	void TrackballCamera::pan(float dx, float dy) {
		viewNeedsUpdate = true;

		updateCartesianCoords();

		Vector_3 forward = targetPos - pos;
		GeomUtils::normalise(forward);
		Vector_3 side = CGAL::cross_product(forward, upVector);
		GeomUtils::normalise(side);

		upVector = CGAL::cross_product(side, forward);

		targetPos = targetPos + (dx * side + dy * upVector);
	}

	// Update the view matrix corr. to the camera
	void TrackballCamera::updateViewMatrix() {
		if (viewNeedsUpdate) {
			updateCartesianCoords();

			Vector_3 forward = targetPos - pos;
			GeomUtils::normalise(forward);
			Vector_3 side = CGAL::cross_product(forward, upVector);
			GeomUtils::normalise(side);

			upVector = CGAL::cross_product(side, forward);

			// Update the view matrix
			//viewMatrix = Eigen::Matrix4f::Identity();
			viewMatrix(0, 0) = side.x();		viewMatrix(0, 1) = side.y();		viewMatrix(0, 2) = side.z();		viewMatrix(0, 3) = 0.0f;
			viewMatrix(1, 0) = upVector.x();	viewMatrix(1, 1) = upVector.y();	viewMatrix(1, 2) = upVector.z();	viewMatrix(1, 3) = 0.0f;
			viewMatrix(2, 0) = -forward.x();	viewMatrix(2, 1) = -forward.y();	viewMatrix(2, 2) = -forward.z();	viewMatrix(2, 3) = 0.0f;
			viewMatrix(3, 0) = 0.0f;			viewMatrix(3, 1) = 0.0f;			viewMatrix(3, 2) = 0.0f;			viewMatrix(3, 3) = 1.0f;

			// Set up the translation corr. to the cam position
			Eigen::Matrix4f transMat = Eigen::Matrix4f::Identity();
			transMat(0, 3) = -pos.x();
			transMat(1, 3) = -pos.y();
			transMat(2, 3) = -pos.z();

			viewMatrix = viewMatrix * transMat;
		}
	}

	void TrackballCamera::updateCartesianCoords() {
		// Update the TrackballCamera position in cartesian coordinates
		//float x = radius * sinf(phi) * sinf(theta);
		//float z = radius * cosf(phi);
		//float y = radius * sinf(phi) * cosf(theta);


		float x = radius * sinf(phi) * cosf(theta);
		float y = radius * cosf(phi);
		float z = radius * sinf(phi) * sinf(theta);

		pos = Vector_3(x, y, z);
	}

	void MouseHandler::checkForUpdate(TrackballCamera &cam) {
		bool isMoved = false;

		// Rotate
		int lb, mb, rb, mx, my;
		FsGetMouseState(lb, mb, rb, mx, my);
		// Rotate the model
		if (lb) {
			float dx = mx - mouseLastPos_x, dy = my - mouseLastPos_y;
			cam.rotate(dx * rotationFactor, -dy * rotationFactor);
			isMoved = true;
		}
		// Pan
		else if (mb) {
			float dx = mx - mouseLastPos_x, dy = my - mouseLastPos_y;
			cam.pan(-dx * panFactor, dy * panFactor);
			isMoved = true;
		}
		// Zoom
		// TODO: Change this to use scroll instead
		else if (rb) {
			float dy = my - mouseLastPos_y;
			cam.zoom(dy * scrollFactor);
			isMoved = true;
		}
		mouseLastPos_x = mx;
		mouseLastPos_y = my;

		// Update the view matrix (if needed)
		if (!isMoved)
			cam.viewNeedsUpdate = false;
		cam.updateViewMatrix();
	}
};