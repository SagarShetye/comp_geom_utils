#include "meshRenderer.h"

// TEMP for debugging
#if 1
#include <fstream>
#endif

// Camera related definitions
camera::camera() {
	// Default camera position
	x = 0.0f; y = 0.0f; z = 0.0f;

	forwardVector = Vector_3(0.0f, 0.0f, -1.0f);
	upVector = Vector_3(0.0f, 1.0f, 0.0f);
}

// Camera set up at a certain position
// (Make it necessary to call this cstor)
camera::camera(float posx, float posy, float posz) : camera() {
	x = posx;
	y = posy;
	z = posz;
}

const Vector_3& camera::GetForwardVector() const{
	return forwardVector;
}

const Vector_3& camera::GetUpVector() const{
	return upVector;
}

// To move the view
bool camera::moveView(Eigen::Matrix4f &modelMat, Eigen::Matrix4f &viewMat) {
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
		transX += forwardVec.x()*0.7;
		transY += forwardVec.y()*0.7;
		transZ += forwardVec.z()*0.7;
		isMoved = true;
	}
	if (0 != FsGetKeyState(FSKEY_S)) {
		Vector_3 forwardVec = GetForwardVector();
		transX -= forwardVec.x()*0.7;
		transY -= forwardVec.y()*0.7;
		transZ -= forwardVec.z()*0.7;
		isMoved = true;
	}
	// Move left
	if (0 != FsGetKeyState(FSKEY_A)) {
		Vector_3 upVec = GetUpVector();
		Vector_3 forwardVec = GetForwardVector();
		Vector_3 leftVec = CGAL::cross_product(upVec, forwardVec);

		transX += leftVec.x()*0.7;
		transY += leftVec.y()*0.7;
		transZ += leftVec.z()*0.7;
		isMoved = true;
	}
	// Move Right
	if (0 != FsGetKeyState(FSKEY_D)) {
		Vector_3 upVec = GetUpVector();
		Vector_3 forwardVec = GetForwardVector();
		Vector_3 rightVec = CGAL::cross_product(forwardVec, upVec);

		transX += rightVec.x()*0.7;
		transY += rightVec.y()*0.7;
		transZ += rightVec.z()*0.7;
		isMoved = true;
	}
	// Move Up
	if (0 != FsGetKeyState(FSKEY_R)) {
		Vector_3 upVec = GetUpVector();

		transX += upVec.x()*0.7;
		transY += upVec.y()*0.7;
		transZ += upVec.z()*0.7;
		isMoved = true;
	}
	// Move down
	if (0 != FsGetKeyState(FSKEY_F)) {
		double upx, upy, upz;
		Vector_3 upVec = GetUpVector();

		transX -= upVec.x()*0.7;
		transY -= upVec.y()*0.7;
		transZ -= upVec.z()*0.7;
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
TrackballCamera::TrackballCamera() {
	// Default TrackballCamera position
	pos = Vector_3(0.0f, 0.0f, -5.0f);

	targetPos = Vector_3(0.0f, 0.0f, 0.0f);
	upVector = Vector_3(0.0f, 1.0f, 0.0f);
	radius = 0.0f;
	viewMatrix = Eigen::Matrix4f::Identity();
}

// TrackballCamera set up at a certain position
// (Make it necessary to call this cstor)
TrackballCamera::TrackballCamera(float posx, float posy, float posz) : TrackballCamera() {
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
	if (phi > 2.0* M_PI) {
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
	MeshUtils::normalise(forward);
	Vector_3 side = CGAL::cross_product(forward, upVector);
	MeshUtils::normalise(side);

	upVector = CGAL::cross_product(side, forward);

	targetPos = targetPos + (dx * side + dy * upVector);
}

// Update the view matrix corr. to the camera
void TrackballCamera::updateViewMatrix() {
	if (viewNeedsUpdate) {
		updateCartesianCoords();

		Vector_3 forward = targetPos - pos;
		MeshUtils::normalise(forward);
		Vector_3 side = CGAL::cross_product(forward, upVector);
		MeshUtils::normalise(side);

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

// END TEMP

namespace MeshUtils {
	meshRenderer::meshRenderer() : mesh(nullptr), selectedFaceIdx(Halfedge_mesh::null_face()), selectedEdgeIdx(Halfedge_mesh::null_edge()),
		selectedFaceColour(153, 255, 255), selectedEdgeColour(128, 0, 128),
		faceColour(255, 255, 255), edgeColour(0, 0, 0) {

	}

	meshRenderer::meshRenderer(Halfedge_mesh* meshToRender, Eigen::Matrix4f* modelMatrix, Eigen::Matrix4f* viewMatrix, Eigen::Matrix4f* projectionMatrix,
		Shader* shaderObj /*= nullptr*/, colour faceCol /*= colour(255, 255, 255)*/, colour edgeCol /*= colour(0, 0, 0)*/,
		colour selectedFaceCol /*= colour(153, 255, 255)*/, colour selectedEdgeCol /*= colour(128, 0, 128)*/) {
		this->mesh = meshToRender;
		this->modelMat = modelMatrix;
		this->viewMat = viewMatrix;
		this->projectionMat = projectionMatrix;
		this->shader = shaderObj;
		this->faceColour = faceCol;
		this->edgeColour = edgeCol;
		this->selectedFaceColour = selectedFaceCol;
		this->selectedEdgeColour = selectedEdgeCol;

		// If no shader is provided, then draw using regular method
		if (!this->shader)
			useShader = false;

		// Set up the vertex arrays and other info for rendering
		Initialize();
	}

	//meshRenderer::meshRenderer(Halfedge_mesh *meshToRender, Shader *shaderObj) : selectedFaceIdx(Halfedge_mesh::null_face()), selectedEdgeIdx(Halfedge_mesh::null_edge()),
	//selectedFaceColour(153, 255, 255), selectedEdgeColour(128, 0, 128),
	//faceColour(255, 255, 255), edgeColour(0, 0, 0), mesh(meshToRender), shader(shaderObj) {
	//	//this->mesh = meshToRender;
	//}

	// Create vertex and index array for the faces & edges in the mesh
	// This would be used to create OpenGL vertex array objects
	void meshRenderer::generateVertexArraysForRendering() {
		faceVertices.resize(0);	edgeVertices.resize(0);
		faceIndices.resize(0);	edgeIndices.resize(0);

		// The number of faces and edges in the mesh
		numFaces = mesh->num_faces();
		numEdges = mesh->num_edges();


		// Add all the vertices in the mesh
	// Different colours for faces and edges, so different arrays are required!
		for (auto vertIter = mesh->vertices_begin(); vertIter != mesh->vertices_end(); ++vertIter) {
			Point_3 pt = mesh->point(*vertIter);

			// The vertices
			faceVertices.push_back(pt.x());	faceVertices.push_back(pt.y());	faceVertices.push_back(pt.z());
			edgeVertices.push_back(pt.x());	edgeVertices.push_back(pt.y());	edgeVertices.push_back(pt.z());

			// The colours
			faceVertices.push_back(faceColour.r / 255.);	faceVertices.push_back(faceColour.g / 255.);	faceVertices.push_back(faceColour.b / 255.);
			edgeVertices.push_back(edgeColour.r / 255.);	edgeVertices.push_back(edgeColour.g / 255.);	edgeVertices.push_back(edgeColour.b / 255.);

			// TODO:
			// The texture coordinates
			// (The texture domain ranges from [0, 1] in both the directions)
		}


		// Add all the indices of vertices in the FACES
		for (auto faceIter = mesh->faces_begin(); faceIter != mesh->faces_end(); ++faceIter) {
			Halfedge_mesh::Halfedge_index hf = mesh->halfedge(*faceIter);
			Halfedge_mesh::Vertex_index vertA = mesh->source(hf), vertB = mesh->source(mesh->next(hf)),
				vertC = mesh->source(mesh->prev(hf));

			faceIndices.push_back(vertA);
			faceIndices.push_back(vertB);
			faceIndices.push_back(vertC);

		}

		// Add all the indices of vertices in the EDGES
		for (auto edgeIter = mesh->edges_begin(); edgeIter != mesh->edges_end(); ++edgeIter) {
			Halfedge_mesh::Halfedge_index hf = mesh->halfedge(*edgeIter);
			Halfedge_mesh::Vertex_index vertA = mesh->source(hf), vertB = mesh->target(hf);

			edgeIndices.push_back(vertA);
			edgeIndices.push_back(vertB);
		}
	}



	// Set up the Vertex arrays and Buffer objects that OpenGL would require for rendering
	void meshRenderer::Initialize() {
		// Since positions, colours and textures are stored in a contigous block for each point
		unsigned int stride = 6;
		if (useTextures)
			stride = 8;

		glGenVertexArrays(1, &VAO1);			// The vertex atrribute object
											// Has info about vertex buffer obj and the element buffer
		glGenVertexArrays(1, &VAO2);			// The vertex atrribute object
												// Has info about vertex buffer obj and the element buffer

		glGenBuffers(1, &VBO1);				// Vertex buffer object
		glGenBuffers(1, &VBO2);				// Vertex buffer object

		glGenBuffers(1, &EBO1);				// Element buffer object
		glGenBuffers(1, &EBO2);				// Element buffer object


		// Generate arrays corr. to the face vertices and edge vertices
		generateVertexArraysForRendering();

		// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
		// For rendering the faces
		glBindVertexArray(VAO1);
		// Use this to specify the vertices that are to be drawn
		glBindBuffer(GL_ARRAY_BUFFER, VBO1);
		glBufferData(GL_ARRAY_BUFFER, faceVertices.size() * sizeof(float), faceVertices.data(), GL_STATIC_DRAW);

		// Specify the indices into the vertices array for the FACES
		// As a result, you dont need repeated vertices in that array.
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO1);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, faceIndices.size() * sizeof(unsigned int), faceIndices.data(), GL_STATIC_DRAW);


		if (useShader) {
			// Get the attrib pointer (positions) for the current buffer (Here, VAO1)
			auto vertAttrib = glGetAttribLocation(shader->programID, "vertex");
			glVertexAttribPointer(vertAttrib, 3, GL_FLOAT, GL_FALSE, stride * sizeof(float), (void*)0);
			glEnableVertexAttribArray(vertAttrib);
			// Get the colour attrib pointer for the vertices
			auto col = glGetAttribLocation(shader->programID, "inColour");
			glVertexAttribPointer(col, 3, GL_FLOAT, GL_FALSE, stride * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(col);

#if 0
			// Get the texture attribute pointer for the vertices
			auto tex = glGetAttribLocation(shader->programID, "inTexCoord");
			glVertexAttribPointer(tex, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
			glEnableVertexAttribArray(tex);
#endif
		}

		// For rendering the edges
		glBindVertexArray(VAO2);
		// Use this to specify the vertices that are to be drawn (for edges)
		glBindBuffer(GL_ARRAY_BUFFER, VBO2);
		glBufferData(GL_ARRAY_BUFFER, edgeVertices.size() * sizeof(float), edgeVertices.data(), GL_STATIC_DRAW);

		// TEMP
			// Specify the indices into the vertices array for the EDGES
			// As a result, you dont need repeated vertices in that array.
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO2);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, edgeIndices.size() * sizeof(unsigned int), edgeIndices.data(), GL_STATIC_DRAW);

		if (useShader) {
			// NOTE: The foll has to be done for each vertex array object that exists
			// Get the attrib pointer (positions) for the current buffer (Here, VAO2)
			auto vertAttrib = glGetAttribLocation(shader->programID, "vertex");
			glVertexAttribPointer(vertAttrib, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(vertAttrib);
			// Get the colour attrib pointer for the vertices
			auto col = glGetAttribLocation(shader->programID, "inColour");
			glVertexAttribPointer(col, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(col);
		}


		// You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
		// VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
		glBindVertexArray(VAO1);
		glBindVertexArray(VAO2);

	}


	// Draw faces in the mesh
	void meshRenderer::DrawFaces(bool smooth) const {
		// OLD RENDERING METHOD

#if 0
		GLfloat white[4] = { 1., 1., 1., 1. };
		GLfloat faceCol[4] = { (float)faceColour.r / 255., (float)faceColour.g / 255., (float)faceColour.b / 255., 1. };
		GLfloat highlightedFaceColor[4] = { (float)selectedFaceColour.r / 255., (float)selectedFaceColour.g / 255., (float)selectedFaceColour.b / 255., 1. };

		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
		// Iterating over each face
		for (auto Face_index = mesh->faces_begin(); Face_index != mesh->faces_end(); ++Face_index) {

			// Prevent z fighting (faces bleeding into edges and points).
			glEnable(GL_POLYGON_OFFSET_FILL);
			glPolygonOffset(1.0, 1.0);

			if (!smooth) {
				if (this->selectedFaceIdx == *Face_index)
					// Draw the selected face blue
					glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, highlightedFaceColor);
				else
					// Draw normal faces white
					glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, faceCol);
			}

			// TODO: Check if the face has been selected

			glBegin(GL_POLYGON);

			if (!smooth) {
				glDisable(GL_LIGHTING);
			}

			Vector_3 faceNorm = (mesh->faceNormals.find(*Face_index))->second;
			if (!smooth)
				glNormal3dv(&faceNorm.x());
			// Traversing the halfedges to get the vertices
			Halfedge_mesh::Halfedge_index startHalfedgeIdx = mesh->halfedge(*Face_index);
			Halfedge_mesh::Halfedge_index currHalfedgeIdx = startHalfedgeIdx;
			do {
				Halfedge_mesh::Vertex_index vertIdx = mesh->source(currHalfedgeIdx);
				Point_3 pt = mesh->point(vertIdx);
				glVertex3d(pt.x(), pt.y(), pt.z());
				// Move to the next halfedge
				currHalfedgeIdx = mesh->next(currHalfedgeIdx);
			} while (currHalfedgeIdx != startHalfedgeIdx);
			glEnd();
		}
		glFlush();
#endif

		// Element-wise rendering
		// Draw the faces
		glBindVertexArray(VAO1);
		glDrawElements(GL_TRIANGLES, faceIndices.size(), GL_UNSIGNED_INT, 0);
	}

	// Drawing edges in the mesh
	void meshRenderer::DrawEdges() const {
		// OLD RENDERING METHOD
#if 0
		glColor3ub(edgeColour.r, edgeColour.g, edgeColour.b);
		//glLineWidth(0.5);
		//// Draw the selected edge at a slightly elevated height
		//// i.e. shift the vertices slightly outward along the vertex normal
		//Halfedge_mesh::Vertex_index startVertIdx = Halfedge_mesh::null_vertex(),
		//								endVertIdx = Halfedge_mesh::null_vertex();
		Point_3 actualStartPos(0.0, 0.0, 0.0), actualEndPos(0.0, 0.0, 0.0);
		double vertexOffset = 0.001;
		bool edgeSelected = false;
		for (auto Edge_index = mesh->edges_begin(); Edge_index != mesh->edges_end(); ++Edge_index) {
			if (*Edge_index == this->selectedEdgeIdx) {
				glEnable(GL_LINE_SMOOTH);
				glColor3ub(selectedEdgeColour.r, selectedEdgeColour.g, selectedEdgeColour.b);
				glLineWidth(3.0);
			}
			else {
				glColor3ub(edgeColour.r, edgeColour.g, edgeColour.b);
				glLineWidth(1.0);
			}

			Halfedge_mesh::Halfedge_index halfEdgeIdx = mesh->halfedge(*Edge_index);
			Point_3 startVert = mesh->point(mesh->source(halfEdgeIdx));
			Point_3 endVert = mesh->point(mesh->source(mesh->opposite(halfEdgeIdx)));

			// TODO: Check if the edge has been selected

			glBegin(GL_LINES);
			glVertex3d(startVert.x(), startVert.y(), startVert.z());
			glVertex3d(endVert.x(), endVert.y(), endVert.z());
			glEnd();
			glDisable(GL_LINE_SMOOTH);
		}
#endif

		// Draw the edges
		glBindVertexArray(VAO2);
		glDrawElements(GL_LINES, edgeIndices.size(), GL_UNSIGNED_INT, 0);
	}

	// Drawing the mesh
	void meshRenderer::DrawMesh() const {

		// TODO: TEXTURES!
#if 0
	// Bind the texture
		glBindTexture(GL_TEXTURE_2D, texture);
#endif

		// Use the shader program
		glUseProgram(shader->programID);

		// Apply Transformations
		auto modelMatLoc = glGetUniformLocation(shader->programID, "model");
		glUniformMatrix4fv(modelMatLoc, 1, GL_FALSE, modelMat->data());

		auto viewMatLoc = glGetUniformLocation(shader->programID, "view");
		glUniformMatrix4fv(viewMatLoc, 1, GL_FALSE, viewMat->data());

		auto projMatLoc = glGetUniformLocation(shader->programID, "projection");
		glUniformMatrix4fv(projMatLoc, 1, GL_FALSE, projectionMat->data());

		if (renderFaces) {
			// Enable lighting for faces
			//glEnable(GL_LIGHTING);
			DrawFaces(false);
		}

		//glEnable(GL_BLEND);
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		if (renderEdges) {
			// Edges are drawn with flat shading.
			//glDisable(GL_LIGHTING);
			DrawEdges();
		}
	}


	// TEMP - Draw an arrow representing a halfedge
	void meshRenderer::DrawHalfedgeArrow(const Halfedge_mesh::Halfedge_index h) {

		const Point_3& p0 = /*h->vertex()->position*/					mesh->point(mesh->source(h));
		const Point_3& p1 = /*h->next()->vertex()->position*/			mesh->point(mesh->source(mesh->next(h)));
		const Point_3& p2 = /*h->next()->next()->vertex()->position*/	mesh->point(mesh->source(mesh->next(mesh->next(h))));

		const Vector_3& e01 = p1 - p0;
		const Vector_3& e12 = p2 - p1;
		const Vector_3& e20 = p0 - p2;

		const Vector_3& u = (e01 - e20) / 2;
		const Vector_3& v = (e12 - e01) / 2;

		const Vector_3& a = /*p0 + u / 5*/ Vector_3(p0.x() + (u.x() / 5.0), p0.y() + (u.y() / 5.0), p0.z() + (u.z() / 5.0));
		const Vector_3& b = /*p1 + v / 5*/ Vector_3(p1.x() + (v.x() / 5.0), p1.y() + (v.y() / 5.0), p1.z() + (v.z() / 5.0));

		const Vector_3& s = (b - a) / 5;
		const Vector_3& t = CGAL::cross_product(mesh->faceNormals.at(mesh->face(h)), s);
		double theta = M_PI * 5 / 6;
		const Vector_3& c = b + cos(theta) * s + sin(theta) * t;

		glBegin(GL_LINE_STRIP);
		glVertex3dv(&a.x());
		glVertex3dv(&b.x());
		glVertex3dv(&c.x());
		glEnd();
	}
	// END TEMP
}
