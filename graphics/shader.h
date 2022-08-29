//#pragma once
#ifndef __SHADER__
#define __SHADER__

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <GL/glew.h>

// Class for loading/compiling/linking shaders
class Shader {
public:
	// The GLSL source code for the vertex and fragment shaders
	std::string vtxShaderData, fragShaderData;

	// The shader program ID. This would be used for OpenGL routines
	unsigned int programID;

	// program IDs for the vertex and fragment shaders, these would be linked in the last step
	unsigned int vertexShaderID, fragmentShaderID;

	Shader() = delete;
	Shader(const std::string &vtxShaderFile, const std::string &fragShaderFile);

	// Compile the shaders
	bool compile();

	// Link the shader programs (vertex and fragment)
	bool linkShader();

private:

	void readShaderSource(const std::string &shaderSource, std::string &shaderData);

	bool compileShader(const std::string &shaderData, unsigned int shaderID);

	// Compile shader source code
	bool compileVertexShader();
	bool compileFragmentShader();
};

#endif