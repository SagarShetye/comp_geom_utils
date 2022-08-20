#include "shader.h"

Shader::Shader() {};

Shader::Shader(const std::string &vtxShaderFile, const std::string &fragShaderFile) {
	// Read the vertex shader source code
	readShaderSource(vtxShaderFile, vtxShaderData);

	// Read the fragment shader souce code
	readShaderSource(fragShaderFile, fragShaderData);
}

// Read the GLSL source code from the file
void Shader::readShaderSource(const std::string &shaderSource, std::string &shaderData) {
	std::ifstream t(shaderSource);
	std::stringstream buffer;
	buffer << t.rdbuf();
	shaderData = buffer.str();
}

bool Shader::compileShader(const std::string &shaderData, unsigned int shaderID) {
	glCompileShader(shaderID);
	// Check if the shader compiled successfully
	int  success;
	char infoLog[512];
	glGetShaderiv(shaderID, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(shaderID, 512, NULL, infoLog);
		std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
		return false;
	}
	// Shader compiled successfully
	return true;
}

bool Shader::compileVertexShader() {
	const char *vtxShaderSourcePtr = vtxShaderData.data();
	vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShaderID, 1, &vtxShaderSourcePtr, NULL);

	// Compile the vertex shader
	return compileShader(vtxShaderData, vertexShaderID);
}

bool Shader::compileFragmentShader() {
	const char *fragShaderSourcePtr = fragShaderData.data();
	fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShaderID, 1, &fragShaderSourcePtr, NULL);

	// Compile the fragment shader
	return compileShader(fragShaderData, fragmentShaderID);
}

bool Shader::compile() {
	bool success = true;
	if (!compileVertexShader())
		success = false;
	if (!compileFragmentShader())
		success = false;
	return success;
}

bool Shader::linkShader() {
	programID = glCreateProgram();
	glAttachShader(programID, vertexShaderID);
	glAttachShader(programID, fragmentShaderID);
	glLinkProgram(programID);

	// Check if the linking happened successfully -
	int  success;
	char infoLog[512];
	glGetProgramiv(programID, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(programID, 512, NULL, infoLog);
		// Output the error message
		std::cerr << "SHADER LINKING FAILED\n" << infoLog << std::endl;
	}

	// Delete the shaders since they are now linked
	glDeleteShader(vertexShaderID);
	glDeleteShader(fragmentShaderID);

	return success;
}
