#ifndef SHADER_H
#define SHADER_H

#include <epoxy/gl.h>
#include <epoxy/glx.h>

class Shader
{
	public:
		Shader();

		int init(const char* vertexShaderSrc, const char* fragmentShaderSrc);
		void use();

		GLint getAttribLocation(const char* name);
		GLint getUniformLocation(const char* name);

	protected:
		void printGlLog(GLuint object);
		GLuint createShader(int shader_type, const char *source);
		GLuint m_program;
};

#endif

