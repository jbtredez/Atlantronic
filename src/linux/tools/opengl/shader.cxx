#include "shader.h"
#include <stdio.h>
#include <stdlib.h>

Shader::Shader()
{
	m_program = 0;
}

int Shader::init(const char* vertexShaderSrc, const char* fragmentShaderSrc)
{
	GLuint vertexShader = createShader(GL_VERTEX_SHADER, vertexShaderSrc);
	if( ! vertexShader )
	{
		return -1;
	}

	GLuint fragmentShader = createShader(GL_FRAGMENT_SHADER, fragmentShaderSrc);
	if( ! fragmentShader )
	{
		return -1;
	}

	m_program = glCreateProgram();
	glAttachShader(m_program, vertexShader);
	glAttachShader(m_program, fragmentShader);
	glLinkProgram(m_program);

	int status;
	glGetProgramiv(m_program, GL_LINK_STATUS, &status);
	if (status == GL_FALSE )
	{
		printGlLog(m_program);
		glDeleteProgram (m_program);
		m_program = 0;
		return -1;
	}

	return 0;
}

void Shader::use()
{
	glUseProgram(m_program);
}

void Shader::printGlLog(GLuint object)
{
	GLint log_length = 0;
	if( glIsShader(object) )
	{
		glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
	}
	else if( glIsProgram(object) )
	{
		glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
	}
	else
	{
		fprintf(stderr, "%s:%d: Not a shader or a m_program\n", __FUNCTION__, __LINE__);
		return;
	}

	char* log = (char*)malloc(log_length);

	if( glIsShader(object) )
	{
		glGetShaderInfoLog(object, log_length, NULL, log);
	}
	else if( glIsProgram(object) )
	{
		glGetProgramInfoLog(object, log_length, NULL, log);
	}

	fprintf(stderr, "%s", log);
	free(log);
}

GLuint Shader::createShader(int shader_type, const char *source)
{
	GLuint shader = glCreateShader(shader_type);
	if( shader == 0 )
	{
		fprintf(stderr, "glCreateShader(%d) error %d", shader_type, glGetError());
		return 0;
	}

	glShaderSource(shader, 1, &source, NULL);
	glCompileShader(shader);

	int status;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
	if(status == GL_FALSE)
	{
		printGlLog(shader);
		glDeleteShader(shader);
		shader = 0;
	}

	return shader;
}

GLint Shader::getAttribLocation(const char* name)
{
	GLint attribute = glGetAttribLocation(m_program, name);
	if (attribute == -1)
	{
		fprintf(stderr, "Could not bind attribute %s\n", name);
	}

	return attribute;
}

GLint Shader::getUniformLocation(const char* name)
{
	GLint uniform = glGetUniformLocation(m_program, name);
	if (uniform == -1)
	{
		fprintf(stderr, "Could not bind uniform %s\n", name);
	}
	return uniform;
}
