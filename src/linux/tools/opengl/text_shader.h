#ifndef TEXT_SHADER_H
#define TEXT_SHADER_H

#include "shader.h"
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class TextShader : public Shader
{
	public:
		TextShader();

		int init();
		inline void setColor(float r, float g, float b, float a);
		void setProjection(glm::mat4 projection);

		GLint m_attribute_coord;
		GLint m_uniform_tex;

	protected:
		GLint m_uniform_color;
		GLint m_uniform_projection;

		glm::mat4 m_projection;
};

inline void TextShader::setColor(float r, float g, float b, float a)
{
	glUniform4f(m_uniform_color, r, g, b, a);
}

#endif

