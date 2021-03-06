#ifndef MAIN_SHADER_H
#define MAIN_SHADER_H

#include "shader.h"
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class MainShader : public Shader
{
	public:
		MainShader();

		int init();

		void setSprite(float sprite);
		void setProjection(glm::mat4 projection);
		void setModelView(glm::mat4 mvp);
		inline glm::mat4 getModelView();
		inline glm::mat4 getProjection();

		void setColor(float r, float g, float b);
		void setColor3f(float* rgb);
		void setColorAmbiant(float r, float g, float b);
		void setColorDiffuse(float r, float g, float b);
		void setColorSpecular(float r, float g, float b);

		GLint m_attribute_coord3d;
		GLint m_attribute_normal;

	protected:
		GLint m_uniform_projection;
		GLint m_uniform_modelView;
		GLint m_uniform_normalMatrix;
		GLint m_uniform_color_ambient;
		GLint m_uniform_color_diffuse;
		GLint m_uniform_color_specular;
		GLint m_uniform_sprite;

		glm::mat4 m_projection;
		glm::mat4 m_modelView;
};

inline glm::mat4 MainShader::getProjection()
{
	return m_projection;
}

inline glm::mat4 MainShader::getModelView()
{
	return m_modelView;
}

#endif

