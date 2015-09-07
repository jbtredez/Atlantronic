#include "main_shader.h"
#include <stdio.h>

static const char* vertexShaderSrc =
"#version 120                                                  \n"
"attribute vec3 coord3d;                                       \n"
"attribute vec3 normal;                                        \n"
"uniform mat4 projection;                                      \n"
"uniform mat4 modelView;                                       \n"
"uniform mat3 normalMatrix;                                    \n"
"uniform float sprite;                                         \n"
"varying vec3 f_coord3d;                                       \n"
"varying vec3 f_normal;                                        \n"
"void main()                                                   \n"
"{                                                             \n"
"	f_coord3d = (modelView * vec4(coord3d, 1.0)).xyz;          \n"
"	f_normal = normalMatrix * normal;                          \n"
"	gl_Position = projection * modelView * vec4(coord3d, 1.0); \n"
"	gl_PointSize = max(1.0, sprite);                           \n"
"}                                                             \n";

static const char* fragmentShaderSrc =
"#version 120                                                \n"
"uniform vec3 color_ambient;                                 \n"
"uniform vec3 color_diffuse;                                 \n"
"uniform vec3 color_specular;                                \n"
"uniform sampler2D tex;                                      \n"
"uniform float sprite;                                       \n"
"varying vec3 f_coord3d;                                     \n"
"varying vec3 f_normal;                                      \n"
"void main()                                                     \n"
"{                                                               \n"
"	if( sprite <= 1.0 )                                          \n"
"	{                                                            \n"
"		vec3 lightPos = vec3(0, 2000, 0);                        \n"
"		vec3 viewPos = vec3(0, -3000, 2000);                     \n"
// Ambient
"		vec3 ambient = color_ambient;                            \n"
// Diffuse
"		vec3 lightDir = normalize(lightPos - f_coord3d);         \n"
"		vec3 n = normalize(f_normal);                            \n"
"		float diff = max(dot(lightDir, n), 0.0);                 \n"
"		vec3 diffuse = diff * color_diffuse;                     \n"
// Specular
"		vec3 viewDir = normalize(viewPos - f_coord3d);           \n"
"		vec3 reflectDir = reflect(-lightDir, n);                 \n"
"		float spec = 0.0;                                        \n"
"		vec3 halfwayDir = normalize(lightDir + viewDir);         \n"
"		spec = pow(max(dot(f_normal, halfwayDir), 0.0), 32.0);   \n"
"		vec3 specular = color_specular * spec;                   \n"
"		gl_FragColor = vec4(ambient + diffuse + specular, 1.0);  \n"
"	}                                                            \n"
"	else                                                         \n"
"	{                                                            \n"
"		gl_FragColor = texture2D(tex, gl_PointCoord) * vec4(color_ambient,1.0);\n"
"	}                                                            \n"
"}                                                               \n";

MainShader::MainShader() :
	Shader()
{

}

int MainShader::init()
{
	int res = Shader::init(vertexShaderSrc, fragmentShaderSrc);
	if( res )
	{
		return res;
	}

	m_attribute_coord3d = getAttribLocation("coord3d");
	m_attribute_normal = getAttribLocation("normal");
	m_uniform_color_ambient = getUniformLocation("color_ambient");
	m_uniform_color_diffuse = getUniformLocation("color_diffuse");
	m_uniform_color_specular = getUniformLocation("color_specular");
	m_uniform_projection = getUniformLocation("projection");
	m_uniform_modelView = getUniformLocation("modelView");
	m_uniform_normalMatrix = getUniformLocation("normalMatrix");
	m_uniform_sprite = getUniformLocation("sprite");

	if( m_attribute_coord3d == -1 || m_attribute_normal == -1 || m_uniform_color_ambient == -1 ||
		m_uniform_color_diffuse == -1 || m_uniform_color_specular == -1 || m_uniform_projection == -1 ||
		m_uniform_modelView == -1 || m_uniform_normalMatrix == -1 || m_uniform_sprite == -1)
	{
		return -1;
	}

	return 0;
}

void MainShader::setSprite(float sprite)
{
	glUniform1f(m_uniform_sprite, sprite);
}

void MainShader::setProjection(glm::mat4 projection)
{
	m_projection = projection;
	glUniformMatrix4fv(m_uniform_projection, 1, GL_FALSE, glm::value_ptr(m_projection));
}

void MainShader::setModelView(glm::mat4 modelView)
{
	m_modelView = modelView;
	glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(m_modelView)));
	glUniformMatrix4fv(m_uniform_modelView, 1, GL_FALSE, glm::value_ptr( m_modelView));
	glUniformMatrix3fv(m_uniform_normalMatrix, 1, GL_FALSE, glm::value_ptr(normalMatrix));
}

void MainShader::setColorAmbiant(float r, float g, float b)
{
	glUniform3f(m_uniform_color_ambient, r, g, b);
}

void MainShader::setColorDiffuse(float r, float g, float b)
{
	glUniform3f(m_uniform_color_diffuse, r, g, b);
}

void MainShader::setColorSpecular(float r, float g, float b)
{
	glUniform3f(m_uniform_color_specular, r, g, b);
}

void MainShader::setColor(float r, float g, float b)
{
	setColorAmbiant(r, g, b);
	setColorDiffuse(0, 0, 0);
	setColorSpecular(0, 0, 0);
}

void MainShader::setColor3f(float* rgb)
{
	setColor(rgb[0], rgb[1], rgb[2]);
}
