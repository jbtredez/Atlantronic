#include "text_shader.h"
#include <stdio.h>

static const char* vertexShaderSrc =
"attribute vec4 coord;                                         \n"
"varying vec2 texpos;                                          \n"
"uniform mat4 projection;                                      \n"
"void main()                                                   \n"
"{                                                             \n"
"	gl_Position = projection * vec4(coord.xy, 0, 1);           \n"
"	texpos = coord.zw;                                         \n"
"}                                                             \n";

static const char* fragmentShaderSrc =
"varying vec2 texpos;                                                \n"
"uniform sampler2D tex;                                              \n"
"uniform vec4 color;                                                 \n"
"void main()                                                         \n"
"{                                                                   \n"
"	gl_FragColor = vec4(1, 1, 1, texture2D(tex, texpos).r) * color;  \n"
"}                                                                   \n";

TextShader::TextShader() :
	Shader()
{

}

int TextShader::init()
{
	int res = Shader::init(vertexShaderSrc, fragmentShaderSrc);
	if( res )
	{
		return res;
	}

	m_attribute_coord = getAttribLocation("coord");
	m_uniform_tex = getUniformLocation("tex");
	m_uniform_color = getUniformLocation("color");
	m_uniform_projection = getUniformLocation("projection");

	if( m_attribute_coord == -1 || m_uniform_tex == -1 || m_uniform_color == -1)
	{
		return -1;
	}

	return 0;
}

void TextShader::setProjection(glm::mat4 projection)
{
	m_projection = projection;
	glUniformMatrix4fv(m_uniform_projection, 1, GL_FALSE, glm::value_ptr(m_projection));
}
