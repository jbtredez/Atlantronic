#include "gl_font.h"
#include <stdarg.h>

// Maximum texture width
#define MAXWIDTH 1024

struct point
{
	GLfloat x;
	GLfloat y;
	GLfloat s;
	GLfloat t;
};

GlFont::GlFont()
{
	m_tex = 0;
}

GlFont::~GlFont()
{
	if( m_tex )
	{
		glDeleteTextures(1, &m_tex);
	}
}

int GlFont::init(const char* fontName, int fontSize)
{
	int res = FT_Init_FreeType(&m_ft);
	if( res )
	{
		fprintf(stderr, "Could not init freetype library\n");
		return -1;
	}

	res = FT_New_Face(m_ft, fontName, 0, &m_face);
	if( res )
	{
		fprintf(stderr, "Could not open font %s\n", fontName);
		return -1;
	}

	res = m_textShader.init();
	if( res )
	{
		return -1;
	}

	FT_Set_Pixel_Sizes(m_face, 0, fontSize);
	FT_GlyphSlot g = m_face->glyph;

	int roww = 0;
	int rowh = 0;
	m_texWidth = 0;
	m_texHeight = 0;

	memset(m_charInfo, 0, sizeof m_charInfo);

	/* Find minimum size for a texture holding all visible ASCII characters */
	for (int i = 32; i < 128; i++)
	{
		if (FT_Load_Char(m_face, i, FT_LOAD_RENDER))
		{
			fprintf(stderr, "Loading character %c failed!\n", i);
			continue;
		}
		if (roww + g->bitmap.width + 1 >= MAXWIDTH)
		{
			m_texWidth = std::max(m_texWidth, roww);
			m_texHeight += rowh;
			roww = 0;
			rowh = 0;
		}
		roww += g->bitmap.width + 1;
		rowh = std::max(rowh, (int)g->bitmap.rows);
	}

	m_texWidth = std::max(m_texWidth, roww);
	m_texHeight += rowh;

	/* Create a texture that will be used to hold all ASCII glyphs */
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &m_tex);
	glBindTexture(GL_TEXTURE_2D, m_tex);
	glUniform1i(m_textShader.m_uniform_tex, 0);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, m_texWidth, m_texHeight, 0, GL_RED, GL_UNSIGNED_BYTE, 0);

	/* We require 1 byte alignment when uploading texture data */
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	/* Clamping to edges is important to prevent artifacts when scaling */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	/* Linear filtering usually looks best for text */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	/* Paste all glyph bitmaps into the texture, remembering the offset */
	int ox = 0;
	int oy = 0;

	rowh = 0;

	for (int i = 32; i < 128; i++) {
		if (FT_Load_Char(m_face, i, FT_LOAD_RENDER)) {
			fprintf(stderr, "Loading character %c failed!\n", i);
			continue;
		}

		if (ox + g->bitmap.width + 1 >= MAXWIDTH) {
			oy += rowh;
			rowh = 0;
			ox = 0;
		}

		glTexSubImage2D(GL_TEXTURE_2D, 0, ox, oy, g->bitmap.width, g->bitmap.rows, GL_RED, GL_UNSIGNED_BYTE, g->bitmap.buffer);
		m_charInfo[i].ax = g->advance.x >> 6;
		m_charInfo[i].ay = g->advance.y >> 6;

		m_charInfo[i].bw = g->bitmap.width;
		m_charInfo[i].bh = g->bitmap.rows;

		m_charInfo[i].bl = g->bitmap_left;
		m_charInfo[i].bt = g->bitmap_top;

		m_charInfo[i].tx = ox / (float)m_texWidth;
		m_charInfo[i].ty = oy / (float)m_texHeight;

		rowh = std::max(rowh, (int)g->bitmap.rows);
		ox += g->bitmap.width + 1;
	}

	height = m_charInfo['0'].bh;
	width = m_charInfo['0'].bw;
	digitHeight = m_charInfo['0'].bh;

	//fprintf(stderr, "Generated a %d x %d (%d kb) texture atlas\n", m_texWidth, m_texHeight, m_texWidth * m_texHeight / 1024);

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);

	glGenBuffers(1, &m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	return 0;
}

void GlFont::glprint(float x, float y, float x_ratio, float y_ratio, const char* text, int size)
{
	if( size == 0)
	{
		return;
	}
	const uint8_t *p;

	glBindVertexArray(m_vao);

	/* Use the texture containing the atlas */
	glBindTexture(GL_TEXTURE_2D, m_tex);
	glUniform1i(m_textShader.m_uniform_tex, 0);

	/* Set up the VBO for our vertex data */
	glEnableVertexAttribArray(m_textShader.m_attribute_coord);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glVertexAttribPointer(m_textShader.m_attribute_coord, 4, GL_FLOAT, GL_FALSE, 0, 0);

	point coords[6 * size];
	int c = 0;

	/* Loop through all characters */
	for (p = (const uint8_t *)text; *p; p++)
	{
		/* Calculate the vertex and texture coordinates */
		float x2 = x + m_charInfo[*p].bl*x_ratio;
		float y2 = -y - m_charInfo[*p].bt*y_ratio;
		float w = m_charInfo[*p].bw*x_ratio;
		float h = m_charInfo[*p].bh*y_ratio;

		/* Advance the cursor to the start of the next character */
		x += m_charInfo[*p].ax*x_ratio;
		y += m_charInfo[*p].ay*y_ratio;

		/* Skip glyphs that have no pixels */
		if (!w || !h)
			continue;

		coords[c++] = (point) {	x2, -y2, m_charInfo[*p].tx, m_charInfo[*p].ty};
		coords[c++] = (point) {	x2 + w, -y2, m_charInfo[*p].tx + m_charInfo[*p].bw / m_texWidth, m_charInfo[*p].ty};
		coords[c++] = (point) {	x2, -y2 - h, m_charInfo[*p].tx, m_charInfo[*p].ty + m_charInfo[*p].bh / m_texHeight};
		coords[c++] = (point) {	x2 + w, -y2, m_charInfo[*p].tx + m_charInfo[*p].bw / m_texWidth, m_charInfo[*p].ty};
		coords[c++] = (point) {	x2, -y2 - h, m_charInfo[*p].tx, m_charInfo[*p].ty + m_charInfo[*p].bh / m_texHeight};
		coords[c++] = (point) {	x2 + w, -y2 - h, m_charInfo[*p].tx + m_charInfo[*p].bw / m_texWidth, m_charInfo[*p].ty + m_charInfo[*p].bh / m_texHeight};
	}

	/* Draw all the character on the screen in one go */
	glBufferData(GL_ARRAY_BUFFER, sizeof(coords), coords, GL_DYNAMIC_DRAW);

	glDrawArrays(GL_TRIANGLES, 0, c);

	glDisableVertexAttribArray(m_textShader.m_attribute_coord);
	glBindVertexArray(0);
}

void GlFont::glPrintf(float x, float y, float x_ratio, float y_ratio, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x, y, x_ratio, y_ratio, buffer, size);
}

void GlFont::glPrintf_xcenter_ycenter(float x, float y, float x_ratio,float y_ratio, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x- x_ratio * size/2.0f * width, y - digitHeight / 2.0f * y_ratio, x_ratio, y_ratio, buffer, size);
}

void GlFont::glPrintf_xright2_ycenter(float x, float y, float x_ratio,float y_ratio, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * (size+2) * width, y - digitHeight / 2.0f * y_ratio, x_ratio, y_ratio, buffer, size);
}

void GlFont::glPrintf_xright2_yhigh(float x, float y, float x_ratio,float y_ratio, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * (size+2) * width, y - digitHeight * y_ratio, x_ratio, y_ratio, buffer, size);
}

void GlFont::glPrintf_xcenter_yhigh2(float x, float y, float x_ratio,float y_ratio, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * size/2.0f * width, y - 2*digitHeight * y_ratio, x_ratio, y_ratio, buffer, size);
}
