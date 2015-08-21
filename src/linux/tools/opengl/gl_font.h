#ifndef GL_FONT_H
#define GL_FONT_H

#include <stdio.h>
#include "linux/tools/opengl/text_shader.h"
#include <algorithm>
#include <ft2build.h>
#include FT_FREETYPE_H

struct CharInfo
{
	float ax;	// advance.x
	float ay;	// advance.y

	float bw;	// bitmap.width;
	float bh;	// bitmap.height;

	float bl;	// bitmap_left;
	float bt;	// bitmap_top;

	float tx;	// x offset of glyph in texture coordinates
	float ty;	// y offset of glyph in texture coordinates
};

class GlFont
{
	public:
		GlFont();
		~GlFont();

		int init(const char* fontName, int fontSize);

		void glPrintf(float x, float y, float x_ratio, float y_ratio, const char* s, ...);
		void glPrintf_xright2_ycenter(float x, float y, float x_ratio, float y_ratio, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
		void glPrintf_xright2_yhigh(float x, float y, float x_ratio, float y_ratio, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
		void glPrintf_xcenter_yhigh2(float x, float y, float x_ratio, float y_ratio, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
		void glPrintf_xcenter_ycenter(float x, float y, float x_ratio, float y_ratio, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
		void glprint(float x, float y, float x_ratio, float y_ratio, const char* text, int size);

		inline void setColor(float r, float g, float b, float a);
		int height;
		int digitHeight;
		int width;
		TextShader m_textShader;

	protected:
		FT_Library m_ft;
		FT_Face m_face;
		GLuint m_vbo;
		GLuint m_vao;

		GLuint m_tex;
		int m_texWidth;
		int m_texHeight;

		CharInfo m_charInfo[128];
};

inline void GlFont::setColor(float r, float g, float b, float a)
{
	m_textShader.setColor(r,g,b,a);
}

#endif
