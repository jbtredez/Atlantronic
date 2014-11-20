#ifndef GL_FONT_H
#define GL_FONT_H

#include <stdio.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>

class GlFont
{
	public:
		bool init(const char* fontName);

		void glPrintf(float x, float y, const char* s, ...);
		void glPrintf_xright2_ycenter(float x, float y, float x_ratio, float y_ratio, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
		void glPrintf_xright2_yhigh(float x, float y, float x_ratio, float y_ratio, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
		void glPrintf_xcenter_yhigh2(float x, float y, float x_ratio, float y_ratio, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
		void glPrintf_xcenter_ycenter(float x, float y, float x_ratio, float y_ratio, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
		void glprint(float x, float y, const char* buffer, int size);

		int height;
		int digitHeight;
		int width;

	protected:
		GLuint fontBase;
		XFontStruct* fontInfo;
};

#endif
