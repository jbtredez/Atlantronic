#include "gl_font.h"
#include <stdarg.h>

bool GlFont::init(const char* fontName)
{
	fontBase = glGenLists(256);

	if( !glIsList(fontBase) )
	{
		fprintf(stderr, "my_init(): Out of display lists. - Exiting.\n");
		return false;
	}

	int first;
	int last;

	// Need an X Display before calling any Xlib routines
	Display* display = XOpenDisplay(0);
	if (display == 0)
	{
		fprintf(stderr, "XOpenDisplay() failed\n");
		return false;
	}

	// Load the font
	fontInfo = XLoadQueryFont(display, fontName);
	if (!fontInfo)
	{
		fprintf(stderr, "XLoadQueryFont() failed\n");
		return false;
	}

	// Tell GLX which font & glyphs to use
	first = fontInfo->min_char_or_byte2;
	last  = fontInfo->max_char_or_byte2;
	glXUseXFont(fontInfo->fid, first, last-first+1, fontBase+first);

	height = fontInfo->ascent;
	digitHeight = fontInfo->per_char['0'].ascent; // on prend la hauteur de 0
	width = fontInfo->max_bounds.width;
	XCloseDisplay(display);

	return true;
}

void GlFont::glprint(float x, float y, const char* buffer, int size)
{
	if( size != 0)
	{
		glRasterPos2f(x, y);
		glPushAttrib(GL_LIST_BIT);
		glListBase(fontBase);
		glCallLists(size, GL_UNSIGNED_BYTE, (GLubyte *)buffer);
		glPopAttrib();
	}
}

void GlFont::glPrintf(float x, float y, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x, y, buffer, size);
}

void GlFont::glPrintf_xcenter_ycenter(float x, float y, float x_ratio,float y_ratio, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x- x_ratio * size/2.0f * width, y - digitHeight / 2.0f * y_ratio, buffer, size);
}

void GlFont::glPrintf_xright2_ycenter(float x, float y, float x_ratio,float y_ratio, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * (size+2) * width, y - digitHeight / 2.0f * y_ratio, buffer, size);
}

void GlFont::glPrintf_xright2_yhigh(float x, float y, float x_ratio,float y_ratio, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * (size+2) * width, y - digitHeight * y_ratio, buffer, size);
}

void GlFont::glPrintf_xcenter_yhigh2(float x, float y, float x_ratio,float y_ratio, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * size/2.0f * width, y - 2*digitHeight * y_ratio, buffer, size);
}
