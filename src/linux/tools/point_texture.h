#ifndef POINT_TEXTURE
#define POINT_TEXTURE

typedef struct
{
	unsigned int 	 width;
	unsigned int 	 height;
	unsigned int 	 bytes_per_pixel; /* 3:RGB, 4:RGBA */
	unsigned char	 pixel_data[15 * 15 * 4 + 1];
} PointTexture;

extern PointTexture pointTexture;

#endif
