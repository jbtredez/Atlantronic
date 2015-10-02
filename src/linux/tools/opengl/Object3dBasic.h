#ifndef OBJECT_3D_BASIC_H
#define OBJECT_3D_BASIC_H

#include <epoxy/gl.h>
#include <epoxy/glx.h>
#include <stdio.h>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>

#include "main_shader.h"

class GlObjectBasic
{
	public:
		GlObjectBasic();
		~GlObjectBasic();

		bool init(aiMesh *mesh, MainShader* shader);
		bool init(float* vertices, int elementSize, int elementCount, MainShader* shader, bool dynamic = false);
		void update(float* vertices, int nbElement);

		void render(GLenum mode);

	protected:
		enum
		{
			VERTEX_BUFFER,
			TEXCOORD_BUFFER,
			NORMAL_BUFFER,
			INDEX_BUFFER,
		};

		GLuint m_vao;
		GLuint m_vbo[4];
		unsigned int m_elementCount;
		unsigned int m_elementSize;
};

#endif
