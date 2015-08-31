#ifndef OBJECT_3D_H
#define OBJECT_3D_H

#include <epoxy/gl.h>
#include <epoxy/glx.h>
#include <stdio.h>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>

#include "main_shader.h"

class Object3dBasic
{
	public:
		Object3dBasic();
		~Object3dBasic();

		void init(aiMesh *mesh, MainShader* shader);
		void init(float* vertices, int elementSize, int elementCount, MainShader* shader, bool dynamic = false);
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

class Object3d
{
	public:
		Object3d();
		~Object3d();

		bool init(const char* filename, MainShader* shader);
		void draw();

		aiVector3D sceneMin;
		aiVector3D sceneMax;
		aiVector3D sceneCenter;
		bool selected;

	protected:
		void getBoundingBoxForNode(const aiNode* nd, aiMatrix4x4* trafo);
		void getBoundingBox();
		void render(const struct aiNode* nd);
		void color4ToFloat4(const aiColor4D *c, float f[4]);
		void setFloat4(float f[4], float a, float b, float c, float d);
		void applyMaterial(const struct aiMaterial *mtl);

		const struct aiScene* m_scene;
		MainShader* m_shader;
		Object3dBasic* m_meshEntries;
};

#endif
