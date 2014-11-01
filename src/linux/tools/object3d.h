#ifndef OBJECT_3D_H
#define OBJECT_3D_H

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <stdio.h>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

class Object3d
{
	public:
		Object3d();

		bool init(const char* filename);
		void draw();

		aiVector3D sceneMin;
		aiVector3D sceneMax;
		aiVector3D sceneCenter;

	protected:
		void getBoundingBoxForNode(const aiNode* nd, aiMatrix4x4* trafo);
		void getBoundingBox();
		void render(const struct aiNode* nd);
		void color4ToFloat4(const aiColor4D *c, float f[4]);
		void setFloat4(float f[4], float a, float b, float c, float d);
		void applyMaterial(const struct aiMaterial *mtl);

		const struct aiScene* scene;
		int glListId;
};

#endif
