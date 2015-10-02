#ifndef OBJECT_3D_H
#define OBJECT_3D_H

#include "Object3dBasic.h"
#include <stdio.h>

#include "main_shader.h"

class GlObject
{
	public:
		GlObject();
		~GlObject();

		bool init(const char* filename, MainShader* shader);

		void draw();

		aiVector3D sceneMin;
		aiVector3D sceneMax;
		aiVector3D sceneCenter;
		bool selected;
		MainShader* m_shader;

	protected:
		void getBoundingBoxForNode(const aiNode* nd, aiMatrix4x4* trafo);
		void getBoundingBox();
		void render(const struct aiNode* nd);
		void color4ToFloat4(const aiColor4D *c, float f[4]);
		void setFloat4(float f[4], float a, float b, float c, float d);
		void applyMaterial(const struct aiMaterial *mtl);

		const struct aiScene* m_scene;
		GlObjectBasic* m_meshEntries;
};

class Object3d
{
	public:
		bool init(GlObject* obj, int selectionId);

		inline void setPosition(float x, float y, float z, float theta);
		void draw();

		bool selected;
		aiVector3D position;
		float theta;
		unsigned int selectionId;

	protected:
		GlObject* m_obj;
};

inline void Object3d::setPosition(float x, float y, float z, float Theta = 0)
{
	position = aiVector3D(x, y, z);
	theta = Theta;
}
#endif
