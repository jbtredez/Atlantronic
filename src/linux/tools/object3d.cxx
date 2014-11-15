#include "object3d.h"

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

Object3d::Object3d()
{
	scene = NULL;
	glListId = 0;
	selected = false;
}

bool Object3d::init(const char* filename)
{
	bool res = false;

	scene = aiImportFile(filename, aiProcessPreset_TargetRealtime_MaxQuality);

	if( ! scene )
	{
		fprintf(stderr, "aiImportFile error\n");
		goto done;
	}

	getBoundingBox();
	sceneCenter.x = (sceneMin.x + sceneMax.x) / 2.0f;
	sceneCenter.y = (sceneMin.y + sceneMax.y) / 2.0f;
	sceneCenter.z = (sceneMin.z + sceneMax.z) / 2.0f;

	res = true;

done:
	return res;
}

void Object3d::getBoundingBoxForNode(const aiNode* nd, aiMatrix4x4* trafo)
{
	aiMatrix4x4 prev;
	unsigned int n = 0, t;
	prev = *trafo;
	aiMultiplyMatrix4(trafo,&nd->mTransformation);
	for (; n < nd->mNumMeshes; ++n)
	{
		const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		for (t = 0; t < mesh->mNumVertices; ++t)
		{
			aiVector3D tmp = mesh->mVertices[t];
			aiTransformVecByMatrix4(&tmp,trafo);
			sceneMin.x = aisgl_min(sceneMin.x,tmp.x);
			sceneMin.y = aisgl_min(sceneMin.y,tmp.y);
			sceneMin.z = aisgl_min(sceneMin.z,tmp.z);
			sceneMax.x = aisgl_max(sceneMax.x,tmp.x);
			sceneMax.y = aisgl_max(sceneMax.y,tmp.y);
			sceneMax.z = aisgl_max(sceneMax.z,tmp.z);
		}
	}
	for (n = 0; n < nd->mNumChildren; ++n)
	{
		getBoundingBoxForNode(nd->mChildren[n],trafo);
	}
	*trafo = prev;
}

void Object3d::getBoundingBox()
{
	aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);
	sceneMin.x = sceneMin.y = sceneMin.z = 1e10f;
	sceneMax.x = sceneMax.y = sceneMax.z = -1e10f;
	getBoundingBoxForNode(scene->mRootNode, &trafo);
}

void Object3d::color4ToFloat4(const aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

void Object3d::setFloat4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

void Object3d::applyMaterial(const struct aiMaterial *mtl)
{
	float c[4];
	GLenum fill_mode;
	int ret1, ret2;
	aiColor4D diffuse;
	aiColor4D specular;
	aiColor4D ambient;
	aiColor4D emission;
	float shininess, strength;
	int two_sided;
	int wireframe;
	unsigned int max;

	setFloat4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	if(aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse) == AI_SUCCESS)
	{
		color4ToFloat4(&diffuse, c);
	}

	glMaterialfv(GL_FRONT, GL_DIFFUSE, c);

	if( ! selected )
	{
		setFloat4(c, 0.0f, 0.0f, 0.0f, 1.0f);
		if(aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular) == AI_SUCCESS)
		{
			color4ToFloat4(&specular, c);
		}
	}
	else
	{
		setFloat4(c, 0.0f, 0.2f, 0.8f, 1.0f);
	}
	glMaterialfv(GL_FRONT, GL_SPECULAR, c);

	setFloat4(c, 0.2f, 0.2f, 0.2f, 1.0f);
	if(aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient) == AI_SUCCESS)
	{
		color4ToFloat4(&ambient, c);
	}
	glMaterialfv(GL_FRONT, GL_AMBIENT, c);

	setFloat4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission) == AI_SUCCESS)
	{
		color4ToFloat4(&emission, c);
	}
	glMaterialfv(GL_FRONT, GL_EMISSION, c);

	max = 1;
	ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
	if(ret1 == AI_SUCCESS)
	{
		max = 1;
		ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
		if(ret2 == AI_SUCCESS)
		{
			glMaterialf(GL_FRONT, GL_SHININESS, shininess * strength);
		}
		else
		{
			glMaterialf(GL_FRONT, GL_SHININESS, shininess);
		}
	}
	else
	{
		glMaterialf(GL_FRONT, GL_SHININESS, 0.0f);
		setFloat4(c, 0.0f, 0.0f, 0.0f, 0.0f);
		glMaterialfv(GL_FRONT, GL_SPECULAR, c);
	}
	max = 1;
	if( aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max) == AI_SUCCESS)
	{
		fill_mode = wireframe ? GL_LINE : GL_FILL;
	}
	else
	{
		fill_mode = GL_FILL;
	}
	glPolygonMode(GL_FRONT, fill_mode);
	max = 1;
	if( (aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max) == AI_SUCCESS) && two_sided)
	{
		glDisable(GL_CULL_FACE);
	}
	else
	{
		glEnable(GL_CULL_FACE);
	}
}

void Object3d::render(const struct aiNode* nd)
{
	unsigned int i;
	unsigned int n = 0, t;
	aiMatrix4x4 m = nd->mTransformation;
	/* update transform */
	aiTransposeMatrix4(&m);
	glPushMatrix();
	glMultMatrixf((float*)&m);
	/* draw all meshes assigned to this node */
	for(; n < nd->mNumMeshes; ++n)
	{
		const struct aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		applyMaterial(scene->mMaterials[mesh->mMaterialIndex]);
		if(mesh->mNormals)
		{
			glEnable(GL_LIGHTING);
		}
		else
		{
			glDisable(GL_LIGHTING);
		}
		for (t = 0; t < mesh->mNumFaces; ++t)
		{
			const struct aiFace* face = &mesh->mFaces[t];
			GLenum face_mode;
			switch(face->mNumIndices)
			{
				case 1: face_mode = GL_POINTS; break;
				case 2: face_mode = GL_LINES; break;
				case 3: face_mode = GL_TRIANGLES; break;
				default: face_mode = GL_POLYGON; break;
			}
			glBegin(face_mode);
			for(i = 0; i < face->mNumIndices; i++)
			{
				int index = face->mIndices[i];
				if(mesh->mColors[0])
				{
					glColor4fv((GLfloat*)&mesh->mColors[0][index]);
				}
				if(mesh->mNormals)
				{
					glNormal3fv(&mesh->mNormals[index].x);
				}
				glVertex3fv(&mesh->mVertices[index].x);
			}
			glEnd();
		}
	}
	// draw all children
	for (n = 0; n < nd->mNumChildren; ++n)
	{
		render(nd->mChildren[n]);
	}
	glPopMatrix();
}

void Object3d::draw()
{
	if( ! selected )
	{
		if( ! glListId )
		{
			glListId = glGenLists(1);
			if( glListId )
			{
				glNewList(glListId, GL_COMPILE_AND_EXECUTE);
				render(scene->mRootNode);
				glEndList();
			}
		}
		else
		{
			glCallList(glListId);
		}
	}
	else
	{
		render(scene->mRootNode);
	}
}
