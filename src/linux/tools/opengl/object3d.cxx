#include "object3d.h"

#define MIN(x,y) (x<y?x:y)
#define MAX(x,y) (y>x?y:x)

GlObject::GlObject()
{
	m_scene = NULL;
	selected = false;
	m_meshEntries = NULL;
}

GlObject::~GlObject()
{
	if( m_meshEntries )
	{
		delete [ ] m_meshEntries;
	}
	if( m_scene )
	{
		aiReleaseImport(m_scene);
	}
}


bool GlObject::init(const char* filename, MainShader* shader)
{
	bool res = false;

	m_shader = shader;
	m_scene = aiImportFile(filename, aiProcessPreset_TargetRealtime_MaxQuality);

	if( ! m_scene )
	{
		fprintf(stderr, "aiImportFile error %s\n", filename);
		goto done;
	}

	getBoundingBox();
	sceneCenter.x = (sceneMin.x + sceneMax.x) / 2.0f;
	sceneCenter.y = (sceneMin.y + sceneMax.y) / 2.0f;
	sceneCenter.z = (sceneMin.z + sceneMax.z) / 2.0f;

	if( m_scene->mNumMeshes < 1)
	{
		fprintf(stderr, "no meshes found\n");
		goto done;
	}

	m_meshEntries = new GlObjectBasic[m_scene->mNumMeshes]();
	for(unsigned int i = 0; i < m_scene->mNumMeshes; ++i)
	{
		m_meshEntries[i].init(m_scene->mMeshes[i], m_shader);
	}

	res = true;

done:
	return res;
}

void GlObject::getBoundingBoxForNode(const aiNode* nd, aiMatrix4x4* trafo)
{
	aiMatrix4x4 prev;
	unsigned int n = 0, t;
	prev = *trafo;
	aiMultiplyMatrix4(trafo,&nd->mTransformation);
	for (; n < nd->mNumMeshes; ++n)
	{
		const aiMesh* mesh = m_scene->mMeshes[nd->mMeshes[n]];
		for (t = 0; t < mesh->mNumVertices; ++t)
		{
			aiVector3D tmp = mesh->mVertices[t];
			aiTransformVecByMatrix4(&tmp,trafo);
			sceneMin.x = MIN(sceneMin.x,tmp.x);
			sceneMin.y = MIN(sceneMin.y,tmp.y);
			sceneMin.z = MIN(sceneMin.z,tmp.z);
			sceneMax.x = MAX(sceneMax.x,tmp.x);
			sceneMax.y = MAX(sceneMax.y,tmp.y);
			sceneMax.z = MAX(sceneMax.z,tmp.z);
		}
	}
	for (n = 0; n < nd->mNumChildren; ++n)
	{
		getBoundingBoxForNode(nd->mChildren[n],trafo);
	}
	*trafo = prev;
}

void GlObject::getBoundingBox()
{
	aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);
	sceneMin.x = sceneMin.y = sceneMin.z = 1e10f;
	sceneMax.x = sceneMax.y = sceneMax.z = -1e10f;
	getBoundingBoxForNode(m_scene->mRootNode, &trafo);
}

void GlObject::color4ToFloat4(const aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

void GlObject::setFloat4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

void GlObject::applyMaterial(const struct aiMaterial *mtl)
{
	float c[4];
	GLenum fill_mode;
	aiColor4D diffuse;
	aiColor4D specular;
	aiColor4D ambient;
	aiColor4D emission;
	int two_sided;
	int wireframe;
	unsigned int max;

	if( ! selected )
	{
		setFloat4(c, 0.8f, 0.8f, 0.8f, 1.0f);
		if(aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse) == AI_SUCCESS)
		{
			color4ToFloat4(&diffuse, c);
		}
	}
	else
	{
		setFloat4(c, 0.0f, 0.2f, 1, 1.0f);
	}

	m_shader->setColorDiffuse(c[0], c[1], c[2]);

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
		setFloat4(c, 0.0f, 0.2f, 1, 1.0f);
	}
	m_shader->setColorSpecular(c[0], c[1], c[2]);

	setFloat4(c, 0.2f, 0.2f, 0.2f, 1.0f);
	if(aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient) == AI_SUCCESS)
	{
		color4ToFloat4(&ambient, c);
	}
	m_shader->setColorAmbiant(c[0], c[1], c[2]);
#if 0
	setFloat4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	if(aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission) == AI_SUCCESS)
	{
		color4ToFloat4(&emission, c);
	}
	glMaterialfv(GL_FRONT, GL_EMISSION, c);

	max = 1;
	float shininess;
	float strength;
	int ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
	if(ret1 == AI_SUCCESS)
	{
		max = 1;
		int ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
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
#endif
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

void GlObject::render(const struct aiNode* nd)
{
	unsigned int n = 0;
	aiMatrix4x4 m = nd->mTransformation;

	aiTransposeMatrix4(&m);
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = oldModelView * glm::mat4(*((float*)&m));
	m_shader->setModelView(modelView);

	for(; n < nd->mNumMeshes; ++n)
	{
		const struct aiMesh* mesh = m_scene->mMeshes[nd->mMeshes[n]];
		applyMaterial(m_scene->mMaterials[mesh->mMaterialIndex]);
		m_meshEntries[nd->mMeshes[n]].render(GL_TRIANGLES);
	}

	for (n = 0; n < nd->mNumChildren; ++n)
	{
		render(nd->mChildren[n]);
	}

	m_shader->setModelView(oldModelView);
}

void GlObject::draw()
{
	render(m_scene->mRootNode);
}

bool Object3d::init(GlObject* obj, int _selectionId)
{
	selected = false;
	selectionId = _selectionId;
	m_obj = obj;

	return true;
}

void Object3d::draw()
{
	glStencilFunc(GL_ALWAYS, selectionId, ~0);
	glm::mat4 oldModelView = m_obj->m_shader->getModelView();
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(position.x, position.y, position.z));
	modelView = glm::rotate(modelView, theta, glm::vec3(0,0,1));
	m_obj->m_shader->setModelView(modelView);
	m_obj->selected = selected;
	m_obj->draw();
	m_obj->m_shader->setModelView(oldModelView);
	glStencilFunc(GL_ALWAYS, 0, ~0);
}
