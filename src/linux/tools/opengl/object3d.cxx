#include "object3d.h"

#define MIN(x,y) (x<y?x:y)
#define MAX(x,y) (y>x?y:x)

Object3dBasic::Object3dBasic()
{
	m_vbo[VERTEX_BUFFER] = 0;
	m_vbo[TEXCOORD_BUFFER] = 0;
	m_vbo[NORMAL_BUFFER] = 0;
	m_vbo[INDEX_BUFFER] = 0;
}

bool Object3dBasic::init(aiMesh *mesh, MainShader* shader)
{
	m_vbo[VERTEX_BUFFER] = 0;
	m_vbo[TEXCOORD_BUFFER] = 0;
	m_vbo[NORMAL_BUFFER] = 0;
	m_vbo[INDEX_BUFFER] = 0;

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);

	m_elementCount = mesh->mNumFaces * 3;
	m_elementSize = 3;
	if(mesh->HasPositions())
	{
		float *vertices = new float[mesh->mNumVertices * 3];
		for(unsigned int i = 0; i < mesh->mNumVertices; ++i)
		{
			vertices[i * 3] = mesh->mVertices[i].x;
			vertices[i * 3 + 1] = mesh->mVertices[i].y;
			vertices[i * 3 + 2] = mesh->mVertices[i].z;
		}

		glGenBuffers(1, &m_vbo[VERTEX_BUFFER]);
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo[VERTEX_BUFFER]);
		glBufferData(GL_ARRAY_BUFFER, mesh->mNumVertices * m_elementSize * sizeof(GLfloat), vertices, GL_STATIC_DRAW);

		glVertexAttribPointer(shader->m_attribute_coord3d, 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(shader->m_attribute_coord3d);

		delete vertices;
	}

#if 0
	if(mesh->HasTextureCoords(0))
	{
		float *texCoords = new float[mesh->mNumVertices * 2];
		for(int i = 0; i < mesh->mNumVertices; ++i)
		{
			texCoords[i * 2] = mesh->mTextureCoords[0][i].x;
			texCoords[i * 2 + 1] = mesh->mTextureCoords[0][i].y;
		}

		glGenBuffers(1, &m_vbo[TEXCOORD_BUFFER]);
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo[TEXCOORD_BUFFER]);
		glBufferData(GL_ARRAY_BUFFER, 2 * mesh->mNumVertices * sizeof(GLfloat), texCoords, GL_STATIC_DRAW);

		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray (1);

		delete texCoords;
	}
#endif
	if(mesh->HasNormals())
	{
		float *normals = new float[mesh->mNumVertices * 3];
		for(unsigned int i = 0; i < mesh->mNumVertices; ++i)
		{
			normals[i * 3] = mesh->mNormals[i].x;
			normals[i * 3 + 1] = mesh->mNormals[i].y;
			normals[i * 3 + 2] = mesh->mNormals[i].z;
		}

		glGenBuffers(1, &m_vbo[NORMAL_BUFFER]);
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo[NORMAL_BUFFER]);
		glBufferData(GL_ARRAY_BUFFER, 3 * mesh->mNumVertices * sizeof(GLfloat), normals, GL_STATIC_DRAW);

		glVertexAttribPointer(shader->m_attribute_normal, 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(shader->m_attribute_normal);

		delete normals;
	}

	if(mesh->HasFaces())
	{
		unsigned int *indices = new unsigned int[mesh->mNumFaces * 3];
		for(unsigned int i = 0; i < mesh->mNumFaces; ++i)
		{
			indices[i * 3] = mesh->mFaces[i].mIndices[0];
			indices[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
			indices[i * 3 + 2] = mesh->mFaces[i].mIndices[2];
		}

		glGenBuffers(1, &m_vbo[INDEX_BUFFER]);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[INDEX_BUFFER]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * mesh->mNumFaces * sizeof(GLuint), indices, GL_STATIC_DRAW);

		delete indices;
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	return true;
}

bool Object3dBasic::init(float* vertices, int elementSize, int elementCount, MainShader* shader, bool dynamic)
{
	GLenum drawType = GL_STATIC_DRAW;
	if( dynamic )
	{
		drawType = GL_DYNAMIC_DRAW;
	}

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);

	m_elementCount = elementCount;
	m_elementSize = elementSize;

	glGenBuffers(1, &m_vbo[VERTEX_BUFFER]);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[VERTEX_BUFFER]);
	glBufferData(GL_ARRAY_BUFFER, elementSize * elementCount * sizeof(GLfloat), vertices, drawType);

	glVertexAttribPointer(shader->m_attribute_coord3d, elementSize, GL_FLOAT, GL_FALSE, 0, NULL);
	glEnableVertexAttribArray(shader->m_attribute_coord3d);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	return true;
}

void Object3dBasic::update(float* vertices, int nbElement)
{
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[VERTEX_BUFFER]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, nbElement * m_elementSize * sizeof(GLfloat), vertices);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	m_elementCount = nbElement;
}

Object3dBasic::~Object3dBasic()
{
	if(m_vbo[VERTEX_BUFFER])
	{
		glDeleteBuffers(1, &m_vbo[VERTEX_BUFFER]);
	}

	if(m_vbo[TEXCOORD_BUFFER])
	{
		glDeleteBuffers(1, &m_vbo[TEXCOORD_BUFFER]);
	}

	if(m_vbo[NORMAL_BUFFER])
	{
		glDeleteBuffers(1, &m_vbo[NORMAL_BUFFER]);
	}

	if(m_vbo[INDEX_BUFFER])
	{
		glDeleteBuffers(1, &m_vbo[INDEX_BUFFER]);
	}

	glDeleteVertexArrays(1, &m_vao);
}

void Object3dBasic::render(GLenum mode)
{
	glBindVertexArray(m_vao);
	if( m_vbo[INDEX_BUFFER] )
	{
		glDrawElements(mode, m_elementCount, GL_UNSIGNED_INT, NULL);
	}
	else
	{
		glDrawArrays(mode, 0, m_elementCount);
	}
	glBindVertexArray(0);
}

Object3d::Object3d()
{
	m_scene = NULL;
	selected = false;
	m_meshEntries = NULL;
}

Object3d::~Object3d()
{
	if( m_meshEntries )
	{
		delete [ ] m_meshEntries;
	}
}


bool Object3d::init(const char* filename, MainShader* shader)
{
	bool res = false;

	m_shader = shader;
	m_scene = aiImportFile(filename, aiProcessPreset_TargetRealtime_MaxQuality);

	if( ! m_scene )
	{
		fprintf(stderr, "aiImportFile error\n");
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

	m_meshEntries = new Object3dBasic[m_scene->mNumMeshes]();
	for(unsigned int i = 0; i < m_scene->mNumMeshes; ++i)
	{
		m_meshEntries[i].init(m_scene->mMeshes[i], m_shader);
	}

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

void Object3d::getBoundingBox()
{
	aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);
	sceneMin.x = sceneMin.y = sceneMin.z = 1e10f;
	sceneMax.x = sceneMax.y = sceneMax.z = -1e10f;
	getBoundingBoxForNode(m_scene->mRootNode, &trafo);
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

void Object3d::render(const struct aiNode* nd)
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
		/*if(mesh->mNormals)
		{
			glEnable(GL_LIGHTING);
		}
		else
		{
			glDisable(GL_LIGHTING);
		}*/
		m_meshEntries[nd->mMeshes[n]].render(GL_TRIANGLES);
	}

	for (n = 0; n < nd->mNumChildren; ++n)
	{
		render(nd->mChildren[n]);
	}

	m_shader->setModelView(oldModelView);
}

void Object3d::draw()
{
	render(m_scene->mRootNode);
}
