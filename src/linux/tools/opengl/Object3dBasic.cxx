#include "Object3dBasic.h"

GlObjectBasic::GlObjectBasic()
{
	m_vbo[VERTEX_BUFFER] = 0;
	m_vbo[TEXCOORD_BUFFER] = 0;
	m_vbo[NORMAL_BUFFER] = 0;
	m_vbo[INDEX_BUFFER] = 0;
}

bool GlObjectBasic::init(aiMesh *mesh, MainShader* shader)
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

bool GlObjectBasic::init(float* vertices, int elementSize, int elementCount, MainShader* shader, bool dynamic)
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

void GlObjectBasic::update(float* vertices, int nbElement)
{
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[VERTEX_BUFFER]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, nbElement * m_elementSize * sizeof(GLfloat), vertices);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	m_elementCount = nbElement;
}

GlObjectBasic::~GlObjectBasic()
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

void GlObjectBasic::render(GLenum mode)
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
