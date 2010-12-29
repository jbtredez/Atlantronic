#include "Table.h"
#include "log.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

void Table::addMeshToTreeCollisionStandard(IMeshBuffer* meshBuffer, NewtonCollision* treeCollision, vector3df scale)
{
	vector3df vArray[3];

	S3DVertex* mb_vertices = (S3DVertex*) meshBuffer->getVertices();

	u16* mb_indices  = meshBuffer->getIndices();

	for (unsigned int j = 0; j < meshBuffer->getIndexCount(); j += 3)
	{
		int v1i = mb_indices[j + 0];
		int v2i = mb_indices[j + 1];
		int v3i = mb_indices[j + 2];

		vArray[0] = mb_vertices[v1i].Pos * scale.X;
		vArray[1] = mb_vertices[v2i].Pos * scale.Y;
		vArray[2] = mb_vertices[v3i].Pos * scale.Z;

		NewtonTreeCollisionAddFace(treeCollision, 3, &vArray[0].X, sizeof (vector3df), 1);
	}
}

void Table::addMeshToTreeCollision2TCoords(IMeshBuffer* meshBuffer, NewtonCollision* treeCollision, vector3df scale)
{
	vector3df vArray[3];

	S3DVertex2TCoords* mb_vertices = (S3DVertex2TCoords*) meshBuffer->getVertices();

	u16* mb_indices  = meshBuffer->getIndices();

	for (unsigned int j = 0; j < meshBuffer->getIndexCount(); j += 3)
	{
		int v1i = mb_indices[j + 0];
		int v2i = mb_indices[j + 1];
		int v3i = mb_indices[j + 2];

		vArray[0] = mb_vertices[v1i].Pos * scale.X;
		vArray[1] = mb_vertices[v2i].Pos * scale.Y;
		vArray[2] = mb_vertices[v3i].Pos * scale.Z;

		NewtonTreeCollisionAddFace(treeCollision, 3, &vArray[0].X, sizeof (vector3df), 1);
	}
}

void Table::addMeshToTreeCollisionTangents(IMeshBuffer* meshBuffer, NewtonCollision* treeCollision, vector3df scale)
{
	vector3df vArray[3];

	S3DVertexTangents* mb_vertices = (S3DVertexTangents*) meshBuffer->getVertices();

	u16* mb_indices  = meshBuffer->getIndices();

	for (unsigned int j = 0; j < meshBuffer->getIndexCount(); j += 3)
	{
		int v1i = mb_indices[j + 0];
		int v2i = mb_indices[j + 1];
		int v3i = mb_indices[j + 2];

		vArray[0] = mb_vertices[v1i].Pos * scale.X;
		vArray[1] = mb_vertices[v2i].Pos * scale.Y;
		vArray[2] = mb_vertices[v3i].Pos * scale.Z;

		NewtonTreeCollisionAddFace(treeCollision, 3, &vArray[0].X, sizeof (vector3df), 1);
	}
}

NewtonBody* Table::createNewtonTree(NewtonWorld *world, IAnimatedMeshSceneNode *node, int id)
{
	NewtonCollision *treeCollision;

	treeCollision = NewtonCreateTreeCollision(world, id);
	NewtonTreeCollisionBeginBuild(treeCollision);

	IMesh *mesh = node->getMesh();

	for (unsigned int i = 0; i < mesh->getMeshBufferCount(); i++)
	{
		IMeshBuffer *mb = mesh->getMeshBuffer(i);

		switch(mb->getVertexType())
		{
			case EVT_STANDARD:
				addMeshToTreeCollisionStandard(mb, treeCollision, node->getScale()/1000);
			break;

			case EVT_2TCOORDS:
				addMeshToTreeCollision2TCoords(mb, treeCollision, node->getScale()/1000);
			break;

			case EVT_TANGENTS:
				addMeshToTreeCollisionTangents(mb, treeCollision, node->getScale()/1000);
			break;

			default:
				meslog(_erreur_, "Newton error: Unknown vertex type in static mesh: %d\n", mb->getVertexType());
				exit(-1);
			break;
		}
	}

	NewtonTreeCollisionEndBuild(treeCollision, 1);

	NewtonBody* treeBody = NewtonCreateBody(world, treeCollision);

	NewtonReleaseCollision(world, treeCollision);

	NewtonBodySetUserData(treeBody, node);

	return treeBody;
}

Table::Table(NewtonWorld *newtonWorld, irr::scene::ISceneManager* smgr)
{
	mesh = smgr->getMesh( "media/table.3ds" );

	if(mesh)
	{
		node = smgr->addAnimatedMeshSceneNode( mesh );
		node->setMaterialFlag(EMF_BACK_FACE_CULLING, true);

		body = createNewtonTree(newtonWorld, node, 0);
		NewtonBodySetMassMatrix(body, 0, 0, 0, 0);

		vector3df min = node->getBoundingBox().MinEdge;
		vector3df max = node->getBoundingBox().MaxEdge;

		origin = (max + min)/2.0f;
		origin.Y = -28.0f;

		matrix4 m;
		// newton en m et non mm
		m.setTranslation( -origin/1000);
		NewtonBodySetMatrix(body, m.pointer());
		node->setPosition( - origin );
	}
	else
	{
		meslog(_erreur_, "impossible de charger le fichier 'media/table.3ds' ");
	}
}

Table::~Table()
{
	// FIXME : destruction du "nody" newton (actuellement réalisé uniquement lors de NewtonDestroy(newtonWorld)
	node->remove();
}

