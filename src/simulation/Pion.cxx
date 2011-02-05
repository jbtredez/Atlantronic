#include "Pion.h"
#include "log.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

void Pion::forceAndTorqueCallback(const NewtonBody *nbody, float, int)
{
	float m, ixx, iyy, izz;
	NewtonBodyGetMassMatrix(nbody, &m, &ixx, &iyy, &izz);
	float force[3] = {0,  -9.81 * m, 0};
	NewtonBodyAddForce(nbody, force);
}

void Pion::transformCallback(const NewtonBody *nbody, const float* mat, int)
{
	Pion* p = (Pion*) NewtonBodyGetUserData(nbody);
	matrix4 m;
	memcpy(m.pointer(), mat, sizeof(float)*16);
	p->node->setRotation(m.getRotationDegrees());
	// attention, conversion en m => mm
	p->node->setPosition(m.getTranslation()*1000 - p->origin);
}

Pion::Pion(NewtonWorld *newtonWorld, irr::scene::ISceneManager* smgr, const char* fichier)
{
	float m = 0.5;
	matrix4 offset;

	mesh = smgr->getMesh( fichier );

	if(mesh)
	{
		node = smgr->addAnimatedMeshSceneNode( mesh );
		node->setMaterialFlag(EMF_BACK_FACE_CULLING, true);

		vector3df min = node->getBoundingBox().MinEdge;
		vector3df max = node->getBoundingBox().MaxEdge;

		vector3df size = max - min;
		vector3df center = (max + min)/2.0f;

		float r = size.X/2000.0f;
		float h = size.Y/1000;
		float Ixx = 0.25f * m * r * r + 1.0f/3.0f * m * h * h;
		float Iyy = 0.5f * m * r * r;
		float Izz = Ixx;

		origin.X = center.X;
		origin.Y = min.Y;
		origin.Z = center.Z;

		offset.setTranslation((center-origin)/1000.0f);
		offset.setRotationDegrees( vector3df(0, 0, 90) );
		NewtonCollision* treeCollision = NewtonCreateCylinder(newtonWorld, r, h, 0, offset.pointer());

		matrix4 identity;

		body = NewtonCreateBody(newtonWorld, treeCollision, identity.pointer());
		NewtonReleaseCollision(newtonWorld, treeCollision);
		NewtonBodySetUserData(body, this);
		NewtonBodySetMassMatrix(body, m, Ixx, Iyy, Izz);
		NewtonBodySetForceAndTorqueCallback(body, forceAndTorqueCallback);
		NewtonBodySetTransformCallback(body, transformCallback);
	}
	else
	{
		meslog(_erreur_, "impossible de charger le fichier %s", fichier);
	}
}

Pion::~Pion()
{
	// FIXME : destruction du "nody" newton (actuellement réalisé uniquement lors de NewtonDestroy(newtonWorld)
	node->remove();
}

void Pion::setPosition(irr::core::vector3df pos, irr::core::vector3df rot)
{
	matrix4 matrix;
	matrix.makeIdentity();
	matrix.setRotationDegrees(rot);
	// attention, conversion en mm => m pour newton
	matrix.setTranslation(pos/1000);
	NewtonBodySetMatrix(body, matrix.pointer());
}

