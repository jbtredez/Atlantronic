#ifndef PATH_H
#define PATH_H

#include "kernel/math/VectPlan.h"
#include "kernel/control/kinematics.h"
#include "kernel/kinematics_model/KinematicsModel.h"

// remarque : la taille du path doit etre une puissance de 2 pour avoir un code optimal
#define PATH_SIZE            512

struct PathPoint
{
	VectPlan pos;
	float vMax;
	float wMax;
};

class Path
{
	public:
		Path();

		void clear();
		bool add(PathPoint* pt, uint32_t size);

		void planify(KinematicsModel* kinematicsModel, Kinematics* kinematicsCmdTmp, KinematicsParameters vParam, KinematicsParameters wParam);
		VectPlan getNextCommand(VectPlan mes, float dt, KinematicsParameters vParam, KinematicsParameters wParam);
		void display();
		VectPlan getLastPoint();
		inline int getCount()
		{
			return m_count;
		}
		inline VectPlan getLastPosCmd()
		{
			return m_cpCmd;
		}

	protected:
		void setMaxSpeed(float vMax, float wMax);
		uint32_t moveId(uint32_t id, uint32_t count);
		void project(VectPlan pt, uint32_t startId, uint32_t count, VectPlan* ptProj, int* id);
		int moveCmd(float dt, KinematicsParameters vParam, KinematicsParameters wParam);
		uint32_t getRemainingCount(uint32_t id);

		// commande
		int m_cpCmdId;              //!< id du debut du segment sur lequel est la position theorique commandee
		VectPlan m_cpCmd;           //!< position theorique commandee
		VectPlan m_cpSpeedCmd;      //!< vitesse theroique commandee

		// trajectoire
		PathPoint m_pt[PATH_SIZE];     //!< buffer circulaire
		uint32_t m_count;              //!< nombre d'elements dans le buffer
		uint32_t m_head;               //!< "debut" du buffer circulaire : position ou on ajoute les elements
		uint32_t m_tail;               //!< "fin" du buffer circulaire : position ou on retire les elements
};

#endif
