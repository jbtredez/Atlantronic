#ifndef PATH_H
#define PATH_H

#include "kernel/math/VectPlan.h"
#include "kernel/control/kinematics.h"

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
		bool add(PathPoint* pt, unsigned int size);

		void planify(KinematicsParameters vParam, KinematicsParameters wParam);
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
		unsigned int moveId(unsigned int id, unsigned int count);
		void project(VectPlan pt, unsigned int startId, unsigned int count, VectPlan* ptProj, int* id);
		int moveCmd(float dt, KinematicsParameters vParam, KinematicsParameters wParam);
		unsigned int getRemainingCount(unsigned int id);

		// commande
		int m_cpCmdId;              //!< id du debut du segment sur lequel est la position theorique commandee
		VectPlan m_cpCmd;           //!< position theorique commandee
		VectPlan m_cpSpeedCmd;      //!< vitesse theroique commandee

		// trajectoire
		PathPoint m_pt[PATH_SIZE];     //!< buffer circulaire
		unsigned int m_count;          //!< nombre d'elements dans le buffer
		unsigned int m_head;           //!< "debut" du buffer circulaire : position ou on ajoute les elements
		unsigned int m_tail;           //!< "fin" du buffer circulaire : position ou on retire les elements
};

#endif
