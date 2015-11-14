#ifndef PATH_H
#define PATH_H

#include "kernel/math/VectPlan.h"
#include "kernel/control/kinematics.h"

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
		bool add(PathPoint* pt, int size);

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
		int moveId(int id, int count);
		void project(VectPlan pt, int startId, int count, VectPlan* ptProj, int* id);
		int moveCmd(float dt, KinematicsParameters vParam, KinematicsParameters wParam);
		int getRemainingCount(int id);

		// commande
		int m_cpCmdId;              //!< id du debut du segment sur lequel est la position theorique commandee
		VectPlan m_cpCmd;           //!< position theorique commandee
		VectPlan m_cpSpeedCmd;      //!< vitesse theroique commandee

		// trajectoire
		PathPoint m_pt[PATH_SIZE];     //!< buffer circulaire
		int m_count;                   //!< nombre d'elements dans le buffer
		int m_head;                    //!< "debut" du buffer circulaire : position ou on ajoute les elements
		int m_tail;                    //!< "fin" du buffer circulaire : position ou on retire les elements
};

#endif
