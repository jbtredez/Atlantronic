#include <string.h>
#include "Path.h"
#include "kernel/math/findRotation.h"
#include "kernel/log.h"

#define PROJECTION_THETA_WEIGHT           573

#define min(a,b)     ((a) < (b)) ? (a) : (b)
#define max(a,b)     ((a) > (b)) ? (a) : (b)

Path::Path()
{
	clear();
}

void Path::clear()
{
	m_count = 0;
	m_head = 0;
	m_tail = 0;

	m_cpCmdId = 0;
	m_cpCmd = VectPlan();
	m_cpSpeedCmd = VectPlan();
}

bool Path::add(PathPoint* pt, int n)
{
	int nMax;

	if (m_count + n > PATH_SIZE)
	{
		return false;
	}

	// nombre d elements max pouvant etre copies sans cycler sur le buffer
	nMax = PATH_SIZE - m_head;
	if(n < nMax)
	{
		nMax = n;
	}
	memcpy(&m_pt[m_head], pt, nMax * sizeof(PathPoint));

	if( n > nMax )
	{
		// on cycle sur le buffer
		// copie du reste
		memcpy(&m_pt[0], &pt[nMax], (n - nMax) * sizeof(PathPoint));
	}

	m_head = (m_head + n) % PATH_SIZE;
	m_count += n;

	return true;
}

VectPlan Path::getLastPoint()
{
	if( m_count < 1)
	{
		return VectPlan();
	}
	return m_pt[(m_tail + m_count - 1)%PATH_SIZE].pos;
}

void Path::planify(KinematicsParameters vParam, KinematicsParameters wParam)
{
	if( m_count < 1)
	{
		// rien a faire
		return;
	}

	// correction de tout les angles si ce n'est pas bon, pas de discontinuite en theta
	// on garde le premier point tel quel
	for(int i = m_tail + 1; i < m_tail + m_count; i++)
	{
		float oldTheta = m_pt[(i-1)%PATH_SIZE].pos.theta;
		m_pt[i%PATH_SIZE].pos.theta = oldTheta + findRotation(oldTheta, m_pt[i%PATH_SIZE].pos.theta);
	}

	// on met toute les vitesses a fond
	setMaxSpeed(vParam.vMax, wParam.vMax);

	// reduction de vitesse selon la deceleration du robot en partant de la fin pour
	// respecter les conditions de vitesses max aux points
	float vmax = 0;
	float wmax = 0;

	for(int i = m_tail + m_count - 1; i > m_tail; i--)
	{
		PathPoint* a = &m_pt[(i-1)%PATH_SIZE];
		PathPoint* b = &m_pt[i%PATH_SIZE];

		VectPlan ab = b->pos - a->pos;
		ab.theta = modulo2pi(ab.theta);
		float distLin = ab.norm();
		float distAng = fabsf(ab.theta);

		VectPlan ba = a->pos - b->pos;
		ba.theta = modulo2pi(ba.theta);

		if( i != m_tail + m_count - 1 )
		{
			PathPoint* c = &m_pt[(i+1)%PATH_SIZE];
			VectPlan cb = b->pos - c->pos;
			cb.theta = modulo2pi(cb.theta);
			float nextDist = cb.norm();
			if( nextDist < EPSILON )
			{
				vmax = 0;
			}
			if( fabsf(cb.theta) < EPSILON )
			{
				wmax = 0;
			}
		}

		// vitesses max au point de fin en fonction de l'iteration precedente (segment d'apres)
		b->vMax = min(vmax, vParam.vMax);
		b->wMax = min(wmax, wParam.vMax);

		float vmaxSlowDown = sqrtf(b->vMax * b->vMax + 2 * vParam.dMax * distLin);
		vmax = min(vmaxSlowDown, vParam.vMax);
		float wmaxSlowDown = sqrtf(b->wMax * b->wMax + 2 * wParam.dMax * distAng);
		wmax = min(wmaxSlowDown, wParam.vMax);

		if( distLin > EPSILON && distAng > EPSILON )
		{
			// translation rotation combinee - reduction de vitesse
			// prise en compte de la courbure
			float sigmaAbs = distAng / distLin;
			wmax = min(wmax, sigmaAbs * vmax);
			vmax = min(vmax, wmax/sigmaAbs);
		}
	}
}

//! mise a jour de la vitesse max pour l'ensemble des points du path
void Path::setMaxSpeed(float vMax, float wMax)
{
	for(int i = m_tail; i < m_tail + (int)m_count; i++)
	{
		m_pt[i % PATH_SIZE].vMax = vMax;
		m_pt[i % PATH_SIZE].wMax = wMax;
	}
}

int Path::moveId(int id, int count)
{
	if( m_tail < m_head)
	{
		// id doit etre entre m_tail et m_head
		if( id < m_tail || id >= m_head )
		{
			// erreur, id invalide
			return -1;
		}

		id += count;
		if(id < m_tail)
		{
			id = m_tail;
		}

		if(id > m_head)
		{
			id = m_head;
		}
	}
	else
	{
		// id doit etre entre m_tail et capacity ou entre 0 et m_head
		if( id < 0 || id >= PATH_SIZE || (id >= m_head && id < m_tail ) )
		{
			// erreur, startId invalide
			return -1;
		}

		id = (id + count) % PATH_SIZE;
		if( id >= m_head && id < m_tail )
		{
			// id en dehors de la plage, on a deborde
			if( count < 0)
			{
				id = m_tail;
			}
			else
			{
				id = m_head;
			}
		}
	}

	return id;
}

VectPlan projectOnSegment(const VectPlan &A, const VectPlan &B, VectPlan C)
{
	VectPlan H;

	VectPlan AB = B - A;
	float nab = AB.norm(PROJECTION_THETA_WEIGHT);

	// on met C.theta entre A.theta et B.theta si possible et avec abs(B.theta - C.theta) le plus court possible
	float dTheta = modulo2pi(C.theta - B.theta);
	if(dTheta > 0)
	{
		dTheta -= 2*M_PI;
	}

	if( AB.theta >= 0)
	{
		C.theta = B.theta + dTheta;

		if(C.theta < A.theta)
		{
			// C.theta en dehors de AB => choix du theta le plus proche de A ou B
			float da = A.theta - C.theta;
			float db = C.theta + 2*M_PI - B.theta;
			if(db < da)
			{
				C.theta += 2*M_PI;
			}
		}
	}
	else
	{
		C.theta = B.theta + dTheta + 2 * M_PI;

		if(C.theta > A.theta)
		{
			// H.theta en dehors de AB => choix du theta le plus proche de A ou B
			float da = C.theta - A.theta;
			float db = B.theta - (C.theta - 2*M_PI);
			if(db < da)
			{
				C.theta -= 2*M_PI;
			}
		}
	}

	VectPlan AC = C - A;
	float factor = 0;

	if(nab > EPSILON)
	{
		VectPlan ux = AB / nab;
		float ah = AC.scalarProd(ux, PROJECTION_THETA_WEIGHT);
		factor =  ah / nab;
	}
	else
	{
		factor = AC.theta / AB.theta;
	}

	H = A + factor * AB;

	if( factor < 0)
	{
		// H sort du segment du coté de A
		// la projection est donc A
		return A;
	}
	else if( factor > 1)
	{
		// H sort du segment du coté de B
		// la projection est donc B
		return B;
	}

	return H;
}

void Path::project(VectPlan pt, int startId, int count, VectPlan* ptProj, int* id)
{
	float dmin = HUGE_VALF;
	int bestId = -1;
	VectPlan bestProjection;
	int endId = startId;

	if( m_count == 0 )
	{
		// path vide, pas de solution
		log_format(LOG_ERROR, "path is empty");
		return;
	}

	endId = moveId(startId, count);

	if( startId < 0 || endId < 0)
	{
		// erreur
		log_format(LOG_ERROR,"invalid path id : start id %d end id %d ", startId, endId);
		return;
	}

	// on met startId < endId pour la boucle et on fait du modulo
	if(startId > endId)
	{
		endId += PATH_SIZE;
	}

	// on est sur le dernier point - projection sur un point
	if( startId == endId -1 )
	{
		bestProjection = m_pt[startId % PATH_SIZE].pos;
		bestId = startId;
	}

	for(int i = startId; i < endId - 1; i++)
	{
		PathPoint a = m_pt[i % PATH_SIZE];
		PathPoint b = m_pt[(i+1) % PATH_SIZE];
		VectPlan h = projectOnSegment(a.pos, b.pos, pt);
		VectPlan hpt = pt - h;
		float d = hpt.norm(PROJECTION_THETA_WEIGHT);

		if( d < dmin - EPSILON)
		{
			dmin = d;
			if( h == b.pos)
			{
				// on est arrive au bout du path
				bestId = (i + 1) % PATH_SIZE;
			}
			else
			{
				bestId = i % PATH_SIZE;
			}
			bestProjection = h;
		}
	}

	if(bestId >= 0)
	{
		// on a trouve
		*ptProj = bestProjection;
		*id = bestId;
	}
}

int Path::getRemainingCount(int id)
{
	int remainingCount = 0;

	if( m_count < 2)
	{
		log_format(LOG_ERROR, "path empty (< 2 points) : %d points", m_count);
		return -1;
	}

	if( m_head > m_tail )
	{
		// verification de la coherence de l'id. Il doit etre dans [m_tail m_head-1]
		if( id < m_tail || id >= m_head )
		{
			log_format(LOG_ERROR, "id %d out of range [%d - %d[", id, m_tail, m_head);
			return -1;
		}
		remainingCount = m_head - 1 - id;
	}
	else
	{
		// verification de la coherence de l'id
		if( (id < m_tail && id >= m_head) || id < 0 || id >= PATH_SIZE )
		{
			log_format(LOG_ERROR, "id %d out of range [%d - %d] U [ 0 - %d]", id, m_tail, PATH_SIZE-1, m_head-1);
			return -1;
		}
		remainingCount = m_head -1 + PATH_SIZE - id;
	}

	return remainingCount;
}

VectPlan Path::getNextCommand(VectPlan mes, float dt, KinematicsParameters vParam, KinematicsParameters wParam)
{
	// on ne souhaite pas rattraper l'erreur selon la trajectoire, on se place sur la projection
	project(mes, m_cpCmdId, 50, &m_cpCmd, &m_cpCmdId);

	VectPlan oldCmd = m_cpCmd;

	// nombre de points restant avant la fin du path
	int remainingCount = getRemainingCount(m_cpCmdId);
	if(remainingCount < 1)
	{
		// on est arrive a la fin du path, on ne bouge plus
		return VectPlan();
	}

	moveCmd(dt, vParam, wParam);
	m_cpSpeedCmd = (m_cpCmd - oldCmd) / dt;

	return m_cpSpeedCmd;
}

int Path::moveCmd(float dt, KinematicsParameters vParam, KinematicsParameters wParam)
{
	if( m_count == 0)
	{
		return -1;
	}

	int endId = (m_cpCmdId + 1) % PATH_SIZE;
	if( endId == m_head )
	{
		return -1;
	}

	PathPoint a = m_pt[m_cpCmdId];
	PathPoint b = m_pt[endId];
	VectPlan ab = b.pos - a.pos;
	VectPlan u;
	float nab = ab.norm();
	float ds = 0;

	if(nab > EPSILON)
	{
		u = ab / nab;
		ds = nab;

		KinematicsParameters param = vParam;

		// en cas de rotation, on prend en compte wParam
		float sigmaAbs = fabsf(u.theta);
		if( sigmaAbs > EPSILON )
		{
			// contraintes pour respecter wMax, aMax, dMax
			if( (1/sigmaAbs) * wParam.vMax < param.vMax)
			{
				param.vMax = (1/sigmaAbs) * wParam.vMax;
			}

			if( (1/sigmaAbs) * wParam.aMax < param.aMax)
			{
				param.aMax = (1/sigmaAbs) * wParam.aMax;
			}

			if( (1/sigmaAbs) * wParam.dMax < param.dMax)
			{
				param.dMax = (1/sigmaAbs) * wParam.dMax;
			}
		}

		VectPlan acp = m_cpCmd - a.pos;
		Kinematics k;
		k.pos = acp.scalarProd(u); // pos actuelle sur le segment
		k.v = m_cpSpeedCmd.scalarProd(u);
		k.setPosition(ds, b.vMax, param, dt);
		m_cpCmd = m_cpCmd + dt * k.v * u;
		project(m_cpCmd, m_cpCmdId, 50, &m_cpCmd, &m_cpCmdId);
	}
	else if(fabsf(ab.theta) > EPSILON)
	{
		// rotation pure
		float ds = ab.theta;

		Kinematics k;
		k.pos = m_cpCmd.theta - a.pos.theta; // pos actuelle sur le segment
		k.v = m_cpSpeedCmd.theta;
		k.setPosition(ds, b.wMax, wParam, dt);
		m_cpCmd.theta = m_cpCmd.theta + dt * k.v;
		project(m_cpCmd, m_cpCmdId, 50, &m_cpCmd, &m_cpCmdId);
	}
	else
	{
		// erreur, A == B
		log_format(LOG_ERROR, "Path error, A == B : %d ; %d ; %d", (int)a.pos.x, (int)a.pos.y, (int)(a.pos.theta * 180 / M_PI));
		return -1;
	}

	return 0;
}

void Path::display()
{
	log_format(LOG_INFO, "%3s %4s %4s %6s %6s %6s", "id", "x", "y", "theta", "vMax", "wMax" );

	for(int i = m_tail; i < m_tail + (int)m_count; i++)
	{
		PathPoint* pt = &m_pt[i % PATH_SIZE];

		log_format(LOG_INFO, "%3d %4d %4d %6d %6d %6d", i, (int)pt->pos.x, (int)pt->pos.y, (int)(pt->pos.theta*180/M_PI), (int)pt->vMax, (int)(pt->wMax*180/M_PI));
	}

	log_format(LOG_INFO, "cpCmdId %d", m_cpCmdId);
	log_format(LOG_INFO, "cpCmd %d %d %d", (int)m_cpCmd.x, (int)m_cpCmd.y, (int)(m_cpCmd.theta*180/M_PI));
}
