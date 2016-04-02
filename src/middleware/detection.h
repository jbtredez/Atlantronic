#ifndef DETECTION_H
#define DETECTION_H

//! @file detection.h
//! @brief Detection
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/VectPlan.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/math/polyline.h"

#define DETECTION_NUM_OBJECT                100
#define DETECTION_NUM_OBJECT_USB             10
#define DETECTION_OPPONENT_ROBOT_RADIUS     200
#define HOKUYO_REG_SEG                      200

enum hokuyo_id
{
	HOKUYO1 = 0,
	HOKUYO2,
	HOKUYO_MAX,
};

enum detection_type
{
	DETECTION_FULL,
	DETECTION_STATIC_OBJ,
	DETECTION_DYNAMIC_OBJ,
};

struct detection_object
{
	float x;
	float y;
	float size;
} __attribute__((packed));

#ifndef LINUX

typedef void (*DetectionCallback)(void* arg);

class Detection
{
	public:
		int init(Hokuyo* hokuyo1, Hokuyo* hokuyo2, Location* location);
		void registerCallback(DetectionCallback callback, void* arg);

		//!< Calcule en fonction de la position (actuelle ou non) pos du robot le segment [a b] (repère table) tel
		//!< que [a b] soit la projection - sur l'axe Y du repère robot - de l'obstacle le plus proche dans l'axe du robot.
		//!< => dans le repère robot, a.x = b.x = distance avant collision (valeur retournée) et [a.y b.y] largeur de
		//!< l'objet vue par le robot. Le robot ne doit pas dépasser le segment [a b]
		//!<
		//!< attention, prise de mutex sur l'ensemble des segments
		//!<
		//!< @return distance maximale d'avance avant collision
		float computeFrontObject(enum detection_type type, const VectPlan& pos, Vect2* a, Vect2* b);

		float computeOpponentInRangeDistance(Vect2 a, Vect2 u);

		float computeOpponentDistance(Vect2 a);

	protected:
		static void hokuyo1Callback(void* arg);
		static void hokuyo2Callback(void* arg);
		static void taskWrapper(void* arg);
		void task();
		void compute();
		void removeStaticElementsFromDynamicList();
		float getSegmentSimilarity( Vect2* a,  Vect2* b,  Vect2* m,  Vect2* n);
		float computeObjectOnTrajectory(const VectPlan& pos, const struct polyline* polyline, int size, Vect2* a, Vect2* b);

		// données privées à la tache detection
		DetectionCallback m_callbackFunction;
		void* m_callbackArg;
		Vect2 m_hokuyoPos[HOKUYO_NUM_POINTS];
		int m_regEcart;
		Hokuyo* m_hokuyo1;
		Hokuyo* m_hokuyo2;

		// données partagées par la tache et des méthodes d'accés
		xQueueHandle m_queue;
		xSemaphoreHandle m_mutex;
		Vect2 m_hokuyoReg[HOKUYO_REG_SEG];
		Vect2 m_omronRectangle[5];
		int m__regSize;
		struct polyline m__objectPolyline[DETECTION_NUM_OBJECT];
		struct detection_object m__obj[DETECTION_NUM_OBJECT];
		//static struct detection_object detection_obj2[DETECTION_NUM_OBJECT];
		int m_numObj;

		Vect2 m_detectionOpponentRobotPt[5];
		Location* m_location;
};

#endif

#endif
