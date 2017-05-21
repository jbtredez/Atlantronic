/*
 * UnloadBall.h
 *
 *  Created on: 21 mai 2017
 *      Author: jul
 */

#ifndef UNLOADBALL_H_
#define UNLOADBALL_H_

class UnloadBall: public Action
{
	private :
		VectPlan m_Intermedary_checkpoint;
	public:
		UnloadBall();
		virtual ~UnloadBall();
};

#endif /* UNLOADBALL_H_ */
