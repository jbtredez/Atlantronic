#ifndef ENVIRONNEMENT_INTERFACE_H
#define ENVIRONNEMENT_INTERFACE_H

#include <stdint.h>

class EnvironnementInterface
{
public:
	EnvironnementInterface(){};
	virtual ~EnvironnementInterface(){};

	virtual void update() = 0;
};


#endif
