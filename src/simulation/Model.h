#ifndef MODEL_H
#define MODEL_H

#include <stdint.h>

class Model
{
public:
	Model(){};
	virtual ~Model(){};

	virtual void update(uint64_t vm_ck) = 0;
};


#endif
