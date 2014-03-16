#include <stdlib.h>
#include "simpson_integrator.h"

SimpsonState * simpson_construct_state()
{
	struct SimpsonState * pstate = 0x0;
	pstate = (SimpsonState *)malloc(sizeof(SimpsonState));

	pstate->xk    = 0;
	pstate->xkm1  = 0;
	pstate->xkm2  = 0;
	pstate->yk    = 0;
	pstate->ykm1  = 0;
	pstate->ykm2  = 0;
	pstate->dtk   = 0;
	pstate->dtkm1 = 0;
	pstate->dtkm2 = 0;
	return pstate;
}

void simpson_reset(SimpsonState * pstate, float val)
{
	pstate->xk    = 0;
	pstate->xkm1  = 0;
	pstate->xkm2  = 0;
	pstate->yk   = val;
	pstate->ykm1 = val;
	pstate->ykm2 = val;
	pstate->dtk   = 0;
	pstate->dtkm1 = 0;
	pstate->dtkm2 = 0;
}

void simpson_set_derivative(SimpsonState * pstate, float age_s, float derivative)
{
	pstate->dtk = age_s;
	pstate->xk  = derivative;
}

void simpson_compute(SimpsonState * p)
{
	float tk;
	float tkm1;

	tk   = p->dtk;
	tkm1 = p->dtkm1;

	if(!p->dtkm2) // first step
	{
		p->yk = p->ykm1 + p->xk * tk;
	}
	else  // all other steps
	{
		// Simpson formula
		p->yk  = 0.;
		p->yk += (2 * tk * tkm1 -  tk  *  tk ) * p->xkm2;
		p->yk +=    (tk + tkm1) * (tk + tkm1)  * p->xkm1;
		p->yk += (2 * tk * tkm1 - tkm1 * tkm1) * p->xk;
		p->yk *= (tk + tkm1) / (6 * tk * tkm1);
		p->yk += p->ykm2;

		// details
		/*float a, b, c;
		float res;
		a  =   1 / (tkm1*tkm1 + tk*tkm1) * p->xkm2;
		a += - 1 / (tk*tkm1)             * p->xkm1;
		a +=   1 / (tk*tk + tk*tkm1)     * p->xk;

		b  = - (tk + 2* tkm1) / (tkm1*tkm1 + tk*tkm1) * p->xkm2;
		b +=   (1 / tkm1 + 1 / tk )                   * p->xkm1;
		b += - tkm1 / (tk*tk + tk*tkm1)               * p->xk;

		c  = p->xkm2;

		res  = p->ykm2;
		res += (tkm1 + tk) * (tkm1 + tk) * (tkm1 + tk) / 3. * a;
		res += (tkm1 + tk) * (tkm1 + tk) / 2.               * b;
		res += (tkm1 + tk)                                  * c;

		p->yk = res;*/
	}

	// update buffer
	p->ykm2  = p->ykm1;
	p->ykm1  = p->yk;
	p->xkm2  = p->xkm1;
	p->xkm1  = p->xk;
	p->dtkm2 = p->dtkm1;
	p->dtkm1 = p->dtk;
}

float simpson_get(SimpsonState * pstate)
{
	return pstate->yk;
}

void simpson_destroy_state(SimpsonState * pstate)
{
	if(pstate)
	{
		free(pstate);
		pstate = 0x0;
	}
}
