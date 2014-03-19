#include <stdlib.h>
#include "simpson_integrator.h"

Simpson::Simpson()
{
	reset(0);
}

void Simpson::reset(float val)
{
	xk    = 0;
	xkm1  = 0;
	xkm2  = 0;
	yk   = val;
	ykm1 = val;
	ykm2 = val;
	dtk   = 0;
	dtkm1 = 0;
	dtkm2 = 0;
}

void Simpson::set_derivative(float age_s, float derivative)
{
	dtk = age_s;
	xk  = derivative;
}

void Simpson::compute()
{
	float tk;
	float tkm1;

	tk   = dtk;
	tkm1 = dtkm1;

	if(!dtkm2) // first step
	{
		yk = ykm1 + xk * tk;
	}
	else  // all other steps
	{
		// Simpson formula
		yk  = 0.;
		yk += (2 * tk * tkm1 -  tk  *  tk ) * xkm2;
		yk +=    (tk + tkm1) * (tk + tkm1)  * xkm1;
		yk += (2 * tk * tkm1 - tkm1 * tkm1) * xk;
		yk *= (tk + tkm1) / (6 * tk * tkm1);
		yk += ykm2;

		// details
		/*float a, b, c;
		float res;
		a  =   1 / (tkm1*tkm1 + tk*tkm1) * p->xkm2;
		a += - 1 / (tk*tkm1)             * p->xkm1;
		a +=   1 / (tk*tk + tk*tkm1)     * p->xk;

		b  = - (tk + 2* tkm1) / (tkm1*tkm1 + tk*tkm1) * p->xkm2;
		b +=   (1 / tkm1 + 1 / tk )                   * p->xkm1;
		b += - tkm1 / (tk*tk + tk*tkm1)               * p->xk;

		c  = xkm2;

		res  = ykm2;
		res += (tkm1 + tk) * (tkm1 + tk) * (tkm1 + tk) / 3. * a;
		res += (tkm1 + tk) * (tkm1 + tk) / 2.               * b;
		res += (tkm1 + tk)                                  * c;

		yk = res;*/
	}

	// update buffer
	ykm2  = ykm1;
	ykm1  = yk;
	xkm2  = xkm1;
	xkm1  = xk;
	dtkm2 = dtkm1;
	dtkm1 = dtk;
}
