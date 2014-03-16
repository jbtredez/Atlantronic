#ifndef __SIMPSON_INTEGRATOR_H__
#define __SIMPSON_INTEGRATOR_H__

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct SimpsonState
{
	// derivatives
	float xk;
	float xkm1;
	float xkm2;

	// outputs
	float yk;
	float ykm1;
	float ykm2;

	// times
	float dtk;
	float dtkm1;
	float dtkm2;
} SimpsonState;

SimpsonState * simpson_construct_state();
void simpson_reset(SimpsonState * pstate, float val);
void simpson_set_derivative(SimpsonState * pstate, float age_s, float derivative);
void simpson_compute(SimpsonState * pstate);
float simpson_get(SimpsonState * pstate);
void simpson_destroy_state(SimpsonState * pstate);

#ifdef __cplusplus
}
#endif

#endif
