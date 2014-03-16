#ifndef __SIMPSON_INTEGRATOR_H__
#define __SIMPSON_INTEGRATOR_H__

typedef float scalar;

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct SimpsonState
{
  // derivatives
  scalar xk;
  scalar xkm1;
  scalar xkm2;

  // outputs
  scalar yk;
  scalar ykm1;
  scalar ykm2;

  // times
  scalar dtk;
  scalar dtkm1;
  scalar dtkm2;

} SimpsonState;

SimpsonState * simpson_construct_state();
void simpson_reset(SimpsonState * pstate, scalar val);
void simpson_set_derivative(SimpsonState * pstate, scalar age_s, scalar derivative);
void simpson_compute(SimpsonState * pstate);
scalar simpson_get(SimpsonState * pstate);
void simpson_destroy_state(SimpsonState * pstate);

#ifdef __cplusplus
}
#endif

#endif
