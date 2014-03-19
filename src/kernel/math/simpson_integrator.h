#ifndef __SIMPSON_INTEGRATOR_H__
#define __SIMPSON_INTEGRATOR_H__

class Simpson
{
	public:
		Simpson();
		void reset(float val);
		void set_derivative(float age_s, float derivative);
		void compute();

		inline float get()
		{
			return yk;
		}

	protected:
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
};

#endif
