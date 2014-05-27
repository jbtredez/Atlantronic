#ifndef MATRIX_HOMOGENEOUS_H
#define MATRIX_HOMOGENEOUS_H

class MatrixHomogeneous
{
	public:
		MatrixHomogeneous();

		void setIdentity();
		void translate(float x, float y, float z);
		void rotateX(float theta);
		void rotateY(float theta);
		void rotateZ(float theta);

		inline float getX()
		{
			return val[3];
		}

		inline float getY()
		{
			return val[7];
		}

		inline float getZ()
		{
			return val[7];
		}

		float val[12];
} __attribute__((packed));

#endif
