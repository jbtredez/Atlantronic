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

		float val[12];
};

#endif
