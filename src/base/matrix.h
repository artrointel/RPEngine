#ifndef _RPMATRIX_H
#define _RPMATRIX_H

#include "vector.h"
/*
Matrix3, 4 must be fast data structure. 
Hence, We implement in this header by using inline keyword.
*/

namespace RPEngine
{
	class Matrix3 // 3 by 3
	{
	public:
		real data[9];
	public:
		Matrix3(){}
		Matrix3(real a, real b, real c,
				real d, real e, real f,
				real g, real h, real i)
		{
			data[0] = a; data[1] = b; data[2] = c;
			data[3] = d; data[4] = e; data[5] = f;
			data[6] = g; data[7] = h; data[8] = i;
		}
		
	public:
		inline Vector3 transform(const Vector3 &vector) const
		{
			return (*this) * vector;
		}

		inline Vector3 transformTranspose(const Vector3 &vector) const
		{
			return Vector3(
				vector.x * data[0] + 
				vector.y * data[3] + 
				vector.z * data[6],

				vector.x * data[1] + 
				vector.y * data[4] + 
				vector.z * data[7],

				vector.x * data[2] + 
				vector.y * data[5] + 
				vector.z * data[8]
			);
		}
	public:
		inline Vector3 operator*(const Vector3 &vector) const
		{
			return Vector3(
				vector.x * data[0] + 
				vector.y * data[1] + 
				vector.z * data[2],

				vector.x * data[3] + 
				vector.y * data[4] + 
				vector.z * data[5],

				vector.x * data[6] + 
				vector.y * data[7] + 
				vector.z * data[8]
				);
		}

		inline Matrix3 operator*(const Matrix3 &mat) const
		{
			return Matrix3(
				data[0]*mat.data[0] + data[1]*mat.data[3] + data[2]*mat.data[6],
				data[0]*mat.data[1] + data[1]*mat.data[4] + data[2]*mat.data[7],
				data[0]*mat.data[2] + data[1]*mat.data[5] + data[2]*mat.data[8],
				
				data[3]*mat.data[0] + data[4]*mat.data[3] + data[5]*mat.data[6],
				data[3]*mat.data[1] + data[4]*mat.data[4] + data[5]*mat.data[7],
				data[3]*mat.data[2] + data[4]*mat.data[5] + data[5]*mat.data[8],
				
				data[6]*mat.data[0] + data[7]*mat.data[3] + data[8]*mat.data[6],
				data[6]*mat.data[1] + data[7]*mat.data[4] + data[8]*mat.data[7],
				data[6]*mat.data[2] + data[7]*mat.data[5] + data[8]*mat.data[8]
				);
		}

		inline Matrix3 operator+(const Matrix3 &mat) const
		{
			return Matrix3(
				data[0]+mat.data[0],
				data[1]+mat.data[1],
				data[2]+mat.data[2],
				data[3]+mat.data[3],
				data[4]+mat.data[4],
				data[5]+mat.data[5],
				data[6]+mat.data[6],
				data[7]+mat.data[7],
				data[8]+mat.data[8]
			);
		}

		inline Matrix3 operator-(const Matrix3 &mat) const
		{
			return Matrix3(
				data[0]-mat.data[0],
				data[1]-mat.data[1],
				data[2]-mat.data[2],

				data[3]-mat.data[3],
				data[4]-mat.data[4],
				data[5]-mat.data[5],
				
				data[6]-mat.data[6],
				data[7]-mat.data[7],
				data[8]-mat.data[8]
			);
		}
		
		inline void operator*=(const real scalar)
        {
            data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
            data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
            data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
        }

        inline void operator+=(const Matrix3 &mat)
        {
            data[0] += mat.data[0]; data[1] += mat.data[1]; data[2] += mat.data[2];
            data[3] += mat.data[3]; data[4] += mat.data[4]; data[5] += mat.data[5];
            data[6] += mat.data[6]; data[7] += mat.data[7]; data[8] += mat.data[8];
        }

		inline void operator*=(const Matrix3 &mat)
		{
			real col1;
			real col2;
			real col3;

			col1 = data[0]*mat.data[0] + data[1]*mat.data[3] + data[2]*mat.data[6];
			col2 = data[0]*mat.data[1] + data[1]*mat.data[4] + data[2]*mat.data[7];
			col3 = data[0]*mat.data[2] + data[1]*mat.data[5] + data[2]*mat.data[8];
			data[0] = col1;
			data[1] = col2;
			data[2] = col3;

			col1 = data[3]*mat.data[0] + data[4]*mat.data[3] + data[5]*mat.data[6];
			col2 = data[3]*mat.data[1] + data[4]*mat.data[4] + data[5]*mat.data[7];
			col3 = data[3]*mat.data[2] + data[4]*mat.data[5] + data[5]*mat.data[8];
			data[3] = col1;
			data[4] = col2;
			data[5] = col3;

			col1 = data[6]*mat.data[0] + data[7]*mat.data[3] + data[8]*mat.data[6];
			col2 = data[6]*mat.data[1] + data[7]*mat.data[4] + data[8]*mat.data[7];
			col3 = data[6]*mat.data[2] + data[7]*mat.data[5] + data[8]*mat.data[8];
			data[6] = col1;
			data[7] = col2;
			data[8] = col3;
		}
		
		static inline real getDeterminant(const Matrix3 &mat)
		{
			return mat.data[0] * mat.data[4] * mat.data[8] +
					mat.data[2] * mat.data[3] * mat.data[7] +
					mat.data[1] * mat.data[5] * mat.data[6] -
					(mat.data[2] * mat.data[4] * mat.data[6] +
					 mat.data[0] * mat.data[5] * mat.data[7] +
					 mat.data[1] * mat.data[3] * mat.data[8]);
		}

		//차라리 getInverse를 새로 만드는 것이 더 나을 것 같다.
		inline void setInverse(const Matrix3 &mat)
		{
			real detM = getDeterminant(mat);
			if(detM == static_cast<real>(0.0f)) return; // but almost not return, it requires epsilon
			real invDetM = static_cast<real>(1.0f) / detM;

			data[0] = (mat.data[4]*mat.data[8] - mat.data[5]*mat.data[7]) * invDetM;
			data[1] = -(mat.data[1]*mat.data[8] - mat.data[2]*mat.data[7]) * invDetM;
			data[2] = (mat.data[1]*mat.data[5] - mat.data[2]*mat.data[4]) * invDetM;
			data[3] = -(mat.data[3]*mat.data[8] - mat.data[5]*mat.data[6]) * invDetM;
			data[4] = (mat.data[0]*mat.data[8] - mat.data[2]*mat.data[6]) * invDetM;
			data[5] = -(mat.data[0]*mat.data[5] - mat.data[2]*mat.data[3]) * invDetM;
			data[6] = (mat.data[3]*mat.data[7] - mat.data[4]*mat.data[6]) * invDetM;
			data[7] = -(mat.data[0]*mat.data[7] - mat.data[1]*mat.data[6]) * invDetM;
			data[8] = (mat.data[0]*mat.data[4] - mat.data[1]*mat.data[3]) * invDetM;
		}

		inline void setTranspose(const Matrix3 &mat)
		{
			data[0] = mat.data[0];
			data[1] = mat.data[3];
			data[2] = mat.data[6];

			data[3] = mat.data[1];
			data[4] = mat.data[4];
			data[5] = mat.data[7];

			data[6] = mat.data[2];
			data[7] = mat.data[5];
			data[8] = mat.data[8];
		}

		inline Matrix3 inverse() const
		{
			Matrix3 result;
			result.setInverse(*this);
			return result;
		}

		inline Matrix3 transpose() const
		{
			Matrix3 result;
			result.setTranspose(*this);
			return result;
		}

		//대각행렬
		inline void setSkewSymmetric(Vector3 vector)
		{
			data[0] = data[4] = data[8] = 0;
			data[1] = -vector.z;
			data[2] = vector.y;
			data[3] = vector.z;

			data[5] = -vector.x;
			data[6] = -vector.y;
			data[7] = vector.x;
		}

		inline void setComponents(Vector3 &x_axis, Vector3 &y_axis, Vector3 &z_axis)
		{
			data[0] = x_axis.x;
			data[1] = y_axis.x;
			data[2] = z_axis.x;

			data[3] = x_axis.y;
			data[4] = y_axis.y;
			data[5] = z_axis.y;
	
			data[6] = x_axis.z;
			data[7] = y_axis.z;
			data[8] = z_axis.z;
		}

		//void setOrientation(const Quaternion &q)
		static Matrix3 linearInterpolate(const Matrix3 &a, 
										 const Matrix3 &b,
										 real prop)
		{
			Matrix3 result;
			real omp = 1.0 - prop;
			for(int i = 0; i < 9; i++)
				result.data[i] = a.data[i] * omp + b.data[i] * prop;
	
			return result;
		}


	};

	class Matrix4 //3 by 4
	{
	public:
		real data[12];
	public:
		explicit Matrix4();
		real getDeterminant() const;
		void setInverse(const Matrix4 &mat);
		Matrix4 inverse() const;
		void invert();
		// Vector multiplication of transform
		Vector3 transform(const Vector3 &vector) const;
		Vector3 transformInverse(const Vector3 &vector) const;
		// Vector multiplication only direction elements
		Vector3 transformDirection(const Vector3 &vector) const;
		Vector3 transformInverseDirection(const Vector3 &vector) const;
	
	public: /* Utilities */
		static inline Vector3 worldToLocal(const Vector3 &world, const Matrix4 &transform) 
		{ return transform.transformInverse(world); }
		static inline Vector3 localToWorld(const Vector3 &local, const Matrix4 &transform) 
		{ return transform.transform(local); }
		static inline Vector3 worldToLocalDirn(const Vector3 &world, const Matrix4 &transform)
		{ return transform.transformInverseDirection(world); }
		static inline Vector3 localToWorldDirn(const Vector3 &local, const Matrix4 &transform)
		{ return transform.transformDirection(local); }
		inline void fillGLArray(float array[16]) const
		{
			array[0] = (float)data[0];
			array[1] = (float)data[4];
			array[2] = (float)data[8];
			array[3] = (float)0;

			array[4] = (float)data[1];
			array[5] = (float)data[5];
			array[6] = (float)data[9];
			array[7] = (float)0;

			array[8] = (float)data[2];
			array[9] = (float)data[6];
			array[10] = (float)data[10];
			array[11] = (float)0;

			array[12] = (float)data[3];
			array[13] = (float)data[7];
			array[14] = (float)data[11];
			array[15] = (float)1;
		}
		inline void setFromGLArray(float array[16])
		{
			data[0] = array[0];
			data[4] = array[1];
			data[8] = array[2];
			
			data[1] = array[4];
			data[5] = array[5];
			data[9] = array[6];

			data[2] = array[8];
			data[6] = array[9];
			data[10] = array[10];
			
			data[3] = array[12];
			data[7] = array[13];
			data[11] = array[14];
		}
	public: /* get/set */
		//void setOrientationAndPos(const Quaternion &q, const Vector3 &pos);
		Vector3 getAxisVector(int axis) const;
	public:
		inline Vector3 operator*(const Vector3 &vector) const
		{
			return Vector3(
				vector.x * data[0] +
				vector.y * data[1] +
				vector.z * data[2] + 
				data[3],

				vector.x * data[4] +
				vector.y * data[5] +
				vector.z * data[6] + 
				data[7],

				vector.x * data[8] +
				vector.y * data[9] +
				vector.z * data[10] + 
				data[11]
				);
		}

		inline Matrix4 operator*(const Matrix4 &mat) const
		{
			Matrix4 result;
			result.data[0] = data[0] * mat.data[0] +
							 data[1] * mat.data[4] +
							 data[2] * mat.data[7];
			result.data[1] = data[0] * mat.data[1] +
							 data[1] * mat.data[5] +
							 data[2] * mat.data[9];
			result.data[2] = data[0] * mat.data[2] +
							 data[1] * mat.data[6] +
							 data[2] * mat.data[10];
			result.data[3] = data[0] * mat.data[3] +
							 data[1] * mat.data[7] +
							 data[2] * mat.data[11] +
							 data[3];

			result.data[4] = data[4] * mat.data[0] +
							 data[5] * mat.data[4] +
							 data[6] * mat.data[7];
			result.data[5] = data[4] * mat.data[1] +
							 data[5] * mat.data[5] +
							 data[6] * mat.data[9];
			result.data[6] = data[4] * mat.data[2] +
							 data[5] * mat.data[6] +
							 data[6] * mat.data[10];
			result.data[7] = data[4] * mat.data[3] +
							 data[5] * mat.data[7] +
							 data[6] * mat.data[11] +
							 data[7];

			result.data[8] = data[8] * mat.data[0] +
							 data[9] * mat.data[4] +
							 data[10] * mat.data[7];
			result.data[9] = data[8] * mat.data[1] +
							 data[9] * mat.data[5] +
							 data[10] * mat.data[9];
			result.data[10] =data[8] * mat.data[2] +
							 data[9] * mat.data[6] +
							 data[10] * mat.data[10];
			result.data[11] =data[8] * mat.data[3] +
							 data[9] * mat.data[7] +
							 data[10] * mat.data[11] +
							 data[11];

			return result;
		}
	};
}

#endif
