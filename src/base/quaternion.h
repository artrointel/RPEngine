#ifndef _QUATERNION_H
#define _QUATERNION_H

#include "vector.h"
/*
Quaternion must be fast data structure. 
Hence, We implement in this header by using inline keyword.
*/

namespace RPEngine
{
	class Quaternion
	{
	public:
		union
		{
			struct  
			{
				real r;
				real i;
				real j;
				real k;
			};
			real data[4];
		};
	public:
		explicit Quaternion()
			:
			r(1.f), i(0.f), j(0.f), k(0.f) {}

		explicit Quaternion(real _r, real _i, real _j, real _k)
			:
			r(_r), i(_i), j(_j), k(_k) {}
		inline void normalize()
		{
			real d = r*r + i*i + j*j + k*k;
			if(IsEqual(d, 0))
			{
				r = 1;
				return;
			}

			d = static_cast<real>(1.0f) / real_sqrt(d);
			r *= d;
			i *= d;
			j *= d;
			k *= d;
		}

		inline void rotateByVector(const Vector3 &vector)
		{
			Quaternion q(0, vector.x, vector.y, vector.z);
			(*this) *= q;
		}
		inline void addScaledVector(const Vector3 &vector, real scale)
		{
			Quaternion q(0, 
				vector.x * scale, 
				vector.y * scale, 
				vector.z * scale);
			q *= *this;

			real half = static_cast<real>(0.5f);
			r += q.r * half;
			i += q.i * half;
			j += q.j * half;
			k += q.k * half;
		}

		inline Quaternion getConjugate()
		{
			return Quaternion(r,-i,-j,-k);
		}

	public:/* Rotation utilities */
		inline void fillAngleAxis(real &angle, real &x, real &y, real &z) const
		{
			Quaternion q = (*this);
			if (r > 1) q.normalize();
			
			angle = 2*acos(q.r);
			double s = sqrt(1-q.r*q.r); // Assuming quaternion normalized then w is less than 1, so term always positive.
			if (s < 0.001) { // To avoid divide by zero, s is always positive due to sqrt
				x = q.i;
				y = q.j;
				z = q.k;
			} else { // normalize
				x = q.i / s;
				y = q.j / s;
				z = q.k / s;
			}
			angle = 180.0f*angle/3.141592f;
		}

		inline void fillAngleAxis(real &angle, Vector3 &axis) const
		{
			Quaternion q = (*this);
			if (r > 1) q.normalize();

			angle = 2*acos(q.r);
			double s = sqrt(1-q.r*q.r);
			if (s < 0.001) {
				axis.x = q.i;
				axis.y = q.j;
				axis.z = q.k;
			} else {
				axis.x = q.i / s;
				axis.y = q.j / s;
				axis.z = q.k / s;
			}
			angle = 180.0f*angle/3.141592f;
		}
		
		// These set methods can be static method and returns its quaternion.
		inline void setByAngleAxis(real angle, real x, real y, real z)
		{
			angle = 3.141592f*angle/180.f;
			Vector3 axis(x,y,z);
			axis.normalize();

			double s = real_sin(angle/2.f);
			this->r = real_cos(angle/2.f);
			this->i = axis.x*s;
			this->j = axis.y*s;
			this->k = axis.z*s;
		}

		inline void setByAngleAxis(real angle, Vector3 axis)
		{
			angle = real_pi*angle/180.f;
			axis.normalize();

			double s = real_sin(angle/2.f);
			this->r = real_cos(angle/2.f);
			this->i = axis.x*s;
			this->j = axis.y*s;
			this->k = axis.z*s;
		}

		inline void rotateByAngleAxis(real angle, Vector3 axis)
		{
			Quaternion temp;
			temp.setByAngleAxis(angle, axis);
			*this *= temp;
		}
		
		inline void rotateByAngleAxis(real angle, real x, real y, real z)
		{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
			Vector3 axis(x, y, z);
			Quaternion temp;
			temp.setByAngleAxis(angle, axis);
			*this *= temp;
		}

		// Returns a rotated vector by current quaterion.
		inline Vector3 operator*(const Vector3& target)
		{
			// qVq*
			Quaternion V(0, target.x, target.y, target.z);
			Quaternion q(*this);
			q.normalize();				//  normalize current quaternion.
			q *= V;						// qV
			q *= getConjugate();		// qVq*
			// q == qVq*
			return Vector3(q.i, q.j, q.k);
		}

	public:
		inline void operator*=(const Quaternion &multiplier)
		{
			Quaternion q = *this;
			r = q.r*multiplier.r - q.i*multiplier.i -
				q.j*multiplier.j - q.k*multiplier.k;

			i = q.r*multiplier.i + q.i*multiplier.r +
				q.j*multiplier.k - q.k*multiplier.j;

			j = q.r*multiplier.j + q.j*multiplier.r +
				q.k*multiplier.i - q.i*multiplier.k;

			k = q.r*multiplier.k + q.k*multiplier.r +
				q.i*multiplier.j - q.j*multiplier.i;
		}
	};
}

#endif
