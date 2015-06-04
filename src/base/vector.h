#ifndef _RP_ENGINE_BASE_H
#define _RP_ENGINE_BASE_H

/*
Vector3 must be fast data structure. 
Hence, We implement in this header by using inline keyword.
*/

#include<math.h>
#include"utility.h"
#include"precision.h"

namespace RPEngine
{
	class Vector3
	{
	public:
		real x;
		real y;
		real z;
	private:
		// Option. Vector3 struct has only 4 real data values for machine performance.
		// So this pad data is not used any algorithm.
		real pad; 

	public:
		explicit Vector3() : x(0), y(0), z(0) { }
		explicit Vector3(const real x, const real y, const real z) : x(x), y(y), z(z) { }


	public:
		// The reason why using not 'get-()' but natural function name is that 
		// current function includes some COSTS for calculating.
		inline void invert()
		{
			x = -x;
			y = -y;
			z = -z;
		}

		// Warning : 'NaN' can be occured. 
		// So you should handle this case when you use this if you have to.
		inline real magnitude()
		{
			return real_sqrt(x*x + y*y + z*z);
		}
		
		// This costs less than magnitude(). 
		// So, We suggest to use this method for your any algorithm instead of 'a.magnitude()*a.magnitude()'.
		inline real squareMagnitude()
		{
			return (x*x + y*y + z*z);
		}

		inline void normalize()
		{
			real scalar = magnitude();
			if(scalar > 0)
				(*this) *= static_cast<real>(1.0)/scalar;
		}

		// Set to zero vector.
		inline void clear()
		{
			x = y = z = 0;
		}
		
	public: /* Vector operations */
		inline void addScaledVector(const Vector3& v, real scale)
		{
			x += v.x * scale;
			y += v.y * scale;
			z += v.z * scale;
		}

		inline Vector3 componentProduct(const Vector3& v) const
		{
			return Vector3(x * v.x, y * v.y, z * v.z);
		}

		// updateComponentProduct
		inline void componentProductUpdate(const Vector3& v)
		{
			x *= v.x;
			y *= v.y;
			z *= v.z;
		}

		inline real scalarProduct(const Vector3& v) const
		{
			return x*v.x+y*v.y+z*v.z;
		}

		inline Vector3 vectorProduct(const Vector3& v)
		{
			return Vector3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
		}

	public: /* [scalar *,/ vecter] operator overloading */
		inline void operator*=(const real value)
		{
			x *= value;
			y *= value;
			z *= value;
		}

		inline Vector3 operator*(const real value) const // C.2. //
		{
			return Vector3(x*value, y*value, z*value);
		}

		inline void operator/=(const real value)
		{
			x /= value;
			y /= value;
			z /= value;
		}

		inline Vector3 operator/(const real value) const
		{
			return Vector3(x/value, y/value, z/value);
		}

	public: /* [vector +,- vector] operator overloading */
		inline void operator+=(const Vector3& v)
		{
			x += v.x;
			y += v.y;
			z += v.z;
		}

		inline Vector3 operator+(const Vector3& v)
		{
			return Vector3(x+v.x, y+v.y, z+v.z);
		}

		inline void operator-=(const Vector3& v)
		{
			x -= v.x;
			y -= v.y;
			z -= v.z;
		}
		
		inline Vector3 operator-(const Vector3& v)
		{
			return Vector3(x-v.x, y-v.y, z-v.z);
		}

	public: /* [inner/outer vector product] operator overloading */
		inline real operator*(const Vector3& v) const // *= is defined by 'product *= scalar' but not here.
		{
			return x*v.x+y*v.y+z*v.z;
		}

		inline Vector3 operator%=(const Vector3& v)
		{
			(*this) = vectorProduct(v);
		}

		inline Vector3 operator%(const Vector3& v)
		{
			return Vector3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
		}
	public:
		inline real operator[](int i) const
		{
			if(i == 0) return x;
			if(i == 1) return y;
			return z;
		}
		
		inline real &operator[](int i)
		{
			if(i == 0) return x;
			if(i == 1) return y;
			return z;
		}
		
	public: /* Utilities */

		// Make orthogonal and normalize at the same time.
		static inline void makeOrthonormalBasis(Vector3& a, Vector3& b, Vector3& c)
		{
			a.normalize();
			c = a % b;

			
			if(IsEqual(c.magnitude(), 0)) return; // a,b is parallel
			c.normalize();
			b = a % c;
		}

	};

	// This nullVect will be used to check null vector.
	static const Vector3 nullVect;
	// This originVect will be used to initialize any vector.
	static const Vector3 originVect;

	// Default Epsilon = 0.0001f;
	static inline bool IsEqual(Vector3 &v1, Vector3 &v2, real epsilon = EPSILON) 
	{
		if(real_abs(v1.x - v2.x) < epsilon)
			if(real_abs(v1.y - v2.y) < epsilon)
				if(real_abs(v1.z - v2.z) < epsilon)
					return true;
		return false;
	}
}

#endif
