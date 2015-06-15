#ifndef _RIGIDBODY_H
#define _RIGIDBODY_H

#include "Base/vector.h"
#include "Base/quaternion.h"
#include "Base/matrix.h"
#include <assert.h>

#define GET_OBJECTS_FORCE

namespace RPEngine
{
	class RigidBody
	{
	protected:
		Vector3 position;
		Vector3 velocity;
		Vector3 acceleration;
		real inverseMass; //inverse of kilogram
		real linearDamping; //C.1.
		real angularDamping;

		Vector3 rotation;
		Quaternion orientation; // wcs
		Matrix3 inverseInertiaTensor;
		Matrix4 transformMatrix; // derived data, mcs <-> wcs

		Matrix3 inverseInertiaTensorWorld;

		Vector3 forceAccum;
		Vector3 torqueAccum;

		Vector3 lastFrameAcceleration; // last frame linear acceleration. because of resolving resting force
		bool isAwake;

#ifdef GET_OBJECTS_FORCE //if the app needs force data
	public:
		Vector3 linearForce;
		Vector3 angularForce;
#endif
	public:
		explicit RigidBody()
		:
		position(originVect), velocity(originVect), acceleration(originVect), lastFrameAcceleration(originVect),
		inverseMass(1.0f), linearDamping(0.99f), angularDamping(0.99f),
		rotation(originVect), orientation(1,0,0,0), 
		inverseInertiaTensor(1.0f, 0, 0, 
							 0, 1.0f, 0, 
							 0, 0, 1.0f)
		{

		}
		virtual ~RigidBody(){}
		void calculateDerivedData();
		
		void addForce(const Vector3 &force);
		//wcs
		void addForceAtPoint(const Vector3 &force,
							const Vector3 &point);
		//mcs
		void addForceAtBodyPoint(const Vector3 &force,
								const Vector3 &point);

		void integrate(real duration);
		void clearAccumulators();

	public:/* utility */
		Vector3 getPointInLocalSpace(const Vector3 &point) const;
		Vector3 getPointInWorldSpace(const Vector3 &point) const;
		// these are all using angle(not radian) interface
		void rotatePositionByAngleAxis(real angle, Vector3 axis);
		void rotatePositionByAngleAxis(real angle, real x, real y, real z);
		void rotateByAngleAxis(real angle, Vector3 axis); // MCS rotation
		void rotateByAngleAxis(real angle, real x, real y, real z); // MCS rotation
		void rotateOrientationByAngleAxis(real angle, Vector3 axis);
		void rotateOrientationByAngleAxis(real angle, real x, real y, real z);
	public:/* get/set */
		//position
		void setPosition(Vector3 &position);
		void setPosition(real x, real y, real z);
		Vector3 &getPosition(); //const
		void fillPosition(Vector3 *position) const;
		void addScaledVectorPosition(Vector3 &direction, real scale);

		//velocity
		void setVelocity(Vector3 &velocity);
		void setVelocity(real x, real y, real z);
		Vector3 &getVelocity(); //const
		void fillVelocity(Vector3 *velocity) const;
		void addVelocity(Vector3 &deltaVelocity);

		//acceleration
		void setAcceleration(Vector3 &acceleration);
		void setAcceleration(real x, real y, real z);
		Vector3 &getAcceleration(); //const

		Vector3 &getLastFrameAcceleration() { return lastFrameAcceleration; }

		//mass
		void setInverseMass(real inverseMass); /* if you have called this function, you should set new tensor */
		void setMass(real mass); /* if you have called this function, you should set new tensor */
		real getInverseMass() const;
		real getMass() const;
		bool hasFiniteMass() const;

		//rotation
		void setRotation(Vector3 rotation);
		void setRotation(real x, real y, real z);
		Vector3 &getRotation();
		void addRotation(Vector3 &deltaRotation);

		//orientation
		void setOrientation(Quaternion orientation);
		Quaternion getOrientation();
		void addScaledVectorOrientation(Vector3 &direction, real scale);
		void setOrientationByAngleAxis(real angle, real x, real y, real z);
		void setOrientationByAngleAxis(real angle, Vector3 axis);

 
		//tensor
		void setInertiaTensor(const Matrix3 &inertiaTensor);
		//transform
		Matrix4 getTransform();
		void setTransform(Matrix4 transform);
		void fillInverseInertiaTensorWorld(Matrix3 *tensor) const; 
	private:
		static inline void _calculateTransformMatrix(Matrix4 &transformMatrix, 
													const Vector3 &position,
													const Quaternion &orientation)
		{
			transformMatrix.data[0] = 1-2*orientation.j*orientation.j - 
				2*orientation.k*orientation.k;
			transformMatrix.data[1] = 2*orientation.i*orientation.j - 
				2*orientation.r*orientation.k;
			transformMatrix.data[2] = 2*orientation.i*orientation.k +
				2*orientation.r*orientation.j;
			transformMatrix.data[3] = position.x;

			transformMatrix.data[4] = 2*orientation.i*orientation.j +
				2*orientation.r*orientation.k;
			transformMatrix.data[5] = 1-2*orientation.i*orientation.i - 
				2*orientation.k*orientation.k;
			transformMatrix.data[6] = 2*orientation.j*orientation.k - 
				2*orientation.r*orientation.i;
			transformMatrix.data[7] = position.y;

			transformMatrix.data[8] = 2*orientation.i*orientation.k - 
				2*orientation.r*orientation.j;
			transformMatrix.data[9] = 2*orientation.j*orientation.k +
				2*orientation.r*orientation.i;
			transformMatrix.data[10] = 1-2*orientation.i*orientation.i - 
				2*orientation.j*orientation.j;
			transformMatrix.data[11] = position.z;
		}

		//inverse inertia tensor
		static inline void _transformInertiaTensor(Matrix3 &iitWorld,
													const Quaternion &q,
													const Matrix3 &iitBody,
													const Matrix4 &rotmat)
		{
			real t4 = rotmat.data[0]*iitBody.data[0] +
				rotmat.data[1]*iitBody.data[3]+
				rotmat.data[2]*iitBody.data[6];

			real t9 = rotmat.data[0]*iitBody.data[1] +
				rotmat.data[1]*iitBody.data[4]+
				rotmat.data[2]*iitBody.data[7];

			real t14 = rotmat.data[0]*iitBody.data[2] +
				rotmat.data[1]*iitBody.data[5]+
				rotmat.data[2]*iitBody.data[8];

			real t28 = rotmat.data[4]*iitBody.data[0] +
				rotmat.data[5]*iitBody.data[3]+
				rotmat.data[6]*iitBody.data[6];

			real t33 = rotmat.data[4]*iitBody.data[1] +
				rotmat.data[5]*iitBody.data[4]+
				rotmat.data[6]*iitBody.data[7];

			real t38 = rotmat.data[4]*iitBody.data[2] +
				rotmat.data[5]*iitBody.data[5]+
				rotmat.data[6]*iitBody.data[8];

			real t52 = rotmat.data[8]*iitBody.data[0] +
				rotmat.data[9]*iitBody.data[3]+
				rotmat.data[10]*iitBody.data[6];

			real t57 = rotmat.data[8]*iitBody.data[1] +
				rotmat.data[9]*iitBody.data[4]+
				rotmat.data[10]*iitBody.data[7];

			real t62 = rotmat.data[8]*iitBody.data[2] +
				rotmat.data[9]*iitBody.data[5]+
				rotmat.data[10]*iitBody.data[8];


			iitWorld.data[0] = t4 * rotmat.data[0]+
				t9*rotmat.data[1]+
				t14*rotmat.data[2];

			iitWorld.data[1] = t4 * rotmat.data[4]+
				t9*rotmat.data[5]+
				t14*rotmat.data[6];

			iitWorld.data[2] = t4 * rotmat.data[8]+
				t9*rotmat.data[9]+
				t14*rotmat.data[10];


			iitWorld.data[3] = t28 * rotmat.data[0]+
				t33*rotmat.data[1]+
				t38*rotmat.data[2];

			iitWorld.data[4] = t28 * rotmat.data[4]+
				t33*rotmat.data[5]+
				t38*rotmat.data[6];

			iitWorld.data[5] = t28 * rotmat.data[8]+
				t33*rotmat.data[9]+
				t38*rotmat.data[10];

			iitWorld.data[6] = t52 * rotmat.data[0]+
				t57*rotmat.data[1]+
				t62*rotmat.data[2];

			iitWorld.data[7] = t52 * rotmat.data[4]+
				t57*rotmat.data[5]+
				t62*rotmat.data[6];

			iitWorld.data[8] = t52 * rotmat.data[8]+
				t57*rotmat.data[9]+
				t62*rotmat.data[10];

		}
	};
}

#endif
