#ifndef _CONTACTS_H
#define _CONTACTS_H

#include<iostream>
#include"Base/vector.h"
#include"Base/matrix.h"
#include "rbody.h"

namespace RPEngine
{
	class Contact
	{
		friend class ContactResolver;
	public:
		RigidBody *body[2];
		Vector3 relativeContactPosition[2]; // CCS

		Vector3 contactPoint;
		Vector3 contactNormal;
		Vector3 contactVelocity; //closing velocity

		real penetration; // > 0
		real friction;
		real restitution;
		
		//preparing values
		real desiredDeltaVelocity; // closing velocity at the point of contact applied restitution
		Matrix3 contactToWorld; // CCS to WCS transform matrix
	public:
		void setBodyData(RigidBody *body1, RigidBody *body2, real friction, real restitution);
		void calculateContactBasis(); // make contactToWorld, CCS matrix
		void calculateDeltaVelocity(Matrix4 *inverseInertiaTensor);
		void applyPositionChange(Vector3 linearChange[2],
									Vector3 angularChange[2],
									real penetration);
		void applyVelocityChange(Vector3 velocityChange[2],
			Vector3 rotationChange[2]);
	private: 
		void calculateInternals(real duration); //prepare its contact

		void swapBodies(); //simply swapping arr index
		Vector3 calculateLocalVelocity(int body_index, real duration);
		void calculateDesiredDeltaVelocity(real duration);
	private:
		Vector3 calculateFrictionlessImpulse(Matrix3 *inverseInertiaTensor);
		Vector3 calculateFrictionImpulse(Matrix3 *inverseInertiaTensor);
	};
	struct CollisionData
	{
		Contact *contactArray; // contact pointer must be initialized before use
		Contact *contacts; // contact pointer to load into contactArray
		int contactsLeft; // array left counter
		int contactCount; // array counter

		real friction;
		real restitution;

		void initialize(Contact *_contactArray, real _friction, real _restitution)
		{
			contactArray = _contactArray;
			friction = _friction;
			restitution = _restitution;
		}

		void addContacts(int count)
		{
			contactsLeft -= count;
			contactCount += count;
			contacts += count;
		}

		void reset(int maxContacts) //initialize CollisionData
		{
			contactsLeft = maxContacts;
			contactCount = 0;
			contacts = contactArray;
		}

		bool isFull()
		{
			return contactsLeft <= 0;
		}
	};


	class ContactResolver
	{
	private:
		int positionIterations;
		int velocityIterations; //

		int positionIterationsUsed;
		int velocityIterationsUsed; //

		real velocityEpsilon;
		real positionEpsilon; // interpenetration for detecting epsilon
	public:
		explicit ContactResolver(int iterations,
								real _velocityEpsilon=(real)0.01,
								real _positionEpsilon=(real)0.01);
		explicit ContactResolver(int _velocityIterations,
								int _positionIterations,
								real _velocityEpsilon=(real)0.01,
								real _positionEpsilon=(real)0.01);
		void resolveContacts(Contact *contactArray, 
								unsigned int numContacts, 
								real duration);
	public: //get, set
		void setIterations(int iterations);
		void setIterations(int _velocityIterations, int _positionIterations);
		void setEpsilon(real _velocityEpsilon, real _positionEpsilon);
	private:
		//resolve validation check
		bool isValid();

		//prepare common contact data
		void prepareContacts(Contact *contactArray, 
			int numContacts, 
			real duration);

		//resolve penetration
		void adjustPositions(Contact *contactArray, 
			int numContacts, 
			real duration);

		//resolve velocity
		void adjustVelocities(Contact *contactArray, 
			int numContacts, 
			real duration);
	};

	//for polymorphism
	class ContactGenerator
    {
    public:
        virtual int addContact(CollisionData *collisionData, int limit) const = 0;
    };

}

#endif
