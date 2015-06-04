#ifndef _STATIC_JOINT_H
#define _STATIC_JOINT_H

#include "../Base/vector.h"
#include "../contacts.h"
#include "../narrow_collision.h"

// Refactoring required

namespace RPEngine
{
	class StaticJoint : public ContactGenerator
	{
	public:
		RigidBody *body[2];
		Vector3 position[2]; // body connection points at each MCS
		real staticDistance; // (> 0)
		real epsilonDistance; // epsilon (> 0)
	public:
		// You must initialize the joint by calling set()
		explicit StaticJoint() {}

		explicit StaticJoint(RigidBody *bodyA, const Vector3 & a_pos,
			RigidBody *bodyB, const Vector3 & b_pos,
			real _staticDistance, real _epsilonDistance) // body connection points at each MCS and distance of joint
		{
			set(bodyA, a_pos, bodyB, b_pos, epsilonDistance, _staticDistance);
		}

		virtual ~StaticJoint() {}

		// If you want to joint with an extra point, use bodyB be null, b_pos be the point in WCS
		void set(RigidBody *bodyA, const Vector3 & a_pos,
			RigidBody *bodyB, const Vector3 & b_pos, 
			real _epsilonDistance, real _staticDistance);
		virtual int addContact(CollisionData *data, int limit) const;
	};
}

#endif
