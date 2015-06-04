#ifndef _JOINT_H
#define _JOINT_H

#include "Base/vector.h"
#include "contacts.h"
#include "narrow_collision.h"
/*
'Joint' ContactGenerator is used to joint two vector position.
*/


namespace RPEngine
{
	class Joint : public ContactGenerator
	{
	public:
		RigidBody *body[2];
		Vector3 position[2]; // body connection points at each MCS
		real epsilonDistance; // epsilon (> 0)
	public:
		// You must initialize the joint by calling set()
		explicit Joint() {}

		explicit Joint(RigidBody *bodyA, const Vector3 & a_pos,
			RigidBody *bodyB, const Vector3 & b_pos,
			real _epsilonDistance) // body connection points at each MCS and distance of joint
		{
			set(bodyA, a_pos, bodyB, b_pos, _epsilonDistance);
		}
		virtual ~Joint() {}

		// If you want to joint with an extra point, remain bodyB be null and set b_pos to the point in WCS.
		void set(RigidBody *bodyA, const Vector3 & a_pos,
				RigidBody *bodyB, const Vector3 & b_pos,
				real _epsilonDistance);
		virtual int addContact(CollisionData *data, int limit) const;

	};
}

#endif
