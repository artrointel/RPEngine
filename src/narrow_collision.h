#ifndef _NARROW_COLLISION_H
#define	_NARROW_COLLISION_H
#include"contacts.h"
#include"vector.h"
#include"rbody.h"

namespace RPEngine
{
	/* Primitive collision models */
	class CollisionPrimitive
	{
	public:
		RigidBody *body;
		Matrix4 offset;
	public:
		Matrix4 transform; //resultant of body.transform * offset
	public:
		void calculateInternals();

		Vector3 getAxis(unsigned int index) const;// getPositionData from its transform and this index should be 3
	};
	
	/* Collision classes */
	class CollisionSphere : public CollisionPrimitive
	{
	public:
		real radius;
	};

	class CollisionPlane // it doesn't represent any RigidBody.
	{
	public:
		Vector3 direction; //normalized direction vector
		real offset;
	};

	class CollisionBox : public CollisionPrimitive
	{
	public:
		Vector3 halfSize;
	};
	
	/* Collision Detector */
	class CollisionDetector
	{
	public:/* detect collision and fill collision data */
		//sphere
		static unsigned int sphere_sphere(const CollisionSphere &sphere1, 
									const CollisionSphere &sphere2, 
									CollisionData *data);

		static unsigned int sphere_halfSpace(const CollisionSphere &sphere,
										const CollisionPlane &plane,
										CollisionData *data);
		static unsigned int sphere_truePlane(const CollisionSphere &sphere,
			const CollisionPlane &plane,
			CollisionData *data);

		//box
		static unsigned int box_sphere(const CollisionBox &box,
			const CollisionSphere &sphere,
			CollisionData *data);
		static unsigned int box_halfSpace(const CollisionBox &box,
			const CollisionPlane &plane,
			CollisionData *data);
		//box_box

		static unsigned int box_box(const CollisionBox &one,
									const CollisionBox &two,
									CollisionData *data);

	public:/* search collision from collision data */

		// find body1 & body2 Collision from contact data and you can process the callback function
		static bool bodyCollision(
			RigidBody *body1, RigidBody *body2, real deltaVelocity,
			CollisionData *_data, void(*callBack)() = NULL); //O(n)

		// get Collision number
		static int getCollisionNumber(
			RigidBody *bodyTarget, real deltaVelocity,
			CollisionData *_data); //O(n^2)

		// get only one colliding body with target
		static RigidBody *getBodyCollisionWith(
			RigidBody *bodyTarget, real deltaVelocity,
			CollisionData *_data); //O(n)

		// get colliding bodies array with size 
		static void getBodyArrayCollisionWith(
			RigidBody *bodyTarget, real deltaVelocity, int limit, RigidBody *pBodyArray[], int *bodySize, 
			CollisionData *_data); //O(n^2)

	private:
		// find Collision body1 and get Contact pointer one
		static Contact *getCollisionContactData(
			RigidBody *bodyTarget, real deltaVelocity,
			CollisionData *_data, int _size);

	public:/* (trigger) search collision from collision data and remove the collision data */
		//
	};
}

#endif
