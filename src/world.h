#ifndef _RPWORLD_H
#define _RPWORLD_H

#include<iostream>
#include<vector>
#include"rbody.h"
#include"fgen.h"
#include"contacts.h"
#include"narrow_collision.h"
#include"world_collisionEvent.h"

#define GET_OBJECTS_FORCE

namespace RPEngine
{
	class RPWorld
	{
	public:
		int ITERATIONS_PER_CONTACT; //it can mean Collision Accuracy
	public:
		typedef std::vector<RigidBody *> RigidBodies;
		typedef std::vector<ContactGenerator *> ContactGenerators;
		typedef std::vector<CollisionBox *> CollisionBoxes;
		typedef std::vector<CollisionSphere *> CollisionSpheres;
		typedef std::vector<CollisionPlane *> CollisionPlanes;

		typedef std::vector<BodyCollision *> BodyCollisionEvents;
		typedef std::vector<CollisionNumber *> CollisionNumberEvents;
		typedef std::vector<CollisionWith *> CollisionWithEvents;
		typedef std::vector<CollisionWithArray *> CollisionWithArrayEvents;
	public:
		RigidBodies bodies;
		ForceRegistry registry;

	public://BVH, BSP for broad collision, not yet.

	public:/* contact detection */
		bool calculateIterations; // true if resolver should calculate at that frame
		unsigned int maxContacts;

		// collision system
		Contact *contactArray;
		CollisionData collisionData;
		ContactGenerators contactGenerators;
		ContactResolver resolver;

		// test for narrow collision model container
		CollisionBoxes collisionBoxes; 
		CollisionSpheres collisionSpheres; 
		CollisionPlanes collisionPlanes; 

	private:/* collision event containers */

		BodyCollisionEvents bodyCollisionEvents;
		CollisionNumberEvents getCollisionNumberEvents;
		CollisionWithEvents getCollisionWithEvents;
		CollisionWithArrayEvents getCollisionWithArrayEvents;

#ifdef GET_OBJECTS_FORCE /* special data */
		Vector3 BiggestLinearForceAtFrame;
		Vector3 BiggestAngularForceAtFrame;
#endif
	public:
		//explicit RPWorld(){}
		explicit RPWorld(unsigned int _maxContacts, unsigned int iterations)
			: resolver(iterations), maxContacts(_maxContacts),
				ITERATIONS_PER_CONTACT(4)			  
		{

			contactArray = new Contact[maxContacts];
			collisionData.initialize(contactArray, 0.9, 0.4); //as default
			calculateIterations = (iterations == 0);
		}
		virtual ~RPWorld()
		{
			delete[] contactArray;
		}
		
	public:// physics
		void startFrame();
		unsigned int generateContacts();
		void integrate(real duration);
		void runPhysics(real duration);
		
	public:/* utilities */
		//delete back
		void UndoContactGenerate(unsigned int size); 
		void UndoForceGenerate(unsigned int size);

		void clearData(); //you must clear your data directly by calling delete operator
	private:
		int _narrowCollisionDetect();
		void _broadCollisionDetect(); //not yet.
		
		void _collisionEventExcute(real duration); // Contact *contactArray, int numContacts);
		void _collisionTriggerExcute();

	public:/* collision event registration */
		
		// O(n) find body1 & body2 Collision from contact data and you can process the callback function
		BodyCollision *addBodyCollisionEvent(RigidBody *body1, RigidBody *body2, 
			void(*callBack)() = NULL, real deltaVelocity = 1, real eventCoolTime = 0.2);
		bool removeBodyCollisionEvent(BodyCollision *collisionEvent);
		void clearBodyCollisionEvent(); //delete all data not only pointer

		// O(n^2) get Collision number
		CollisionNumber *addCollisionNumberEvent(RigidBody *bodyTarget, 
			 real deltaVelocity = 1, real eventCoolTime = 0.2); //detecting coolTime in milli second
		bool removeCollisionNumberEvent(CollisionNumber *collisionEvent);
		void clearCollisionNumberEvent(); //delete all data not only pointer
		
		// O(n) get only one colliding body with target
		CollisionWith *addCollisionWithEvent(RigidBody *bodyTarget, 
			 real deltaVelocity = 1, real eventCoolTime = 0.2);
		bool removeCollisionWithEvent(CollisionWith *collisionEvent);
		void clearCollisionWithEvent(); //delete all data not only pointer

		// O(n^2) get colliding bodies array with size 
		CollisionWithArray *addCollisionWithArrayEvent(RigidBody *bodyTarget, int limit,
			 real deltaVelocity = 1, real eventCoolTime = 0.2);
		bool removeCollisionWithArrayEvent(CollisionWithArray *collisionEvent);
		void clearCollisionWithArrayEvent(); //delete all data not only pointer

	private:/* template version not yet */
		/*
		template<class T>
		bool removeCollision(T *collisionEvent, std::vector<T*> container)
		{
			class T::type::iterator iter = container.begin();
			for(; iter != container.end(); iter++)
				if(collisionEvent == (*iter))
				{
					container.erase(iter);
					return true;
				}
				return false;
		}
		*/
	};
}

#endif
