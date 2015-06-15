/*
hpp file
Policy Base Design
*/

#ifndef _COLLISION_EVENT_H
#define _COLLISION_EVENT_H

#include "rbody.h"
#include "contacts.h"
#include "narrow_collision.h"

namespace RPEngine
{
	struct CollisionTrigger
	{
		RigidBody *body1;
		RigidBody *body2;
		void (*callback)();
	};

	class CollisionEvent
	{
	public:
		virtual void OnCollision() = 0;
	};
	/*
	template<class CollisionType>
	class OnCollisionEvent : public CollisionType
	{
	public:
		typedef CollisionType type;

		void eventExecute()
		{
			execute();
		}
	};*/

	//refactoring required.
	/* Policy classes */
	class BodyCollision : public CollisionEvent
	// find body1 & body2 Collision from contact data and you can process the callback function
	{
		friend class RPWorld;
	public:
		typedef BodyCollision type;
		RigidBody *body1;
		RigidBody *body2;
		void (*OnCollisionCallBack)();
	private:
		real eventCoolTime;
		real coolTimer;
		real deltaVelocity;
		CollisionData *data;
	private:
		bool collide;
	public:
		explicit BodyCollision()
			:
		body1(NULL), body2(NULL), OnCollisionCallBack(NULL), deltaVelocity(1),
			eventCoolTime(0), coolTimer(0), data(NULL)
		{
			
		}

		bool isCollide() { return collide; }
		real getCoolTime() { return eventCoolTime; }
	protected:
		inline void set(
			RigidBody *_body1, RigidBody *_body2, real _deltaVelocity,
			CollisionData *_data, void (*OnCollision)(), real _eventCoolTime) 
		{
			this->body1 = _body1;
			this->body2 = _body2;
			this->deltaVelocity = _deltaVelocity;
			this->data = _data;
			this->OnCollisionCallBack = OnCollision;
			this->eventCoolTime = _eventCoolTime;
		}

		inline void execute(real duration)
		{
			collide = false;
			if(eventCoolTime <= coolTimer)
			{
				collide = CollisionDetector::bodyCollision(
					body1, body2, deltaVelocity, data, 
					OnCollisionCallBack);
				if(collide == false)
					return;
				coolTimer = 0;
			}
			else
				coolTimer += duration;
		}
		void OnCollision()
		{

		}
	};
	
	class CollisionNumber 
	//getCollisionNumber
	{
		friend class RPWorld;
	public:
		typedef CollisionNumber type;
		RigidBody *bodyTarget;
	private:
		real eventCoolTime;
		real coolTimer;
		real deltaVelocity;
		CollisionData *data;
	private:
		int number;
	public:
		explicit CollisionNumber()
			:
		bodyTarget(NULL), deltaVelocity(1), number(0), data(NULL),
			eventCoolTime(0), coolTimer(0)
		{

		}
		
		int getNumber() { return number; }
		real getCoolTime() { return eventCoolTime; }
	protected:
		inline void set(RigidBody *_bodyTarget, real _deltaVelocity, CollisionData *_data, real _eventCoolTime)
		{
			this->bodyTarget = _bodyTarget;
			this->deltaVelocity = _deltaVelocity;
			this->data = _data;
			this->eventCoolTime = _eventCoolTime;
		}

		inline void execute(real duration)
		{
			number = 0;
			if(eventCoolTime <= coolTimer)
			{
				number = CollisionDetector::getCollisionNumber(
					bodyTarget, deltaVelocity, data);
				if(number == 0)
					return;
				coolTimer = 0;
			}
			else
				coolTimer += duration;
		}
	};

	class CollisionWith 
	//getBodyCollisionWith
	{
		friend class RPWorld;
	public:
		typedef CollisionWith type;
		RigidBody *bodyTarget;
	private:
		real eventCoolTime;
		real coolTimer;
		real deltaVelocity;
		CollisionData *data;
	private:
		RigidBody *collideBody;
	public:
		explicit CollisionWith()
			:
		bodyTarget(NULL), deltaVelocity(1), collideBody(NULL), data(NULL), 
			eventCoolTime(0), coolTimer(0)
		{

		}

		RigidBody *getBody() { return collideBody; }
		real getCoolTime() { return eventCoolTime; }
	protected:
		inline void set(RigidBody *_bodyTarget, real _deltaVelocity, CollisionData *_data, real _eventCoolTime)
		{
			this->bodyTarget = _bodyTarget;
			this->deltaVelocity = _deltaVelocity;
			this->data = _data;
			this->eventCoolTime = _eventCoolTime;
		}

		inline void execute(real duration)
		{
			collideBody = NULL;
			if(eventCoolTime <= coolTimer)
			{
				collideBody = CollisionDetector::getBodyCollisionWith(
					bodyTarget, deltaVelocity, data);
				if(collideBody == NULL)
					return;
				coolTimer = 0;
			}
			else
				coolTimer += duration;
		}
	};
	
	
	class CollisionWithArray 
	//getBodyArrayCollisionWith
	{
		friend class RPWorld;
	private:
		typedef CollisionWithArray type;
		RigidBody *bodyTarget;
		int limit; //max array size
	private:
		real eventCoolTime;
		real coolTimer;
		real deltaVelocity;
		CollisionData *data;
	private:
		RigidBody **pCollideBodyArray;
		int size;
	public:
		explicit CollisionWithArray()
			:
		bodyTarget(NULL), deltaVelocity(1), limit(0), 
		pCollideBodyArray(NULL), size(0), data(NULL),
		eventCoolTime(0), coolTimer(0)
		{

		}
		virtual ~CollisionWithArray()
		{
			delete[] pCollideBodyArray;
		}

		RigidBody **getBodyArray() { return pCollideBodyArray; }
		int getArraySize() { return size; }
		void getBodyArrayData(RigidBody *rigidBody[], int *size)
		{
			rigidBody = pCollideBodyArray;
			*size = this->size;
		}
		real getCoolTime() { return eventCoolTime; }
	protected:
		inline void set(RigidBody *_bodyTarget, real _deltaVelocity, 
			int _arrayLimit, CollisionData *_data, real _eventCoolTime)
		{
			if(pCollideBodyArray != NULL)
			{
				delete[] pCollideBodyArray;
				pCollideBodyArray = NULL; //warning
			}
			this->bodyTarget = _bodyTarget;
			this->deltaVelocity = _deltaVelocity;
			this->limit = _arrayLimit;
			this->data = _data;
			this->eventCoolTime = _eventCoolTime;
			pCollideBodyArray = new RigidBody*[limit];
			size = 0;
		}

		inline void execute(real duration)
		{
			for(int i = 0; i < limit; i++)
				pCollideBodyArray[i] = NULL;
			size = 0;
			if(eventCoolTime <= coolTimer)
			{
				CollisionDetector::getBodyArrayCollisionWith(
					bodyTarget, deltaVelocity,
					limit, pCollideBodyArray, &size, data);
				if(pCollideBodyArray[0] == NULL)
					return;
				coolTimer = 0;
			}
			else
				coolTimer += duration;
		}
	};
}

#endif
