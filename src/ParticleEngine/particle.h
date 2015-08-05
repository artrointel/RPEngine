
#ifndef _RP_ENGINE_PARTICLE_H
#define _RP_ENGINE_PARTICLE_H
#include "../Base/vector.h"
namespace RPEngine
{
	class Particle
	{
	protected:
		Vector3 position;
		Vector3 velocity;
		Vector3 acceleration;
		real inverseMass;
		real damping;
		
	protected: //d'alembert's priciple
		Vector3 forceAccum;

	public:
		explicit Particle() : position(nullVect), velocity(nullVect), acceleration(nullVect), forceAccum(nullVect), 
								inverseMass(1), damping(0.99f){};
		explicit Particle(Vector3 &position, Vector3 &velocity, Vector3 &acceleration, real inverseMass, real damping)
		{
			this->position = position;
			this->velocity = velocity;
			this->acceleration = acceleration;
			this->inverseMass = inverseMass;
			this->damping = damping;
		}
		virtual ~Particle(){};
	public:
		void integrate(real duration);
		void addForce(Vector3 &force);
		void addForce(real x, real y, real z);
		void clearAccumulator();

	public:/* get/set */
		//position
		void setPosition(Vector3 &position);
		void setPosition(real x, real y, real z);
		Vector3 &getPosition(); //const
		void getPosition(Vector3 *position) const; // refactoring required; to static methods

		//velocity
		void setVelocity(Vector3 &velocity);
		void setVelocity(real x, real y, real z);
		Vector3 &getVelocity(); //const
		void getVelocity(Vector3 *velocity) const; // refactoring required

		//accel
		void setAcceleration(Vector3 &acceleration);
		void setAcceleration(real x, real y, real z);
		const Vector3 &getAcceleration();
		
		//mass
		void setInverseMass(real inverseMass);
		void setMass(real mass);
		real getInverseMass() const;
		real getMass() const;
		bool hasFiniteMass() const;

		//damping
		void setDamping(real damping);
		real getDamping() const;
		
		//get/set force?

	public://static functions
		//pos_cpy, vel_cpy, acc_cpy
	};
}
#endif
