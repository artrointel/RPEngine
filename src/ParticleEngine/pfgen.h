/*
	particle force generator.h
*/
#ifndef _PFGEN_H
#define _PFGEN_H

#include<vector>
#include "particle.h"

namespace RPEngine
{
	class ParticleForceGenerator
	{
	public:
		virtual void updateForce(Particle *particle, real duration) = 0;
	};


	class ParticleForceRegistry
	{
	protected:
		struct ParticleForceGeneration
		{
			Particle *particle;
			ParticleForceGenerator *fg;
		};
		typedef std::vector<ParticleForceGeneration> Registry;

		Registry registrations;

	public:	//stl vector wrapper
		void add(Particle *particle, ParticleForceGenerator *fg);
		void remove(Particle *particle, ParticleForceGenerator *fg);
		void clear();

	public:
		void updateForces(real duration);
	};


	/* implements force */

	class ParticleGravity : public ParticleForceGenerator
	{
	private:
		Vector3 gravity;
	public:
		ParticleGravity(const Vector3 &_gravity) : gravity(_gravity) {}

		virtual void updateForce(Particle *particle, real duration);
	};

	class ParticleRainDrop : public ParticleForceGenerator
	{
	private:
		real cloud; //check only y axis
		real ground; //check only y axis
	public:
		ParticleRainDrop(real ground_y, real cloud_y)
			:
		ground(ground_y), cloud(cloud_y)
		{
			if(ground > cloud)
			{
				real temp = cloud;
				cloud = ground;
				ground = temp;
			}
		}

		virtual void updateForce(Particle *particle, real duration);
	};

	class ParticleDrag : public ParticleForceGenerator
	{
	private:
		real k1; // drag force
		real k2;
	public:
		ParticleDrag(real _k1, real _k2) : k1(_k1), k2(_k2) {} 

		virtual void updateForce(Particle *particle, real duration);

	};

	class ParticleSpring : public ParticleForceGenerator
	{//should apply to the other Particle
	private:
		Particle *other; // the other end of the spring
		real springConstant; // k
		real restLength; // meaning maximum length
	public:
		ParticleSpring(Particle *_other, 
			real _springConstant, 
			real _restLength) 
			: other(_other), 
			springConstant(_springConstant), 
			restLength(_restLength) {}

		virtual void updateForce(Particle *particle, real duration);
	};

	class ParticleAnchoredSpring : public ParticleForceGenerator
	{
	private:
		Vector3 *anchor;
		real springConstant;
		real restLength;

	public:
		ParticleAnchoredSpring(Vector3 *_anchor, 
			real _springConstant, 
			real _restLength) 
			: anchor(_anchor), 
			springConstant(_springConstant), 
			restLength(_restLength) {}

		virtual void updateForce(Particle *particle, real duration);
	};

	class ParticleBungee : public ParticleForceGenerator
	{
	private:
		Particle *other;
		real springConstant;
		real restLength;

	public:
		ParticleBungee(Particle *_other, 
			real _springConstant,
			real _restLength)
			: other(_other),
			springConstant(_springConstant),
			restLength(_restLength) {}
		virtual void updateForce(Particle *particle, real duration);

	};

	class ParticleBuoyancy : public ParticleForceGenerator
	{
	private:
		real maxDepth; //submersion depth
		real volume;
		real waterHeight; // above y=0
		real liquidDensity; // water : 1000 kg / m^3
	public:
		ParticleBuoyancy(real _maxDepth, real _volume, 
			real _waterHeight, real _liquidDensity = 1000.0f)
			: maxDepth(_maxDepth), volume(_volume), 
			waterHeight(_waterHeight), liquidDensity(_liquidDensity) {};

		virtual void updateForce(Particle *particle, real duration);
	};


}

#endif
