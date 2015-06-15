/*
	force generator.h
*/
#ifndef _FGEN_H
#define _FGEN_H

#include<vector>
#include "rbody.h"

namespace RPEngine
{
	class ForceGenerator
	{
	public:
		virtual void setPointMode(int mode) {}; /* MCS or WCS */
		virtual void updateForce(RigidBody *body, real duration) = 0;
	};

	class ForceRegistry
	{
	public:
		struct ForceGeneration
		{
			RigidBody *body;
			ForceGenerator *fg;
		};
		typedef std::vector<ForceGeneration> Registry;

		Registry registrations;

	public:	//stl vector wrapper
		void add(RigidBody *body, ForceGenerator *fg);
		void remove(RigidBody *body, ForceGenerator *fg);
		void clear();

	public:
		void updateForces(real duration);
	};
	

	/* implements force */

	class Gravity : public ForceGenerator
	{
	private:
		Vector3 gravity;
	public:
		Gravity(const Vector3 &_gravity) : gravity(_gravity) {}

		virtual void updateForce(RigidBody *body, real duration);
	};
	
	class SimpleDrag : public ForceGenerator
	{
	private:
		real k1; // drag coefficient ^1
		real k2; // drag coefficient ^2
	public:
		SimpleDrag(real _k1, real _k2) : k1(_k1), k2(_k2) {} 
		void setDrag(real _k1, real _k2) { k1 = _k1; k2 = _k2; }
		virtual void updateForce(RigidBody *body, real duration);

	};
	
	class Spring : public ForceGenerator
	{//should apply to the other Particle
	private:
		Vector3 connectionPoint; // point of connection of the spring in MSC
		Vector3 otherConnectionPoint; // connection point to the other object

		RigidBody *other; // the other end of the spring
		real springConstant; // k
		real restLength; //spring length
	public:
		Spring(const Vector3 &localConnectionPt, 
			RigidBody *_other, 
			const Vector3 &otherConnectionPt,
			real _springConstant, 
			real _restLength) 
			: 
			connectionPoint(localConnectionPt),
			other(_other), 
			otherConnectionPoint(otherConnectionPt),
			springConstant(_springConstant), 
			restLength(_restLength) {}

		virtual void updateForce(RigidBody *body, real duration);
	};

	class HardSpring : public ForceGenerator
	{//should apply to the other Particle
	private:
		Vector3 connectionPoint; // point of connection of the spring in MSC
		Vector3 otherConnectionPoint; // connection point to the other object

		RigidBody *other; // the other end of the spring
		real springConstant; // joint power
		real linearCoeff;
		real restLength; // spring length
		real destroyLength; // limit Length
	public:
		bool isDestroy;
	public:
		HardSpring(const Vector3 &localConnectionPt, 
			RigidBody *_other, 
			const Vector3 &otherConnectionPt,
			real _springConstant, 
			real _linearCoeff,
			real _restLength,
			real _destroyLength) 
			: 
		connectionPoint(localConnectionPt),
			other(_other), 
			otherConnectionPoint(otherConnectionPt),
			springConstant(_springConstant), 
			linearCoeff(_linearCoeff),
			restLength(_restLength),
			destroyLength(_destroyLength),
			isDestroy(false)
		{}

		virtual void updateForce(RigidBody *body, real duration);
	};

	class Aero : ForceGenerator
	{
	protected:
		Matrix3 tensor;
		Vector3 position;
		const Vector3 *windspeed;
	public:
		Aero(const Matrix3 &tensor, const Vector3 &position,
			const Vector3 *windspeed);
		virtual void updateForce(RigidBody *body, real duration);
	protected:
		//use explicit tensor
		void updateForceFromTensor(RigidBody *body, real duration, const Matrix3 &tensor);
	};

	class AeroControl : public Aero
	{
	protected:
		Matrix3 maxTensor;
		Matrix3 minTensor;
		real controlSetting;
	private:
		Matrix3 getTensor();
	public:
		AeroControl(const Matrix3 &base,
					const Matrix3 &min, const Matrix3 &max, 
					const Vector3 &position, const Vector3 *windspeed);
		void setControl(real value);
		virtual void updateForce(RigidBody *body, real duration);
	};

	//timeout시 world의 자료구조에서 해당 force를 registry로부터 삭제되도록 구현 예정
	const char RP_MCS_DEFAULT = 0x00;
	const char RP_MCS_ANCHOR = 0x01;
	const char RP_WCS = 0x02;
	const char RP_TIME = 0x04;
	class CustomForce : public ForceGenerator // Engine user can use this customized force freely
	{
	public:
		Matrix3 tensor;
		Vector3 force;
		Vector3 point;
		real timeAccum; //accumulate duration
		real timeOut;
		void (CustomForce:: *pUpdateForce)(RigidBody *body, real duration);
	private:
		Vector3 anchoredForce;
	public:
		explicit CustomForce(const Matrix3 &_tensor,
			const Vector3 &_force,
			const Vector3 &_point)
			:
			tensor(_tensor), force(_force), 
			point(_point), 
			timeAccum(0.0f), timeOut(0.0f),
			anchoredForce(anchoredForce),
			pUpdateForce(&CustomForce::updateForceMCS)
			{}

		explicit CustomForce(const Vector3 &_force)
			:
			tensor(1.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 1.0f),
			force(_force), 
			point(originVect),
			timeAccum(0.0f), timeOut(0.0f),
			anchoredForce(anchoredForce),
			pUpdateForce(&CustomForce::updateForceMCS)
			{}

		virtual void setPointMode(char mode) //default is RP_MCS
		{
			if(RP_TIME != (mode & RP_TIME))
			{
				if(RP_WCS == (mode & RP_WCS)) //world
					pUpdateForce = &CustomForce::updateForceWCS;
				else if(RP_MCS_ANCHOR == (mode & RP_MCS_ANCHOR))
					pUpdateForce = &CustomForce::updateForceMCS_ANCHOR;
				else if(RP_MCS_DEFAULT == (mode & RP_MCS_DEFAULT)) //definitely 0
					pUpdateForce = &CustomForce::updateForceMCS;
			}
			else
			{
				if(RP_WCS == (mode & RP_WCS)) //world
					pUpdateForce = &CustomForce::updateForceWCS_timer;
				else if(RP_MCS_ANCHOR == (mode & RP_MCS_ANCHOR))
					pUpdateForce = &CustomForce::updateForceMCS_ANCHOR_timer;
				else if(RP_MCS_DEFAULT == (mode & RP_MCS_DEFAULT))
					pUpdateForce = &CustomForce::updateForceMCS_timer;
			}
		}

		virtual void updateForce(RigidBody *body, real duration);
	private:/* some customized update functions */

		void updateForceWCS(RigidBody *body, real duration)
		{
			body->addForceAtPoint(force, point);
		}

		void updateForceMCS(RigidBody *body, real duration)
		{
			body->addForceAtBodyPoint(force, point);
		}

		void updateForceMCS_ANCHOR(RigidBody *body, real duration)
		{
			_calculateAnchoredForce(body);
			body->addForceAtBodyPoint(anchoredForce, point);
		}

		//timer functions
		void updateForceWCS_timer(RigidBody *body, real duration)
		{
			timeAccum += duration;
			if(timeAccum >= timeOut) return; //refactoring required. -> world interface or delete this timeout
			body->addForceAtPoint(force, point);
		}
		
		void updateForceMCS_timer(RigidBody *body, real duration)
		{
			timeAccum += duration;
			if(timeAccum >= timeOut) return; //refactoring required. -> world interface or delete this timeout
			body->addForceAtBodyPoint(force, point);
		}

		void updateForceMCS_ANCHOR_timer(RigidBody *body, real duration)
		{
			timeAccum += duration;
			if(timeAccum >= timeOut) return;
			_calculateAnchoredForce(body);
			body->addForceAtBodyPoint(anchoredForce, point);
		}
	private:
		void _calculateAnchoredForce(RigidBody *body)
		{
			//set anchored point
			Matrix4 transform = body->getTransform();
			anchoredForce = Matrix4::worldToLocalDirn(force, transform);
		}
	};
}

#endif
