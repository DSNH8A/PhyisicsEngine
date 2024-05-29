#pragma once

#include "core.h"

/*
* A particle is the simplest object that can be simulated in the
* physics system.
*/
class Particle
{
public:

	/*
	*Holds the linear position of the particle in
	* world space.
	*/
	Vector3 position;

	/*
	* Holds the linear veclocity of the particle in world space.
	*/

	Vector3 velocity;

	/*
	* Holds the acceleration of the particle. This value can
	* be used to set acceleration due ti gravity (its primary
	* purpose), or any other constant acceleration.
	*/
	Vector3 acceleration;

	/*
	* Holds the amount of damping applied ti linear motion.
	* Damping is required to remove energy added through numerical
	* instability in the integrator.
	*/
	real damping;

	/*
	* Holds the inverse mass of the particle. Is is more useful
	* to hold the inverse because integration is simpler,
	* ans becasue in real-time simulation it is more useful 
	* to have objects with infinite(immovable) than zero mass
	* (completely unstable in nuerical simulation).
	*/
	real inverseMass;

	/*
	* Holds the accumulated forced to be applied at the next
	* simulation iteration only. This value is zerod at each
	* integration step.
	*/
	Vector3 forceAccum;

	/*
	* Ckears the forces applied to the particle. This will
	* be called automatically after each integration step.
	*/
	void clearAccumulator();

	/*
	* Integrates the particle forward in time by the given amount.
	* This function uses a Newton-Euler integration method, which is a
	* linear approximation to the correct integral. For this reason it
	* may be inaccurate is some cases.
	*/
	void integrate(real duration);

	/*
	* Adds the given force to the particle to be applied at the 
	* next iteration only.
	*/
	void addForce(const Vector3 force);

	bool hasFiniteMass() const;

	real getMass() const;

	void setMass(const real mass);

	void setInverseMass(const real mass);

	real getInverseMass() const;

	void getVelocity(Vector3* velocity) const;

	Vector3 getVelocity() const;

	void setVelocity(const Vector3 velocity);

	void getPosition(Vector3* position) const;

	Vector3 getPosition() const;

	Vector3 getPosition(const Vector3& position);

	void setPosition(const Vector3& position);
};