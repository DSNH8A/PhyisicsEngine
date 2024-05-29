
#define real_pow powf

#include "assert.h"
#include "particle.h"
#include "precision.h"

void Particle::integrate(real duration)
{
	//We dont integrate things with infinte mass.
	if (inverseMass <= 0.0f)
	{
		return;
	}

	assert(duration > 0.0, "Duration is fucked");

	//Update linear position.
	position.addScaledVector(velocity, duration);

	//work out the acceleration from the force.
	//(We will add to this vector when we come to generate forces.)
	Vector3 resultingAcc = acceleration;
	resultingAcc.addScaledVector(forceAccum, inverseMass);

	//Update linear velocity from acceleration.
	velocity.addScaledVector(resultingAcc, duration);

	//Impose drag.
	velocity *= real_pow(damping, duration);

	//Clear the forces.
	clearAccumulator();
}

void Particle::clearAccumulator()
{
	forceAccum.clear();
}

void Particle::addForce(const Vector3 force)
{
	forceAccum += force;
}

bool Particle::hasFiniteMass() const
{
	return inverseMass >= 0.0f;
}

real Particle::getMass() const
{
	if (inverseMass == 0) 
	{
		return REAL_MAX;
	}

	else 
	{
		return ((real)1.0) / inverseMass;
	}
}

void Particle::setMass(const real mass)
{
	assert(mass != 0);
	Particle::inverseMass = ((real)1.0) / mass;
}

void Particle::setInverseMass(const real inverseMass)
{
	Particle::inverseMass = inverseMass;
}

real Particle::getInverseMass() const
{
	return inverseMass;
}

void Particle::getVelocity(Vector3* velocity) const
{
	*velocity = Particle::velocity;
}

Vector3 Particle::getVelocity() const
{
	return velocity;
}

void Particle::setVelocity(const Vector3 velocity)
{
	Particle::velocity = velocity;
}

void Particle::getPosition(Vector3* position) const
{
	*position = Particle::position;
}

Vector3 Particle::getPosition() const
{
	return position;
}

Vector3 Particle::getPosition(const Vector3& vector)
{
	return Particle::position = vector;
}

void Particle::setPosition(const Vector3& vector)
{
	Particle::position = vector;
}