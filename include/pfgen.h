#pragma once

#include "particle.h"
#include <vector>

/*
* A force generator can be be asked to add a force to one or more
* particles.
*/
class ParticleForceGenerator
{
public:
	/*
	* Overload this in implementations of the interface to calculate
	* and update the force applied to the given particle.
	*/
	virtual void updateForce(Particle* particle, real duration) = 0;

	
};


/*
* Holds all the force generators and the particles that they apply to.
*/
class ParticleForceRegistry
{
protected:

	/*
	* Keeps track of one force generator and the particle it applies to.
	*/
	struct ParticleForceRegistration
	{
		Particle* particle;
		ParticleForceGenerator* fg;
	};

public:
	/*
	* Holds the list of registrations.
	*/
	typedef std::vector<ParticleForceRegistration> Registry;
	Registry registrations;

	/*
	* Registers the given force generator to apply to the 
	* given particle.
	*/
	void add(Particle* particle, ParticleForceGenerator* fg);

	/*
	* Removes the given registered pair from the rgistry.
	* If the pair is not registered, this method will have
	* no effect.
	*/
	void remove(Particle* particle, ParticleForceGenerator* fg);

	/*
	* Clears all registrations from the registry. This will
	* not delete the particle or the force generators
	* themselves. Just the records of their connections.
	*/
	void clear();

	/*
	* Calls all the force generators to updatw the forces
	* of their corresponding particles.
	*/
	void updateForces(real duration);
};

/*
* A force generator thet applies a gravitational force. One instance
* can be used for mutiple particles.
*/
class ParticleGravity : public ParticleForceGenerator
{
	/*
	* Holds the acceleration due to gravity.
	*/
	Vector3 gravity;

public :

	/*
	* Creates the generator woth the given acceleration.
	*/
	ParticleGravity(const Vector3& gravity);

	/*
	* Applies the gravitational force to the given particle.
	*/
	virtual void updateForce(Particle* particle, real duration);
};

/*
* A force generator that applies a drag force. One instance
* can be used for multiple particles.
*/
class ParticleDrag : public ParticleForceGenerator
{

	//Holds the velocity drag coefficient.
	real k1;

	//Holds the velocity sqaured drag coefficient.
	real k2;

public:

	//Creates the generator with the given coefficients.
	ParticleDrag(real k1, real k2);

	//Applies the drag force to the given particle.
	virtual void updateForce(Particle* particle, real duration);
};

/*
* A force generator that applies a spring force.
*/
class ParticleSpring : public ParticleForceGenerator
{
	//The particle at the other end of the spring.
	Particle* other;

	//Holds the springconstant.
	real springConstant;

	//Holds the reslenght of the spring.
	real restLength;

public:

	//Creates a new spring with the given parameters.
	ParticleSpring(Particle* other, real springConstant, real restLength);

	//Applies the spring force to the given particle.
	virtual void updateForce(Particle* particle, real duration);
};

/*
* A force generator that applies a spring force, where
* one end it attached to a fixed in space.
*/
class ParticleAnchoredSpring : public ParticleForceGenerator
{
protected:

	/*
	* The location of the anchored end of the spring.
	*/
	Vector3* anchor;

	/*
	* Holds the springconstant.
	*/
	real springConstant;

	/*
	* Holds the rest length.
	*/
	real restLenght;

public:
	/*
	* Creates a new spring with the given paramaeters.
	*/
	ParticleAnchoredSpring(Vector3* anchor, real springConstant, real restLength);

	/*
	* Applies the spring force to the given particle.
	*/
	virtual void updateForce(Particle* particle, real duration);
};

class ParticleBungee : public ParticleForceGenerator
{
	/*
	* The particle at the other end of the of the spring.
	*/
	Particle* other;

	/*
	* Holds the spring constant.
	*/
	real springConstant;

	/*
	* Holds the length of the bungee at the it begins to
	* generate a force.
	*/
	real restLength;

public:

	/*
	* Creates a new bungee with the given parameters.
	*/
	ParticleBungee(Particle* other, real springConstant, real restLength);

	/*
	* Applies the spring force to the given particle.
	*/
	virtual void updateForce(Particle* particle ,real duration);
};

/*
* A force generator thet applies a buoyancy force for a plane of
* liquid parallel to XZ plane.
*/
class ParticleBuoyancy : public ParticleForceGenerator
{
	/*
	* The maximum submersion depth of the object before
	* it generatges its maximum buoyancy force.
	*/
	real maxDepth;

	/*
	* The volume of the object.
	*/
	real volume;

	/*
	* The height of the water plane above y = 0. The plane will be parallel to the XZ plane.
	*/
	real waterHeight;

	/*
	* The destiny of the liquid. Pure water has a destiny of
	* 1000kg per cubic meter.
	*/
	real liquidDensity;

public:

	/*
	* Creates a new buoyancy force with the given prameters.
	*/
	ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDesnsity = 1000.0f);

	/*
	* Applies the buoyancy force to the given particle.
	*/ 
	virtual void updateForce(Particle* particle, real duration);
};

/*
* A forcegenerator that fakes a stiff Spring force, and where 
* one end is attached to a fixed point in space.
*/
class ParticleFakeSpring : public ParticleForceGenerator
{
	/*
	* The location of the anchored end of the spring.
	*/
	Vector3* anchor;

	/*
	* Holds the spring constant.
	*/
	real springConstant;

	/*
	* Holds the damping on tje oscillation of the spring.
	*/
	real damping;

public:

	/*
	* Creates a new spring with the given paramaeters.
	*/
	ParticleFakeSpring(Vector3* anchor, real springConstant, real damping);

	/*
	* Applies the spring force to the given particle.
	*/
	virtual void updateForce(Particle* particle, real duration);

};

class UpLiftForceGenerator : public ParticleForceGenerator
{
public:
	//Origin point of the uplift calculation
	Vector3 origin;

	real distance;

public:

	UpLiftForceGenerator(Vector3 origin, real distance);

	virtual void updateForce(Particle* particle, real duration);
};

/*
* If the airbrake condition is true provides a large
* number of drag.
*/
class AirBrakeForceGenerator
{
public:
	bool airbrake;

	real k1;

	real k2;

public:
	AirBrakeForceGenerator(real k1, real k2);

	virtual void updateForce(Particle* particle, real duration);
};

/*
* Creates a gravitational pull to a chosen point
* or creates a planetary gravitation.
*/
class GravityToChosenPoint : public ParticleForceGenerator
{
public:
	//The point where the the grvitional acceleration points to.
	Vector3 attractionPoint;

public:

	GravityToChosenPoint(Vector3 attaractionPoint);

	void updateForce(Particle* particle, real duration);
};


class MaximumSpringForceGenerator : public ParticleForceGenerator
{
public:
	real springConstant;

	real restLength;

	Vector3 forceDirection;

	Vector3* anchor;

	real maximumDistance;


public:

	MaximumSpringForceGenerator(real restLength, real springConstant, real maximumDistance);

	virtual void updateForce(Particle* particle, real duaration);
};

class LighterThanAirForceGenerator : public ParticleForceGenerator
{
public:
	real airDensity;

	real maxDepth;

	real depth;

	real volume;

	real diminishFactor;

	real waterHeigth;

	real time;

public:
	LighterThanAirForceGenerator(real vloume, real waterHeigth, real maxDepth, real airDensity);

	virtual void updateForce(Particle* particle, real duration);
};

class OverCrowdingForceGenerator : ParticleForceGenerator
{
public:

	Vector3 center;

	int maxNumber;

	real distanceFromCenter;

	real distance;

	real springConstant;

	real restLength;

	Particle* particles[30];

	int numberOfApplies;

public:

	OverCrowdingForceGenerator(real distance, real springConstant, real restLength, int maxNumber);

	virtual void updateForce(Particle* particle, real duration);
};

class HomingBulletForceGenerator : public ParticleForceGenerator
{
public:
	Particle* target;

	real distance;

	real springConstant;

	real restLength;

public:

	HomingBulletForceGenerator(real distance, real restLength, real springConstant, Particle* target);

	virtual void updateForce(Particle* particle, real duration);
};