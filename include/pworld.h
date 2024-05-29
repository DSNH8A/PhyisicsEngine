#pragma once
#include <vector>
#include "particle.h"
#include "pcontancts.h"
#include "pfgen.h"

/*
* Keeps track of a set pf particles, and provides the means to
* update them all.
*/
class ParticleWorld
{
public:
	typedef std::vector<Particle*>Particles;

protected:
	/*
	* Holds the particles.
	*/
	Particles particles;

public:

	typedef std::vector<ParticleContactGenerator*> ContactGenerators;

	/*
	* Holds the force generators for the particles inthis world.
	*/
	ParticleForceRegistry registry;

	/*
	* Holds the resolver for contacts.
	*/
	ParticleContactResolver resolver;

	/*
	* Contact generators.
	*/
	ContactGenerators contactGenerators;

	/*
	* Holds the list of contacts.
	*/
	ParticleContact* contacts;


	/*
	* Holds the maximum number of contacts allowed (i. e., the
	* size of the contacts array).
	*/
	unsigned maxContacts;

	/*
	* True if the world should calculate the number of iterations
	* to give the contact resolver at each frame.
	*/
	bool calculateIterations;

public:
	/*
	* Creates a new particle simulator that can handle up to the
	* given number of contacts per frame. You can also optionally
	* give a number of conatac-resolutioon iterations to use. If you
	* dont give a number of iterations, tthen twice the number of
	* contacts will be used.
	*/
	ParticleWorld(unsigned maxContacts, unsigned iterations = 0);


	/*
	* Initialized the world for simulation frame. This clears
	* the force accumulators for particles in the world. After
	* calling this, particles can have their forces for this
	* frame added.
	*/
	void startFrame();

	/*
	* Deletes the simulator.
	*/
	~ParticleWorld();

	/*
	* Calls each of the registered contact generators to report
	* their contacts. Retuirns the mnumber of generated contacts.
	*/
	unsigned generateContacts();

	/*
	* Integrates all the particles in this world forward in time
	* by the given duration.
	*/
	void integrate(real duration);

	/*
	* Processes all the physics for the particle world.
	*/
	void runPhysics(real duration);

	/*
	*  Returns the list of particles.
	*/
	Particles& getParticles();

	/*
	* Returns the list of contact generators.
	*/
	ContactGenerators& getContactGenerators();

	/*
	* Returns the force registry.
	*/
	ParticleForceRegistry& getForceRegistry();
};

/**
	  * A contact generator that takes an STL vector of particle pointers and
	 * collides them against the ground.
	 */
class GroundContacts : public ParticleContactGenerator
{
	ParticleWorld::Particles* particles;

public:
	void init(ParticleWorld::Particles* particles);

	virtual unsigned addContact(ParticleContact* contact,unsigned limit) const;
};