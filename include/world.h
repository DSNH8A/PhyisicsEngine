#pragma once

#include "core.h"
#include <vector>
#include "contacts.h"

/*
* The world represents an independent simulation of phydsics.
* It keeps treack of a set of rigid bodies, and provides the
* to updatte them all.
*/
class World
{
public:

	typedef std::vector<RigidBody>RigidBodies;

protected:

	/*
	* Holds the rigid bodies being simulated.
	*/
	RigidBodies bodies;


    /*
    * True if the world should calculate the number of iterations
    * to give the contact resolver at each frame.
    */
    bool calculateIterations;

    /*
    * Holds the resolver for sets of contacts.
    */
    ContactResolver resolver;

    struct BodyRegistration
    {
        RigidBody* body;
        BodyRegistration* next;
    };

    BodyRegistration* firstBody;

    /*
    * Holds one contact generators in a linked list.
    */
    struct ContactGenRegistration
    {
        ContactGenerator* gen;
        ContactGenRegistration* next;
    };

    /*
    * Holds the head of the list of contact generators.
    */
    ContactGenRegistration* firstContactGen;

    /*
    * Holds an array of contacts, for filling by the contact
    * generators.
    */
    Contact* contacts;

    /*
    * Holds the maximum number of contacts allowed (i.e. the size
    * of the contacts array).
    */
    unsigned maxContacts;

public:

    /*
    * Creates a new simulator that can handle up to the given
    * number of contacts per frame. You can also optionally give
    * a number of contact-resolution iterations to use. If you
    * don't give a number of iterations, then four times the
    * number of detected contacts will be used for each frame.
    */
    World(unsigned maxContacts, unsigned iterations = 0);
    ~World();

    /*
    * Calls each of the registered contact generators to report
    * their contacts. Returns the number of generated contacts.
    */
    unsigned generateContacts();

	/*
	* Initializes the world for simulation frame. This clears
	* the force and torque accumulators for boides in the world.
	* After calling this, the bodies can have their forces and
	* torquies for this frame added.
	*/
	void startFrame();

	/*
	* Processes all the physics for the world.
	*/
	void runPhysics(real duration);

	void integrate(real duration);
};
