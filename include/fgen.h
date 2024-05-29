#pragma once

#include "core.h"
#include <vector>

/*
* A force generator can be asked to add a force to one or more
* bodies.
*/
class ForceGenerator
{
public:

	/*
	* Overload this in implementations of the interface to calculate
	* and update the force applied to the given rigid body.
	*/
	virtual void updateForce(RigidBody* body, real duration) = 0;
};

class Gravity : public ForceGenerator
{
	//Holds the acceleration due to gravity.
	Vector3 gravity;

public:

	//Creates the generator with the given acceleration.
	Gravity(const Vector3& gravity);

	//Applies the gravitational force to the given rigid body.
	virtual void updateForce(RigidBody* body, real duratuion);
};

/*
* A force generator that applies a spring force.
*/
class Spring : public ForceGenerator
{
	/*
	* the point of connection of the spring in local coordinates.
	*/
	Vector3 connectionPoint;

	/*
	* The point of connection of the spring to the other object
	* in that object's loacl coordinates.
	*/
	Vector3 otherConnectionPoint;

	/*
	* the particle at the ither end of the spring.
	*/
	RigidBody* other;

	/*
	* Hoolds the spring constant.
	*/
	real springConstant;

	/*
	* Holds the rest length of the spring.
	*/
	real restLength;

public:

	/*
	* Creates a new spring with the fiven parameters.
	*/
	Spring(const Vector3& localConnectionPoint, RigidBody* other, const Vector3& otherConnectionPoint, real springConstant, real resLength);

	/*
	* Applies the spring force to the given rigid body.
	*/
	virtual void updateForce(RigidBody* body, real duration);
};


/*
* Keeps an object at a given angular velocity.
* If the current rotation is far from the target
* applies a large toque. If the object is near the
* target velocity the torque drops.
*/
class TorqueGenerator : public ForceGenerator
{
	Vector3 targetVelocity;

	Vector3 force;

	Vector3 pointOnBody;

	TorqueGenerator(Vector3 targetVelocity, Vector3 force, Vector3 pointOnBody);

	virtual void updateForce(RigidBody* body, real duration);
};

/*
* Rotates its object to a specific orientation.  
*/
class TorqueGeneratorOrientation : public ForceGenerator
{
	Quaternion targetOrientation;

	Vector3 force;

	Vector3 pointOnBody;

	TorqueGeneratorOrientation(Quaternion targetOrientation, Vector3 force, Vector3 pointOnBody);

	virtual void updateForce(RigidBody* body, real duration);
};

class PropulsionGenerator : public ForceGenerator
{
public:

	Vector3 force;

public:

	PropulsionGenerator(Vector3 force);

	virtual void updateForce(RigidBody* body, real duration);
};

class Engine : public ForceGenerator
{
public :

	bool isDown;

	Vector3 downword = Vector3(0, -1, 0);

	Vector3 forward = Vector3(0, 0, 1);

public:

	Engine();

	virtual void updateForce(RigidBody* body, real duration);
};

/*
* A force generator thet applies an aerodynamic force.
*/
class Aero : public ForceGenerator
{
protected:

	/*
	* Holds the aerodynamic tensor for the surface in body space.
	*/
	Matrix3 tensor;

	/*
	* Holds the relative position of the aerodynamic surface
	* in body coordinates.
	*/
	Vector3 position;

	/*
	* Holds a pointer to a vector containing the wind speed of the
	* environment. This is a easier than managing a speprate
	* wind speed vector per generator and having to update it
	* manually as the wind changes.
	*/
	const Vector3* windSpeed;

public:

	/*
	* Creates a new aerodynamic force generator with the
	* given paramaters.
	*/
	Aero(const Matrix3& tensor, const Vector3& position, const Vector3* windSpeed);

	/*
	* Applies the force to the given rigid body.
	*/
	virtual void updateForce(RigidBody* body, real duration);

protected:

	/*
	* Uses an explicit tensor matrix to update the force on
	* the given rigid body. This is exactly the same as for 
	* updateForce, except tha it takes an explicit tensor.
	*/
	void updateForceFromTensor(RigidBody* body, real duration, const Matrix3& tensor);
};

/*
* A force generator with a control aerodynamic surface. This requires
* three inertia tensors, for the two extremes and 'resting' position
* of the control surface. The latter tensor is the one inherited from
* the base class, while the two extremes are defined in this class.
*/
class AeroControl : public Aero
{
protected:

	/*
	* The aeroDynamic tensor for the surface when the control is at
	* its maximum valu.
	*/
	Matrix3 maxTensor;

	/*
	* The aerodynamic tensor for the surface when the control is at
	* its minimum value.
	*/
	Matrix3 minTensor;

	/*
	* The current position of the control for this surface. This
	* should range between -1(in which case the minTensor value
	* is used), through 0 (where the base-class tensor value is
	* used) to +1 (where the maxTensor value is used).
	*/
	real controlSetting;

private:

	/*
	* Calculates the final aerodynamic tensor for the current
	* control setting.
	*/
	Matrix3 getTensor();

public:

	/*
	* Creates a new aerodynamic control surface with the given
	* properties.
	*/
	AeroControl(const Matrix3& base, const Matrix3& min, const Matrix3& max, const Vector3& position, const Vector3* windSpeed);

	/*
	* sets the control position of this control. This should
	* range between -1 (in which case the minTensor value is
	* used), through 0 (where the base-class tensor value is 
	* used) to +1 (where maxTensor value is used). Values 
	* outside that range give indefined results.
	*/
	void setControl(real value);

	/*
	* Applies the force to the given rigid body.
	*/
	virtual void updateForce(RigidBody* body, real duration);
};

/*
* A force generator to apply a buoynacy force to a rigid body.
*/
class Buoyancy : public ForceGenerator
{
	/*
	* The maximum submersion depth of the object before
	* ite generates its maxim,um buoyncy force.
	*/
	real maxDepth;

	/*
	* The volume of the object.
	*/
	real volume;

	/*
	* The height of the water plane above y=0. The plane will
	* be parallel to the XZ plane.
	*/
	real waterHeight;

	/*
	* The density of the liquid. Pure water has a density of
	* 1000kg per cubic meter.
	*/
	real liquidDensity;

	/*
	* The center of buoyancy of hte rigid body, in body coordinates.
	*/
	Vector3 centerOfBuoyancy;

public:

	//Creates a new buoyancy forec with the given parameters.
	Buoyancy(const Vector3& cOfB, real maxDepth, real volume, real waterHeight, real liquidDensity = 1000.0f);

	/*
	* Applies the force to the given rigid body.
	*/
	//virtual void addForce(RigidBody* body, real duration);

	virtual void updateForce(RigidBody* body, real duration);
};

/*
* A force generator with an aerdynamic surface that can be
* reoriented relative to ith rigi body.
*/
class AngledAero : public Aero
{
	/*
	* Holds the orientation of the aerdynamic surface relative
	* to the rigid body to which it is attached.
	*/
	Quaternion orientation;

public:

	/*
	* Creates a new aerodynamic surface with the given properties.
	*/
	AngledAero(const Matrix3& tensor, const Vector3& position, const Vector3* windSpeed);

	/*
	* Sets the relative orientation of the aerodynamic surface
	* realtive to the rigid body that ii is attached to. Note
	* that this doesnt affect the point of connection of the surface
	* to the body.
	*/
	void setOrientation(const Quaternion& quat);

	/*
	* Applies the force to the given rigid body.
	*/
	virtual void updateForce(RigidBody* body, real duration);
};

/**
   * Holds all the force generators and the bodies they apply to.
   */
class ForceRegistry
{
public:

	/**
	* Keeps track of one force generator and the body it
	* applies to.
	*/
	struct ForceRegistration
	{
		RigidBody* body;
		ForceGenerator* fg;
	};

	/**
	* Holds the list of registrations.
	*/
	typedef std::vector<ForceRegistration> Registry;
	Registry registrations;

public:
	/*
	* Registers the given force generator to apply to the
	* given body.
	*/
	void add(RigidBody* body, ForceGenerator* fg);

	/*
	* Removes the given registered pair from the registry.
	* If the pair is not registered, this method will have
	* no effect.
	*/
	void remove(RigidBody* body, ForceGenerator* fg);

	/*
	* Clears all registrations from the registry. This will
	* not delete the bodies or the force generators
	* themselves, just the records of their connection.
	*/
	void clear();

	/*
	* Calls all the force generators to update the forces of
	* their corresponding bodies.
	*/
	void updateForces(real duration);
};