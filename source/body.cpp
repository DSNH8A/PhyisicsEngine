
#include "core.h"
#include "precision.h"
#include <assert.h>

#define real_pow powf

/**
 * Internal function that checks the validity of an inverse inertia tensor.
 */
static inline void _checkInverseInertiaTensor(const Matrix3& iitWorld)
{
	// TODO: Perform a validity check in an assert.
}


/*
* Inline function that creates a transform matrix from a
* position and orientation.
*/
static inline void _calculateTransformMatrix(Matrix4& transformMatrix, const Vector3& position, const Quaternion& orientation)
{
	transformMatrix.data[0] = 1 - 2 * orientation.j * orientation.j - 2 * orientation.k * orientation.k;
	transformMatrix.data[1] = 2 * orientation.i * orientation.j - 2 * orientation.r * orientation.k;
	transformMatrix.data[2] = 2 * orientation.i * orientation.k + 2 * orientation.r * orientation.j;
	transformMatrix.data[3] = position.x;

	transformMatrix.data[4] = 2 * orientation.i * orientation.j + 2 * orientation.r * orientation.k;
	transformMatrix.data[5] = 1 - 2 * orientation.i * orientation.i - 2 * orientation.k * orientation.k;
	transformMatrix.data[6] = 2 * orientation.j * orientation.k - 2 * orientation.r * orientation.i;
	transformMatrix.data[7] = position.y;

	transformMatrix.data[8] = 2 * orientation.i * orientation.k - 2 * orientation.r * orientation.j;
	transformMatrix.data[9] = 2 * orientation.j * orientation.k - 2 * orientation.r * orientation.j;
	transformMatrix.data[10] = 1 - 2 * orientation.i * orientation.i - 2 * orientation.j * orientation.j;
	transformMatrix.data[11] = position.z;
}

void RigidBody::calculateDerivedData()
{
	orientation.normalize();

	//Calcualte the transformMatrix for the body.
	_calculateTransformMatrix(transformMatrix, position, orientation);
}

void RigidBody::setInertiaTensor(const Matrix3& inertiaTensor)
{
	inverseInertiaTensor.setInverse(inertiaTensor);
}

/*
* Internal function to do an inertia tensor transform by a quaternion.
* Note that the implementation of this function was created by an
* automated code generator and optimizer.
*/
static inline void _transformInertiaTensor(Matrix3& itWorld, const Quaternion& q, const Matrix3& itBody, const Matrix4& rotmat)
{
	real t4 = rotmat.data[0] * itBody.data[0] +
		rotmat.data[1] * itBody.data[3] +
		rotmat.data[2] * itBody.data[6];

	real t9 = rotmat.data[0] * itBody.data[1] +
		rotmat.data[1] * itBody.data[4] +
		rotmat.data[2] * itBody.data[7];

	real t14 = rotmat.data[0] * itBody.data[2] +
		rotmat.data[1] * itBody.data[5] +
		rotmat.data[2] * itBody.data[8];

	real t28 = rotmat.data[4] * itBody.data[2] +
		rotmat.data[5] * itBody.data[3] +
		rotmat.data[6] * itBody.data[6];

	real t33 = rotmat.data[4] * itBody.data[1] +
		rotmat.data[5] * itBody.data[4] +
		rotmat.data[6] * itBody.data[7];

	real t38 = rotmat.data[5] * itBody.data[2] +
		rotmat.data[5] * itBody.data[5] +
		rotmat.data[6] * itBody.data[8];

	real t52 = rotmat.data[8] * itBody.data[0] +
		rotmat.data[9] * itBody.data[3] +
		rotmat.data[10] * itBody.data[6];

	real t57 = rotmat.data[8] * itBody.data[1] +
		rotmat.data[9] * itBody.data[4] +
		rotmat.data[10] * itBody.data[7];
	
	real t62 = rotmat.data[8] * itBody.data[2] +
		rotmat.data[9] * itBody.data[5] +
		rotmat.data[10] * itBody.data[8];



	itWorld.data[0] = t4 * rotmat.data[0] +
		t9 * rotmat.data[1] +
		t14 * rotmat.data[2];

	itWorld.data[1] = t4 * rotmat.data[4] +
		t9 * rotmat.data[5] +
		t14 * rotmat.data[6];

	itWorld.data[2] = t4 * rotmat.data[8] +
		t9 * rotmat.data[9] +
		t14 * rotmat.data[10];

	itWorld.data[3] = t28 * rotmat.data[0] +
		t33 * rotmat.data[1] +
		t38 * rotmat.data[2];

	itWorld.data[4] = t28 * rotmat.data[4] +
		t33 * rotmat.data[5] +
		t38 * rotmat.data[6];

	itWorld.data[5] = t28 * rotmat.data[8] +
		t33 * rotmat.data[9] +
		t38 * rotmat.data[10];

	itWorld.data[6] = t52 * rotmat.data[0] +
		t57 * rotmat.data[1] +
		t62 * rotmat.data[2];

	itWorld.data[7] = t52 * rotmat.data[4] +
		t57 * rotmat.data[5] +
		t62 * rotmat.data[6];

	itWorld.data[8] = t52 * rotmat.data[8] +
		t57 * rotmat.data[9] +
		t62 * rotmat.data[10];
}

//void RigidBody::calculateDerivedData()
//{
//	orientation.normalize();
//
//	//Calculate the transform matrix for the body.
//	_calculateTransformMatrix(transformMatrix, position, orientation);
//
//	//Calculate the inertiaTensor in world space.
//	_transformInertiaTensor(inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix);
//}

void RigidBody::addForce(const Vector3& force)
{
	forceAccum += force;
	isAwake = true;
}

void RigidBody::clearAccumulators()
{
	forceAccum.clear();
	torqueAccum.clear();
}

void RigidBody::integrate(real duration)
{
	//Calculate linear acceleration from force inputs.
	lastFrameAcceleration = acceleration;
	lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);

	//Calculate angular acceleration from torque inputs.
	Vector3 angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccum);

	//Adjust velocities.
	//Update linear velocity from both acceleration and impulse.
	velocity.addScaledVector(lastFrameAcceleration, duration);

	//Update angular velocity from both acceleration and impulse.
	rotation.addScaledVector(angularAcceleration, duration);

	//Impose drag.
	velocity *= real_pow(linearDamping, duration);
	rotation *= real_pow(angularDamping, duration);

	//Adjust positions.
	//Update linear position.
	position.addScaledVector(velocity, duration);

	//Update angular position.
	orientation.addScaledVector(rotation, duration);

	//Normalize the orientation, and update the matrices with the new
	//position and orientation.
	calculateDerivedData();

	//Clear accumulators.
	clearAccumulators();
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3& point) const
{
	return transformMatrix.transform(point);
}

void RigidBody::addForceToBodyPoint(const Vector3& force, const Vector3& point)
{
	//Convert to coordinates relative to center of mass.
	Vector3 pt = getPointInWorldSpace(point);
	addForceAtPoint(force, pt);

	isAwake = true;
}

void RigidBody::addForceAtPoint(const Vector3& force, const Vector3& point)
{
	//Convert to coordinates relative to center of mass.
	Vector3 pt = point;
	pt -= position;

	forceAccum += force;
	torqueAccum += pt % force;

	isAwake = true;
}

bool RigidBody::hasFiniteMass() const
{
	return inverseMass >= 0.0f;
}

void RigidBody::setMass(const real mass)
{
	assert(mass != 0);

	RigidBody::inverseMass = ((real)1.0) / mass;
}

real RigidBody::getMass() const
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

void RigidBody::setInverseMass(const real inverseMass)
{
	RigidBody::inverseMass = inverseMass;
}

real RigidBody::getInverseMass()
{
	return inverseMass;
}

Vector3 RigidBody::getVelocity() const
{
	return velocity;
}

void RigidBody::setVelocity(const Vector3& velocity)
{
	RigidBody::velocity = velocity;
}

void RigidBody::addVelocity(const Vector3& deltaVelocity)
{
	RigidBody::velocity += deltaVelocity;
}

void RigidBody::getGLTransform(float matrix[16]) const
{
	matrix[0] = (float)transformMatrix.data[0];
	matrix[1] = (float)transformMatrix.data[4];
	matrix[2] = (float)transformMatrix.data[8];
	matrix[3] = 0;

	matrix[4] = (float)transformMatrix.data[1];
	matrix[5] = (float)transformMatrix.data[5];
	matrix[6] = (float)transformMatrix.data[9];
	matrix[7] = 0;

	matrix[8] = (float)transformMatrix.data[2];
	matrix[9] = (float)transformMatrix.data[6];
	matrix[10] = (float)transformMatrix.data[10];
	matrix[11] = 0;

	matrix[12] = (float)transformMatrix.data[3];
	matrix[13] = (float)transformMatrix.data[7];
	matrix[14] = (float)transformMatrix.data[11];
	matrix[15] = 1;
}

Matrix4 RigidBody::getTransform()const
{
	return transformMatrix;
}

void RigidBody::setPosition(const Vector3& position)
{
	RigidBody::position = position;
}

Vector3 RigidBody::getPosition()const
{
	return position;
}

void RigidBody::getPosition(Vector3* position) const
{
	*position = RigidBody::position;
}

Vector3 RigidBody::getAcceleration() const 
{
	return RigidBody::acceleration;
}

void RigidBody::getAcceleration(Vector3* acceleration) const
{
	*acceleration = RigidBody::acceleration;
}

void RigidBody::setAcceleration(const Vector3& acceleration)
{
	RigidBody::acceleration = acceleration;
}

void RigidBody::setAcceleration(const real x, const real y, const real z)
{
	RigidBody::acceleration.x = x;
	RigidBody::acceleration.y = y;
	RigidBody::acceleration.z = z;
}

Vector3 RigidBody::getLastFrameAcceleration()
{
	return lastFrameAcceleration;
}

Quaternion RigidBody::getOrientation()const
{
	return RigidBody::orientation;
}

void RigidBody::getOrientation(Quaternion* orientation) const
{
	*orientation = RigidBody::orientation;
}

void RigidBody::setOrientation(real r, real i, real j, real k)
{
	RigidBody::orientation.r = r;
	RigidBody::orientation.i = i;
	RigidBody::orientation.j = j;
	RigidBody::orientation.k = k;
}

void RigidBody::setOrientation(Quaternion q)
{
	RigidBody::orientation.r = q.r;
	RigidBody::orientation.i = q.i;
	RigidBody::orientation.j = q.j;
	RigidBody::orientation.k = q.k;
}

Vector3 RigidBody::getRotation()const
{
	return RigidBody::rotation;
}

void RigidBody::getRotation(Vector3* rotation)const
{
	*rotation = RigidBody::rotation;
}

void RigidBody::setRotation(const Vector3& rotation)
{
	RigidBody::rotation = rotation;
}

void RigidBody::setRotation(const real x, const real y, const real z)
{
	RigidBody::rotation.x = x;
	RigidBody::rotation.y = y;
	RigidBody::rotation.z = z;
}
 
void RigidBody::addRotation(const Vector3& deltaRotation)
{
	RigidBody::rotation += deltaRotation;
}

//// > SetInertiaTensor
//void RigidBody::setInertiaTensor(const Matrix3& inertiaTensor)
//{
//	inverseInertiaTensor.setInverse(inertiaTensor);
//	// < SetInertiaTensor
//	_checkInverseInertiaTensor(inverseInertiaTensor);
//	// > SetInertiaTensor
//}
//// < SetInertiaTensor

void RigidBody::getInertiaTensor(Matrix3* inertiaTensor) const
{
	inertiaTensor->setInverse(inverseInertiaTensor);
}

Matrix3 RigidBody::getInertiaTensor() const
{
	Matrix3 it;
	getInertiaTensor(&it);
	return it;
}

void RigidBody::getInertiaTensorWorld(Matrix3* inertiaTensor) const
{
	inertiaTensor->setInverse(inverseInertiaTensorWorld);
}

Matrix3 RigidBody::getInertiaTensorWorld() const
{
	Matrix3 it;
	getInertiaTensorWorld(&it);
	return it;
}

void RigidBody::setInverseInertiaTensor(const Matrix3& inverseInertiaTensor)
{
	_checkInverseInertiaTensor(inverseInertiaTensor);
	RigidBody::inverseInertiaTensor = inverseInertiaTensor;
}

void RigidBody::getInverseInertiaTensor(Matrix3* inverseInertiaTensor) const
{
	*inverseInertiaTensor = RigidBody::inverseInertiaTensor;
}

Matrix3 RigidBody::getInverseInertiaTensor() const
{
	return inverseInertiaTensor;
}

void RigidBody::getInverseInertiaTensorWorld(Matrix3* inverseInertiaTensor) const
{
	*inverseInertiaTensor = inverseInertiaTensorWorld;
}

Matrix3 RigidBody::getInverseInertiaTensorWorld() const
{
	return inverseInertiaTensorWorld;
}

void RigidBody::setDmaping(const real lineraDamping, const real angularDamping)
{
	RigidBody::linearDamping = linearDamping;
	RigidBody::angularDamping = angularDamping;
}

void RigidBody::setLinearDamping(const real linearDamping)
{
	RigidBody::linearDamping = linearDamping;
}

real RigidBody::getLinearDamping() const
{
	return linearDamping;
}

void RigidBody::setAngularDamping(const real angularDamping)
{
	RigidBody::angularDamping = angularDamping;
}

real RigidBody::getAngularDamping() const
{
	return angularDamping;
}

void RigidBody::setAwake(const bool awake)
{
	if (awake) 
	{
		isAwake = true;

		// Add a bit of motion to avoid it falling asleep immediately.
		motion = sleepEpsilon * 2.0f;
	}
	else 
	{
		isAwake = false;
		velocity.clear();
		rotation.clear();
	}
}

void RigidBody::setCanSleep(const bool canSleep)
{
	RigidBody::canSleep = canSleep;

	if (!canSleep && !isAwake) setAwake();
}