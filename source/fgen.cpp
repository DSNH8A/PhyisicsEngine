

#include "fgen.h"
#include "precision.h"

void Gravity::updateForce(RigidBody* body, real duration)
{
	//Check that we do not have infinite mass.
	if (!body->hasFiniteMass())
	{
		return;
	}

	//Apply the mass-scaled force to the body.
	body->addForce(gravity * body->getMass());
}


void Spring::updateForce(RigidBody* body, real duration)
{
	//Calculate the two end in world space.
	Vector3 lws = body->getPointInWorldSpace(connectionPoint);
	Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);

	//Calculate the vector of the spring.
	Vector3 force = lws - ows;

	//Calculate the final force and apply it.
	real magnitude = force.magnitude();
	magnitude = real_abs(magnitude - restLength);
	magnitude *= springConstant;

	//Calculate the final force and apply it.
	force.normalize();
	force *= -magnitude;
	body->addForceAtPoint(force, lws);
}

TorqueGenerator::TorqueGenerator(Vector3 targetVelocity, Vector3 force, Vector3 pointOnBody)
	: targetVelocity(targetVelocity), force(force), pointOnBody(pointOnBody)
{}

void TorqueGenerator::updateForce(RigidBody* body, real duration)
{
	if (body->rotation.magnitude() < targetVelocity.magnitude())
	{
		body->addForceToBodyPoint(force, pointOnBody);
	}

	else
	{
		return;
	}
}

TorqueGeneratorOrientation::TorqueGeneratorOrientation(Quaternion targetOrientation, Vector3 force, Vector3 pointOnBody)
	: targetOrientation(targetOrientation), force(force), pointOnBody(pointOnBody) {}

void TorqueGeneratorOrientation::updateForce(RigidBody* body, real duration)
{

}

Aero::Aero(const Matrix3& tensor, const Vector3& position, const Vector3* windSpeed)
	: tensor(tensor), position(position), windSpeed(windSpeed)
{}

void Aero::updateForce(RigidBody* body, real duration)
{
	Aero::updateForceFromTensor(body, duration, tensor);
}

void Aero::updateForceFromTensor(RigidBody* body, real duration, const Matrix3& tensor)
{
	//Calculate total velocity (wind speed and bdies velocity).
	Vector3 velocity = body->getVelocity();
	velocity += *windSpeed;

	//Calculate the velocity in body coordinates.
	Vector3 bodyVel = body->getTransform().transformInverseDirection(velocity);

	//Calculate the force in body coordinates.
	Vector3 bodyForce = tensor.transform(bodyVel);
	Vector3 force = body->getTransform().transformDirection(bodyForce);

	//Apply the force.
	body->addForceToBodyPoint(force, position);
}

Matrix3 AeroControl::getTensor()
{
	if(controlSetting <= -1.0f)
	{
		return minTensor;
	}

	else if(controlSetting >= 1.0f)
	{
		return maxTensor;
	}

	else if (controlSetting < 0)
	{
		return Matrix3::linearInterpolate(minTensor, tensor, controlSetting + 1.0f);
	}

	else if (controlSetting > 0)
	{
		return Matrix3::linearInterpolate(tensor, maxTensor, controlSetting);
	}

	else
	{
		return tensor;
	}
}

void AeroControl::updateForce(RigidBody* body, real duration)
{
	Matrix3 tensor = getTensor();
	Aero::updateForceFromTensor(body, duration, tensor);
}