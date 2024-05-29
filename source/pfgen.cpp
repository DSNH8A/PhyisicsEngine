
#include "pfgen.h"
#define real_abs fabsf

/** Defines the precision of the sine operator.*/
#define real_sin sinf

/** Deinfes the precision of the cosine operator.*/
#define real_cos cosf

/** Defines the precision of the exponent operator.*/
#define real_exp expf
#define real_sqrt sqrtf

void ParticleForceRegistry::updateForces(real duration)
{
	Registry::iterator i = registrations.begin();

	for (; i != registrations.end(); i++)
	{
		i->fg->updateForce(i->particle, duration);
	}
}

ParticleGravity::ParticleGravity(const Vector3& gravity)
	: gravity(gravity)
	{}

void ParticleGravity::updateForce(Particle* particle, real duration)
{
	//Check if we have infinte mass.
	if (!particle->hasFiniteMass())
	{
		return;
	}

	//Apply the mass-scaled force to the particle.
	particle->addForce(gravity * particle->getMass());
}

ParticleDrag::ParticleDrag(real k1, real k2)
	: k1(k1), k2(k2)
	{}

void ParticleDrag::updateForce(Particle* particle, real duration)
{
	Vector3 force;

	particle->getVelocity(&force);

	//Calculate the total drag coefficient.
    real dragCoeff = force.magnitude();

	dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

	//Calculate the final force and apply it.
	force.normalize();
	force *= -dragCoeff;
	particle->addForce(force);
}

ParticleSpring::ParticleSpring(Particle* other, real springConstant, real restLength)
	: other(other), springConstant(springConstant), restLength(restLength)
	 {}


void ParticleSpring::updateForce(Particle* particle, real duaration)
{
	//Calculate the vector of the spring.
	Vector3 force;
	particle->getPosition(&force);
	force -= other->getPosition();

	//Calculate the magnitude of the force.
	real magnitude = force.magnitude();
	magnitude = real_abs(magnitude - restLength);
	magnitude *= springConstant;

	//Calculate the final force and apply it.
	force.normalize();
	force *= -magnitude;
	particle->addForce(force);
}

ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3* anchor, real springConstant, real restLength)
	: anchor(anchor), springConstant(springConstant), restLenght(restLength)
	{}

void ParticleAnchoredSpring::updateForce(Particle* particle, real duration)
{
	//Calculate the vector of the spring.
	Vector3 force;
	particle->getPosition(&force);
	force -= *anchor;

	//Calculates the magnitude of the force.
	real magnitude = force.magnitude();
	magnitude = (restLenght - magnitude) * springConstant;

	//Calculate the final force and apply it.
	force.normalize();
	force *= magnitude;
	particle->addForce(force);
}

ParticleBungee::ParticleBungee(Particle* other, real springConstant, real restLength)
	: other(other), springConstant(springConstant), restLength(restLength)
	{}

void ParticleBungee::updateForce(Particle* particle, real duration)
{
	//Calculate the vector of the spring.
	Vector3 force; 
	particle->getPosition(&force);
	force -= other->getPosition();

	//Check if the bungee is compressed.
	real magnitude = force.magnitude();
	if (magnitude <= restLength)
	{
		return;
	}

	//Calculate the magnitude of the force.
	magnitude = springConstant * (restLength - magnitude);

	//Calculate the final force and apply it.
	force.normalize();
	force *= -magnitude;
	particle->addForce(force);
}


ParticleBuoyancy::ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity)
	: maxDepth(maxDepth), volume(volume), waterHeight(waterHeight)
	{}

void ParticleBuoyancy::updateForce(Particle* particle, real duration)
{
	//Calculate the submersion depth.
	real depth = particle->getPosition().y;

	//Check if we are out of the water.
	if (depth >= waterHeight + maxDepth)
	{
		return;
	}

	Vector3 force(0, 0, 0);

	//Check if we are at maximum depth.
	if (depth <= waterHeight - maxDepth)
	{
		force.y = liquidDensity * volume;
		particle->addForce(force);
		return;
	}

	//Ohterwise we are partly submerged.
	force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / 2 * maxDepth;
	particle->addForce(force);
}

ParticleFakeSpring::ParticleFakeSpring(Vector3* anchor, real springConstant, real damping)
	: anchor(anchor), springConstant(springConstant), damping(damping)
	 {}

void ParticleFakeSpring::updateForce(Particle* particle, real duration)
{
	//Check if we do not have infinite mass.
	if (!particle->hasFiniteMass())
	{
		return;
	}

	//Calculate the relative position of the particle to the anchor.
	Vector3 position;
	particle->getPosition(&position);
	position -= *anchor;

	//Calculate the constants and check that they are in bounds.
	real gamma = 0.5f * real_sqrt(4 * springConstant - damping * damping);

	if (gamma = 0.0f)
	{
		return;
	}

	Vector3 c = position * (damping / (2.0f * gamma)) + particle->getVelocity() * (1.0f / gamma);

	//Calculate the target position.
	Vector3 target = position * real_cos(gamma * duration) + c * real_sin(gamma * duration);
	target *= real_exp(-0.5f * duration * damping);

	//Calculates the resulting acceleration, and therfore the force.
	Vector3 accel = (target - position) * (1.0f / duration * duration) - particle->getVelocity() * duration;
	particle->addForce(accel * particle->getMass());
}

UpLiftForceGenerator::UpLiftForceGenerator(Vector3 ori, real dis)
    : origin(ori), distance(dis)
{

}

void UpLiftForceGenerator::updateForce(Particle* particle, real duration)
{
    if (abs(particle->getPosition().x - origin.x) > distance || abs(particle->getPosition().z - origin.z) > distance)
    {
        Vector3 goal = Vector3(origin + Vector3(0, particle->getPosition().y, 0));

        Vector3 force = Vector3(0, -100, 0);//goal - particle->getPosition();
        force.normalize();
        force *= force.magnitude();
        particle->addForce(force);
    }
}

AirBrakeForceGenerator::AirBrakeForceGenerator(real k1, real k2)
    :k1(k1), k2(k2)
{
}

void AirBrakeForceGenerator::updateForce(Particle* particle, real duration)
{
    if (airbrake == false)
    {
        return;
    }

    if (airbrake == true)
    {
        Vector3 force;
        particle->getVelocity(&force);

        // Calculate the total drag coefficient
        real dragCoeff = force.magnitude();
        dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

        // Calculate the final force and apply it
        force.normalize();
        force *= -dragCoeff;
        particle->addForce(force);
    }
}

GravityToChosenPoint::GravityToChosenPoint(Vector3 attractionPoint)
    : attractionPoint(attractionPoint)
{
}

void GravityToChosenPoint::updateForce(Particle* particle, real duration)
{
    Vector3 force = attractionPoint - particle->getPosition();
    real magnitude = force.magnitude();
    magnitude = magnitude * magnitude;

    force.normalize();
    force *= magnitude;
    particle->addForce(force);

}

MaximumSpringForceGenerator::MaximumSpringForceGenerator(real restLength, real springConstant, real maximumDistance)
    : restLength(restLength), springConstant(springConstant), maximumDistance(maximumDistance)
{}


void MaximumSpringForceGenerator::updateForce(Particle* particle, real duration)
{
    Vector3 force;
    particle->getPosition(&force);
    Vector3 origin = Vector3(0, 0, 0);
    force -= origin;

    // Calculate the magnitude of the force
    real magnitude = force.magnitude();
    if (restLength + magnitude > maximumDistance)
    {
        springConstant = springConstant / 30;
    }

    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    // Calculate the final force and apply it
    force.normalize();
    force *= -magnitude;
    particle->addForce(force);
}

LighterThanAirForceGenerator::LighterThanAirForceGenerator(real volume, real waterHeigth, real maxDepth, real airDensity)
    : volume(volume), waterHeigth(waterHeigth), maxDepth(maxDepth), airDensity(1.3f)
{}

void LighterThanAirForceGenerator::updateForce(Particle* particle, real duration)
{
    // Calculate the submersion depth
    real depth = particle->getPosition().y;

    time += duration;

    // Check if we're out of the water
    if (depth >= waterHeigth + maxDepth) return;
    Vector3 force(0, 0, 0);

    // Check if we're at maximum depth
    if (depth <= waterHeigth - maxDepth)
    {
        force.y = airDensity * volume;
        particle->addForce(force);
        return;
    }

    // Otherwise we are partly submerged
    force.y = airDensity * volume *
        (depth - maxDepth - waterHeigth - duration) / 2 * maxDepth;

    volume += duration;
    if (volume >= airDensity)
    {
        return;
    }

    particle->addForce(force);
}

OverCrowdingForceGenerator::OverCrowdingForceGenerator(real distance, real springConstant, real resLength,
    int maxNumber) : distance(distance), springConstant(springConstant), restLength(restLength), maxNumber(maxNumber)
{}

void OverCrowdingForceGenerator::updateForce(Particle* particle, real duration)
{
    for (int i = 0; i < maxNumber; i++)
    {
        for (int j = 0; j < maxNumber; j++)
        {
            if (particles[i] == particles[j] || particles[j] == particle)
            {
                break;
            }
            if (particles[i]->getPosition().y > 0 && particles[j]->getPosition().y > 0)
            {
                if (abs((particles[i]->getPosition() - particles[j]->getPosition()).magnitude()) < distance)
                {
                    Vector3 force;
                    particle->getPosition(&force);
                    force -= particles[j]->getPosition();

                    real magnitude = force.magnitude();
                    magnitude = real_abs(magnitude - 0.02); //nem veszi a be az eredeti változót csak a varázsszámot
                    magnitude *= springConstant;

                    force.normalize();
                    force *= -magnitude;
                    Vector3 jForce = force * -magnitude;

                    particles[i]->addForce(force);
                    particles[j]->addForce(jForce);

                }
            }

            else
            {
                continue;
            }
        }
    }
}

HomingBulletForceGenerator::HomingBulletForceGenerator(real distance, real restLength, real springConstant, Particle* target)
    :distance(distance), restLength(restLength), springConstant(springConstant), target(target)
{}

void HomingBulletForceGenerator::updateForce(Particle* particle, real duration)
{
    /*target->getPosition().addScaledVector(target->getVelocity(), 1);
    particle->getPosition().addScaledVector(target->getVelocity(), 1);*/

    Vector3 sanyika = Vector3(0, 5, 0);

    Vector3 force;
    particle->getPosition(&force);
    force -= sanyika;

    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    force.normalize();
    force *= magnitude;
    // particle->addForce(force);
    particle->addForce(Vector3(0, 5, 0));
}