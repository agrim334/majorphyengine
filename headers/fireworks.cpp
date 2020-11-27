#include"particle.h"

class Firework : public wizphys::Particle {
public:
	unsigned type;
	wizphys::real age;
	bool update(wizphys::real duration)
	{
		integrate(duration);

		age -= duration;
		return (age < 0) || (position.y < 0);
	}

};

struct FireworkRule {
	unsigned type;

	wizphys::real minAge;

	wizphys::real maxAge;

	wizphys::Vector3D minVelocity;

	wizphys::Vector3D maxVelocity;

	wizphys::real damping;

	struct Payload {
		unsigned type;

		unsigned count;

		void set(unsigned type, unsigned count)
		{
			Payload::type = type;
			Payload::count = count;
		}
	};

	unsigned payloadCount;

	Payload *payloads;

	FireworkRule()
	:
	payloadCount(0),
	payloads(NULL)
	{
	}

	void init(unsigned payloadCount)
	{
		FireworkRule::payloadCount = payloadCount;
		payloads = new Payload[payloadCount];
	}

	~FireworkRule()
	{
		if (payloads != NULL) delete[] payloads;
	}

	/**
	 * Set all the rule parameters in one go.
	 */
	void setParameters(unsigned type, wizphys::real minAge, wizphys::real maxAge,
		const wizphys::Vector3D &minVelocity, const wizphys::Vector3D &maxVelocity,
		wizphys::real damping)
	{
		FireworkRule::type = type;
		FireworkRule::minAge = minAge;
		FireworkRule::maxAge = maxAge;
		FireworkRule::minVelocity = minVelocity;
		FireworkRule::maxVelocity = maxVelocity;
		FireworkRule::damping = damping;
	}

	/**
	 * Creates a new firework of this type and writes it into the given
	 * instance. The optional parent firework is used to base position
	 * and velocity on.
	 */
	void create(Firework *firework, const Firework *parent = NULL) const
	{

		firework->type = type;
		firework->age = crandom.randomReal(minAge, maxAge);
		wizphys::Vector3D vel;
		if (parent) {
			// The position and velocity are based on the parent.
			firework->setPosition(parent->getPosition());
			vel += parent->getVelocity();
		}
		else
		{
			wizphys::Vector3D start;
			int x = (int)crandom.randomInt(3) - 1;
			start.x = 5.0f * wizphys::real(x);
			firework->setPosition(start);
		}

		vel += crandom.randomVector(minVelocity, maxVelocity);
		firework->setVelocity(vel);

		// We use a mass of one in all cases (no point having fireworks
		// with different masses, since they are only under the influence
		// of gravity).
		firework->setMass(1);

		firework->setDamp(damping);
		wizphys::Vector3D gravity(0,-10,0);
		firework->setAccel(gravity);

		firework->clearAccumulator();
	}
};

int main(){
	return 0;
}