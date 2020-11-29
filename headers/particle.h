#ifndef CYCLONE_PARTICLE_H
#define CYCLONE_PARTICLE_H

#include "core.h"

namespace cyclone {

	/*
	  A particle is the simplest object that can be simulated in the
	  physics system.
	 
	  It has position data (no orientation data), along with
	  velocity. It can be integrated forward through time, and have
	  linear forces, and impulses applied to it. The particle manages
	  its state and allows access through a set of methods.
	 */
	class Particle
	{
	public:

	protected:
		/*
		  Holds the inverse of the mass of the particle. It
		  is more useful to hold the inverse mass because
		  integration is simpler, and because in real time
		  simulation it is more useful to have objects with
		  infinite mass (immovable) than zero mass
		  (completely unstable in numerical simulation).
		 */
		real inverseMass;

		/*
		  Holds the amount of damping applied to linear
		  motion. Damping is required to remove energy added
		  through numerical instability in the integrator.
		 */
		real damping;

		/*
		  Holds the linear position of the particle in
		  world space.
		 */
		Vector3 position;

		/*
		  Holds the linear velocity of the particle in
		  world space.
		 */
		Vector3 velocity;

		/*
		  Holds the accumulated force to be applied at the next
		  simulation iteration only. This value is zeroed at each
		  integration step.
		 */
		Vector3 forceAccum;

		/*
		  Holds the acceleration of the particle.  This value
		  can be used to set acceleration due to gravity (its primary
		  use), or any other constant acceleration.
		 */
		Vector3 acceleration;

	public:

		/*
		 Perform integration to calculate new position velocity and acceleration.
		 This function uses a Newton-Euler integration method, which is a
		 linear approximation to the correct integral. For this reason it
		 may be inaccurate in some cases.
		 */
		void integrate(real duration);

		/*
			Sets the mass of the particle.
			check for infinite mass also done
		 */
		void setMass(const real mass);

		//Gets the mass of the particle.
		
		real getMass() const;

		// Sets the inverse mass of the particle.

		void setInverseMass(const real inverseMass);

		
		// Gets the inverse mass of the particle.
		 
		real getInverseMass() const;

		
		// Returns true if the mass of the particle is not-infinite.
		 
		bool hasFiniteMass() const;

		
		// Sets the damping of the particle. close to 1 -- no drag close to 0 -- heavy drag
		
		void setDamping(const real damping);

		
		// Gets the current damping value.
		
		real getDamping() const;

		// Sets the position of the particle.
		
		void setPosition(const Vector3 &position);

		
		// Sets the position of the particle by component.
		
		void setPosition(const real x, const real y, const real z);

	    // Set the given particle's position 
		 
		void getPosition(Vector3 *position) const;

		
		// Gets the position of the particle.
		
		Vector3 getPosition() const;

		 // Sets the velocity of the particle.
		
		void setVelocity(const Vector3 &velocity);

		// Sets the velocity of the particle by component.
		
		void setVelocity(const real x, const real y, const real z);

		// Fills the given vector with the velocity of the particle.
		
		void getVelocity(Vector3 *velocity) const;

		// Gets the velocity of the particle.
		 
		Vector3 getVelocity() const;

		// Sets the constant acceleration of the particle.
		
		void setAcceleration(const Vector3 &acceleration);

		//Sets the constant acceleration of the particle by component.

		void setAcceleration(const real x, const real y, const real z);

		// Fills the given vector with the acceleration of the particle.

		void getAcceleration(Vector3 *acceleration) const;

		// Gets the acceleration of the particle.

		Vector3 getAcceleration() const;

		// Clears the forces applied to the particle. This will be
		//  called automatically after each integration step.
		
		void clearAccumulator();
	
		//  Adds the given force to the particle, to be applied at the
		// next iteration only.
		 
		void addForce(const Vector3 &force);


	};
}
#endif 