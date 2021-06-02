#ifndef CYCLONE_BODY_H
#define CYCLONE_BODY_H

#include "core.h"

namespace cyclone {

	class RigidBody
	{
	public:


	protected:
		/**
		 * Holds the inverse of the mass of the rigid body. It
		 * is more useful to hold the inverse mass because
		 * integration is simpler, and because in real time
		 * simulation it is more useful to have bodies with
		 * infinite mass (immovable) than zero mass
		 * (completely unstable in numerical simulation).
		 */
		real inverseMass;

		/**
		 * Holds the inverse of the body's inertia tensor. The
		 * inertia tensor provided must not be degenerate
		 * (that would mean the body had zero inertia for
		 * spinning along one axis). As long as the tensor is
		 * finite, it will be invertible. The inverse tensor
		 * is used for similar reasons to the use of inverse
		 * mass.
		 *
		 * The inertia tensor, unlike the other variables that
		 * define a rigid body, is given in body space.
		 *
		 */
		Matrix3 inverseInertiaTensor;

		/**
		 * Holds the amount of damping applied to linear
		 * motion.  Damping is required to remove energy added
		 * through numerical instability in the integrator.
		 */
		real linearDamping;

		/**
		 * Holds the amount of damping applied to angular
		 * motion.  Damping is required to remove energy added
		 * through numerical instability in the integrator.
		 */
		real angularDamping;

		/**
		 * Holds the linear position of the rigid body in
		 * world space.
		 */
		Vector3 position;

		/**
		 * Holds the angular orientation of the rigid body in
		 * world space.
		 */
		Quaternion orientation;

		/**
		 * Holds the linear velocity of the rigid body in
		 * world space.
		 */
		Vector3 velocity;

		/**
		 * Holds the angular velocity, or rotation, or the
		 * rigid body in world space.
		 */
		Vector3 rotation;

		/**
		 * Holds the inverse inertia tensor of the body in world
		 * space. The inverse inertia tensor member is specified in
		 * the body's local space.
		 *
		 */
		Matrix3 inverseInertiaTensorWorld;

		/**
		 * Holds the amount of motion of the body. This is a recency
		 * weighted mean that can be used to put a body to sleap.
		 */
		real motion;

		/**
		 * A body can be put to sleep to avoid it being updated
		 * by the integration functions or affected by collisions
		 * with the world.
		 */
		bool isAwake;

		/**
		 * Some bodies may never be allowed to fall asleep.
		 * User controlled bodies, for example, should be
		 * always awake.
		 */
		bool canSleep;

		/**
		 * Holds a transform matrix for converting body space into
		 * world space and vice versa. This can be achieved by calling
		 * the getPointIn*Space functions.
		 */
		Matrix4 transformMatrix;

		/**
		 * Holds the accumulated force to be applied at the next
		 * integration step.
		 */
		Vector3 forceAccum;

		/**
		 * Holds the accumulated torque to be applied at the next
		 * integration step.
		 */
		Vector3 torqueAccum;

	   /**
		 * Holds the acceleration of the rigid body.  This value
		 * can be used to set acceleration due to gravity (its primary
		 * use), or any other constant acceleration.
		 */
		Vector3 acceleration;

		/**
		 * Holds the linear acceleration of the rigid body, for the
		 * previous frame.
		 */
		Vector3 lastFrameAcceleration;


	public:
		/**
		 * Calculates internal data from state data. This should be called
		 * after the body's state is altered directly (it is called
		 * automatically during integration). If you change the body's state
		 * and then intend to integrate before querying any data (such as
		 * the transform matrix), then you can ommit this step.
		 */
		void calculateDerivedData();

		/**
		 * Integrates the rigid body forward in time by the given amount.
		 * This function uses a Newton-Euler integration method, which is a
		 * linear approximation to the correct integral. For this reason it
		 * may be inaccurate in some cases.
		 */
		void integrate(real duration);

		/**
		 * Sets the mass of the rigid body.
		 */
		void setMass(const real mass);

		/**
		 * Gets the mass of the rigid body.
		 */
		real getMass() const;

		/**
		 * Sets the inverse mass of the rigid body.
		 */
		void setInverseMass(const real inverseMass);

		/**
		 * Gets the inverse mass of the rigid body.
		 */
		real getInverseMass() const;

		/**
		 * Returns true if the mass of the body is not-infinite.
		 */
		bool hasFiniteMass() const;

		/**
		 * Sets the intertia tensor for the rigid body.
		 */
		void setInertiaTensor(const Matrix3 &inertiaTensor);

		/**
		 * Copies the current inertia tensor of the rigid body into
		 * the given matrix.
		 */
		void getInertiaTensor(Matrix3 *inertiaTensor) const;

		/**
		 * Gets a copy of the current inertia tensor of the rigid body.
		 */
		Matrix3 getInertiaTensor() const;

		/**
		 * Copies the current inertia tensor of the rigid body into
		 * the given matrix.
		 */
		void getInertiaTensorWorld(Matrix3 *inertiaTensor) const;

		/**
		 * Gets a copy of the current inertia tensor of the rigid body.
		 *
		 */
		Matrix3 getInertiaTensorWorld() const;

		/**
		 * Sets the inverse intertia tensor for the rigid body.
		 */
		void setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);

		/**
		 * Copies the current inverse inertia tensor of the rigid body
		 * into the given matrix.
		 */
		void getInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const;

		/**
		 * Gets a copy of the current inverse inertia tensor of the
		 * rigid body.
		 */
		Matrix3 getInverseInertiaTensor() const;

		/**
		 * Copies the current inverse inertia tensor of the rigid body
		 * into the given matrix.
		 */
		void getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;

		/**
		 * Gets a copy of the current inverse inertia tensor of the
		 * rigid body.
		 */
		Matrix3 getInverseInertiaTensorWorld() const;

		/**
		 * Sets both linear and angular damping in one function call.
		 */
		void setDamping(const real linearDamping, const real angularDamping);

		/**
		 * Sets the linear damping for the rigid body.
		 */
		void setLinearDamping(const real linearDamping);

		/**
		 * Gets the current linear damping value.
		 *
		 */
		real getLinearDamping() const;

		/**
		 * Sets the angular damping for the rigid body.
		 */
		void setAngularDamping(const real angularDamping);

		/**
		 * Gets the current angular damping value.
		 */
		real getAngularDamping() const;

		/**
		 * Sets the position of the rigid body.
		 */
		void setPosition(const Vector3 &position);

		/**
		 * Sets the position of the rigid body by component.
		 */
		void setPosition(const real x, const real y, const real z);

		/**
		 * Fills the given vector with the position of the rigid body.
		 *
		 */
		void getPosition(Vector3 *position) const;

		/**
		 * Gets the position of the rigid body.
		 */
		Vector3 getPosition() const;

		/**
		 * Sets the orientation of the rigid body.
		 */
		void setOrientation(const Quaternion &orientation);

		/**
		 * Sets the orientation of the rigid body by component.
		 *
		 */
		void setOrientation(const real r, const real i,
			const real j, const real k);

		/**
		 * Fills the given quaternion with the current value of the
		 * rigid body's orientation.
		 */
		void getOrientation(Quaternion *orientation) const;

		/**
		 * Gets the orientation of the rigid body.
		 */
		Quaternion getOrientation() const;

		/**
		 * Fills the given matrix with a transformation representing
		 * the rigid body's orientation.
		 */
		void getOrientation(Matrix3 *matrix) const;

		/**
		 * Fills the given matrix data structure with a transformation
		 * representing the rigid body's orientation.
		 */
		void getOrientation(real matrix[9]) const;

		/**
		 * Fills the given matrix with a transformation representing
		 * the rigid body's position and orientation.
		 */
		void getTransform(Matrix4 *transform) const;

		/**
		 * Fills the given matrix data structure with a
		 * transformation representing the rigid body's position and
		 * orientation.
		 */
		void getTransform(real matrix[16]) const;

		/**
		 * Fills the given matrix data structure with a
		 * transformation representing the rigid body's position and
		 * orientation. The matrix is transposed from that returned
		 * by getTransform. This call returns a matrix suitable
		 * for applying as an OpenGL transform.
		 */
		void getGLTransform(float matrix[16]) const;

		/**
		 * Gets a transformation representing the rigid body's
		 * position and orientation.
		 */
		Matrix4 getTransform() const;

		/**
		 * Converts the given point from world space into the body's
		 * local space.
		 */
		Vector3 getPointInLocalSpace(const Vector3 &point) const;

		/**
		 * Converts the given point from world space into the body's
		 * local space.
		 */
		Vector3 getPointInWorldSpace(const Vector3 &point) const;

		/**
		 * Converts the given direction from world space into the
		 * body's local space.
		 */
		Vector3 getDirectionInLocalSpace(const Vector3 &direction) const;

		/**
		 * Converts the given direction from world space into the
		 * body's local space.
		 */
		Vector3 getDirectionInWorldSpace(const Vector3 &direction) const;

		/**
		 * Sets the velocity of the rigid body.
		 *
		 */
		void setVelocity(const Vector3 &velocity);

		/**
		 * Sets the velocity of the rigid body by component. The
		 * velocity is given in world space.
		 *
		 */
		void setVelocity(const real x, const real y, const real z);

		/**
		 * Fills the given vector with the velocity of the rigid body.
		 *
		 */
		void getVelocity(Vector3 *velocity) const;

		/**
		 * Gets the velocity of the rigid body.
		 *
		 */
		Vector3 getVelocity() const;

		/**
		 * Applies the given change in velocity.
		 */
		void addVelocity(const Vector3 &deltaVelocity);

		/**
		 * Sets the rotation of the rigid body.
		 *
		 */
		void setRotation(const Vector3 &rotation);

		/**
		 * Sets the rotation of the rigid body by component. The
		 * rotation is given in world space.
		 */
		void setRotation(const real x, const real y, const real z);

		/**
		 * Fills the given vector with the rotation of the rigid body.
		 *
		 */
		void getRotation(Vector3 *rotation) const;

		/**
		 * Gets the rotation of the rigid body.
		 *
		 */
		Vector3 getRotation() const;

		/**
		 * Applies the given change in rotation.
		 */
		void addRotation(const Vector3 &deltaRotation);

		/**
		 * Returns true if the body is awake and responding to
		 * integration.
		 *
		 */
		bool getAwake() const
		{
			return isAwake;
		}

		/**
		 * Sets the awake state of the body. If the body is set to be
		 * not awake, then its velocities are also cancelled, since
		 * a moving body that is not awake can cause problems in the
		 * simulation.
		 *
		 */
		void setAwake(const bool awake=true);

		/**
		 * Returns true if the body is allowed to go to sleep at
		 * any time.
		 */
		bool getCanSleep() const
		{
			return canSleep;
		}

		/**
		 * Sets whether the body is ever allowed to go to sleep. Bodies
		 * under the player's control, or for which the set of
		 * transient forces applied each frame are not predictable,
		 * should be kept awake.
		 *
		 */
		void setCanSleep(const bool canSleep=true);

		/**
		 * Fills the given vector with the current accumulated value
		 * for linear acceleration. The acceleration accumulators
		 * are set during the integration step. They can be read to
		 * determine the rigid body's acceleration over the last
		 * integration step. The linear acceleration is given in world
		 * space.
		 *
		 */
		void getLastFrameAcceleration(Vector3 *linearAcceleration) const;

		/**
		 * Gets the current accumulated value for linear
		 * acceleration. The acceleration accumulators are set during
		 * the integration step. They can be read to determine the
		 * rigid body's acceleration over the last integration
		 * step. The linear acceleration is given in world space.
		 *
		 */
		Vector3 getLastFrameAcceleration() const;

		/**
		 * Clears the forces and torques in the accumulators. This will
		 * be called automatically after each intergration step.
		 */
		void clearAccumulators();

		/**
		 * Adds the given force to centre of mass of the rigid body.
		 * The force is expressed in world-coordinates.
		 *
		 */
		void addForce(const Vector3 &force);

		/**
		 * Adds the given force to the given point on the rigid body.
		 * Both the force and the
		 * application point are given in world space. Because the
		 * force is not applied at the centre of mass, it may be split
		 * into both a force and torque.
		 */
		void addForceAtPoint(const Vector3 &force, const Vector3 &point);

		/**
		 * Adds the given force to the given point on the rigid body.
		 * The direction of the force is given in world coordinates,
		 * but the application point is given in body space. This is
		 * useful for spring forces, or other forces fixed to the
		 * body.
		 */
		void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);

		/**
		 * Adds the given torque to the rigid body.
		 * The force is expressed in world-coordinates.
		 */
		void addTorque(const Vector3 &torque);

		/**
		 * Sets the constant acceleration of the rigid body.
		 */
		void setAcceleration(const Vector3 &acceleration);

		/**
		 * Sets the constant acceleration of the rigid body by component.
		 */
		void setAcceleration(const real x, const real y, const real z);

		/**
		 * Fills the given vector with the acceleration of the rigid body.
		 *
		 */
		void getAcceleration(Vector3 *acceleration) const;

		/**
		 * Gets the acceleration of the rigid body.
		 */
		Vector3 getAcceleration() const;


	};

} // namespace cyclone

#endif // CYCLONE_BODY_H