#include"namedefs.h"
#include"vectormath.h"
#include<GL/glut.h>
namespace wizphys{

	class Particle{
		public:
			Vector3D position;
			Vector3D velocity;
			Vector3D acceleration;
			Vector3D forceAccum;
			real damping;
			real massinverse;
			real mass;

			void integrate(real duration);
			void setMass(real m);
			void setMassInverse(real invertmass);

			real getMass();
			real getMassInverse();

			bool hasMassFinite();
			bool hasMass0();

			void setPosition(Vector3D pos) ;
			void setPosition(real x,real y,real z);
			Vector3D getPosition() const;
			void getPosition(Vector3D* pos) const;

			void setVelocity(Vector3D v);
			void setVelocity(real x,real y,real z);
			Vector3D getVelocity() const;
			void getVelocity(Vector3D* v) const;

			void setAccel(Vector3D a);
			void setAccel(real x,real y,real z);
			Vector3D getAccel() const;
			void getAccel(Vector3D* a) const;

			void setForceAccum(Vector3D v);
			void setForceAccum(real x,real y,real z);
			Vector3D getForceAccum() const;
			void getForceAccum(Vector3D* f) const;
			void addForce(Vector3 force);

			void setDamp(real dampval);
			real getDamp() const;

			void clearAccumulator();
	};
}