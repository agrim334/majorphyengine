#include"namedefs.h"
#include<random>
namespace wizphys{

	class Vector3D{
		public:
			real x;
			real y;
			real z;

		Vector3D(){
			x = 0;
			y = 0;
			z = 0;
		}

		Vector3D(real rx,real ry, real rz){
			x = rx;
			y = ry;
			z = rz;
		}
		void clear(){
			x = y = z = 0;
		}
		real magnitude(){
			return sqrt(x*x+y*y+z*z);
		}
		
		real magnitudeSquare(){
			return x*x+y*y+z*z;
		}

		void normalize(){
			real mag = magnitude();
			if(mag > 0)
				(*this)*=(((real)1)/mag);
		}

		Vector3D scale(real sf){
			return Vector3D(sf*x,sf*y,sf*z);
		}

		void operator*=(real sf){
			x = sf*x;
			y = sf*y;
			z = sf*z;
		}

		void invert(){
			x = -x;
			y = -y;
			z = -z;
		}

		Vector3D operator+(Vector3D &v){
			Vector3D vf(x + v.x,y + v.y,z + v.z);
			return vf;
		}

		Vector3D operator-(Vector3D &v){
			Vector3D vf(x - v.x,y - v.y,z - v.z);
			return vf;
		}

		void operator+=(Vector3D v){
			x = x + v.x;
			y = y + v.y;
			z = z + v.z;
		}

		void operator-=(Vector3D v){
			x = x - v.x;
			y = y - v.y;
			z = z - v.z;
		}

		void scaledAdd(Vector3D &v, real sf){
			Vector3D vf;
			vf = v;
			vf *= sf;
			(*this) += vf;
		}

		Vector3D compProduct(Vector3D &v){
			Vector3D vf(x * v.x,y * v.y,z * v.z);
			return vf;
		}

		void compProductupd(Vector3D &v){
			x *= v.x;
			y *= v.y;
			z *= v.z;
		}

		real scalarProduct(Vector3D &vf){
			return x*vf.x + y*vf.y + z*vf.z;
		}

		Vector3D crossProduct(Vector3D &v){
			Vector3D vf(y*v.z - z*v.y,z*v.x - x*v.z,x*v.y - y*v.x);
			return vf;
		}

		Vector3D operator% (Vector3D &v){
			Vector3D vf(y*v.z - z*v.y,z*v.x - x*v.z,x*v.y - y*v.x);
			return vf;
		}

		real angle(Vector3D v1,Vector3D v2){
			real v1mag = v1.magnitude();
			real v2mag = v2.magnitude();
			real sp = v1.scalarProduct(v2);
			return acos(sp/(v1mag*v2mag));
		}

		void operator%=(Vector3D &v){
			*this = *(this)%v;
		}

	};
}