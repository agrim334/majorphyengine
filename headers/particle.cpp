#include"particle.h"
#include<iostream>
#include<assert.h>
using namespace std;
using namespace wizphys;

const Vector3D gravity(0,10,0);

void Particle::integrate(real interval){
	assert(interval > 0.0);
	position.scaledAdd(velocity, interval);
	Vector3D resultingAcc = acceleration;
	resultingAcc.scaledAdd(forceAccum, massinverse);
	velocity.scaledAdd(resultingAcc, interval);
	float dampval = pow(damping, interval);
	velocity *= dampval;
}

int main(){
	Vector3D v1 = Vector3D(1.0,2.0,0.0);
	Particle p1;
	cout << v1.x << "\n";
	Vector3D v2 = v1 + v1;
	cout << v2.x << " " << p1.mass << "\n";

	return 0;
}