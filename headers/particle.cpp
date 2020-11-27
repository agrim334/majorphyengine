#include"particle.h"
#include<iostream>
#include<assert.h>
using namespace std;
using namespace wizphys;

const Vector3D gravity(0,-10,0);

void Particle::integrate(real interval){
	assert(interval > 0.0);
	position.scaledAdd(velocity, interval);
	Vector3D resultingAcc = acceleration;
	resultingAcc.scaledAdd(forceAccum, massinverse);
	velocity.scaledAdd(resultingAcc, interval);
	float dampval = pow(damping, interval);
	velocity *= dampval;
	clearAccumulator();
}

void Particle::setMass(real m){
	mass = m;
	setMassInverse(mass);
}

void Particle::setMassInverse(real m){
	if(m != 0)
		massinverse = ((real)1)/m;
	else
		massinverse = INFINITY;
}
real Particle::getMass(){
	if (massinverse != 0)
		return mass;
	else
		return INFINITY;
}

real Particle::getMassInverse(){
	if (mass != 0)
		return massinverse;
	else
		return INFINITY;
}

void Particle::setPosition(Vector3D pos){
	position = pos;
}

void Particle::setPosition(real x ,real y , real z){
	position.x = x;
	position.y = y;
	position.z = z;

}

Vector3D Particle::getPosition(){
	return position;
}

void Particle::getPosition(Vector3D* pos){
	*pos = position;
}

void Particle::setVelocity(Vector3D v){
	velocity = v;
}

void Particle::setVelocity(real x ,real y , real z){
	velocity.x = x;
	velocity.y = y;
	velocity.z = z;
}

Vector3D Particle::getVelocity(){
	return velocity;
}

void Particle::getVelocity(Vector3D* v){
	*v = velocity;
}

void Particle::setAccel(Vector3D a){
	acceleration = a;
}

void Particle::setAccel(real x ,real y , real z){
	acceleration.x = x;
	acceleration.y = y;
	acceleration.z = z;
}

Vector3D Particle::getAccel(){
	return acceleration;
}

void Particle::getAccel(Vector3D* a){
	*a = acceleration;
}

void Particle::setForceAccum(Vector3D f){
	forceAccum = f;
}

void Particle::setForceAccum(real x ,real y , real z){
	forceAccum.x = x;
	forceAccum.y = y;
	forceAccum.z = z;
}

Vector3D Particle::getForceAccum(){
	return forceAccum;
}

void Particle::getForceAccum(Vector3D* f){
	*f = forceAccum;
}

void Particle::clearAccumulator()
{
    forceAccum.clear();
}

void Particle::setDamp(real dampval){
	damping = dampval;
}

void Particle::addForce(Vector3D force){
	forceAccum += force;
}

real Particle::getDamp(){
	return damping;
}

int main(){
	Vector3D v1 = Vector3D(1.0,2.0,0.0);
	Vector3D v2 = Vector3D(1.0,2.0,3.423);
	v1.scaledAdd(v2,-29);
	cout << v1.z << "\n";
	v1 = v1 - v2;
	cout << v1.z << "\n";
	v1 *= 2;
	cout << v1.z << "\n";
	real i = v1.scalarProduct(v2);
	v1 %= v2;
	cout << v1.x << "\n";		
	return 0;
}