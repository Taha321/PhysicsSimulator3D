#include <cmath>
#include "rigidbody.h"

using namespace MathUtil;

namespace PhysicsSim {

RigidBody::RigidBody(RenderingFramework3D::Mesh& mesh)
    :
    RenderingFramework3D::WorldObject(mesh),
    _position(0),
    _velocity(0),
    _angular_velocity_b(0),
    _mass(50),
    _rot_inertia(50),
    _transform_inv_update(true)
{}

RigidBody::RigidBody(RenderingFramework3D::Mesh& mesh, float mass, Vec<3> principalMomentsOfInertia)
    :
    RenderingFramework3D::WorldObject(mesh),   
    _position(0),
    _velocity(0),
    _angular_velocity_b(0),
    _mass(mass),
    _rot_inertia(principalMomentsOfInertia),
    _transform_inv_update(true)
{}

Vec<3> RigidBody::GetVelocity() const {
    return _velocity;
}
Vec<3> RigidBody::GetMomentum() const {
    return _momentum;
}
Vec<3> RigidBody::GetAngularVelocity() const {
    return _angular_velocity_i;
}
Vec<3> RigidBody::GetAngularMomentum() const {
    return Vec<3>({_angular_momentum(0), _angular_momentum(1), _angular_momentum(2)});
}

void RigidBody::SetVelocity(const Vec<3>& velocity_mps){
    _velocity = velocity_mps;
}

void RigidBody::SetAngularVelocity(const MathUtil::Vec<3>& velocity_radps) {
    _angular_velocity_i = velocity_radps;

    _angular_velocity_b(0) = velocity_radps(0);
    _angular_velocity_b(1) = velocity_radps(1);
    _angular_velocity_b(2) = velocity_radps(2);    

    _angular_velocity_b = GetTransformInv() * _angular_velocity_b;
}

void RigidBody::AddForce(const Vec<3>& force, const Vec<3> pointOfImpact ){
    _pending_forces.push({force, pointOfImpact});
}

void RigidBody::Step(float interval_ms){
    //rotation
    Vec<3> netForce(0);
    Vec<4> netTorque_i(0);
    Vec<4> netTorque_b(0);

    while(_pending_forces.empty()==false) {
        netForce += _pending_forces.front().force;
        auto cross = Cross(_position - _pending_forces.front().poi, _pending_forces.front().force);
        netTorque_i += Vec<4>({cross(0), cross(1), cross(2), 0});
        _pending_forces.pop();
    }

    netTorque_b = GetTransformInv()*netTorque_i;

    Vec<4>& w0 = _angular_velocity_b;
    Vec<4> w1;
    int n = 100;
    float dt = interval_ms/n/1000;
    for(int i=0; i < n; i++) {
        w1(0) = w0(0) + (netTorque_b(0) + (_rot_inertia(1)-_rot_inertia(2))*w0(1)*w0(2))*dt/_rot_inertia(0);
        w1(1) = w0(1) + (netTorque_b(1) + (_rot_inertia(2)-_rot_inertia(0))*w0(2)*w0(0))*dt/_rot_inertia(1);
        w1(2) = w0(2) + (netTorque_b(2) + (_rot_inertia(0)-_rot_inertia(1))*w0(0)*w0(1))*dt/_rot_inertia(2);
        w1(3) = 0;

        w0 = w1;
    }

    auto Lb = Vec<4> ({_rot_inertia(0)*w0(0),_rot_inertia(1)*w0(1), _rot_inertia(2)*w0(2), 0});
    _angular_momentum =  GetTransform() * Lb;

    auto w1i = GetTransform() * w1;
    _angular_velocity_i = Vec<3>({w1i(0), w1i(1), w1i(2)});
    
    Rotate(_angular_velocity_i, std::sqrt(_angular_velocity_i.LenSqr())); 
}
void RigidBody::OnRescale(float x, float y, float z) {
    _transform_inv_update = true;
}
void RigidBody::OnRotate(const MathUtil::Vec<3>& axis, float radians) {
    _transform_inv_update = true;
}
void RigidBody::OnMove(const MathUtil::Vec<3>& position) {
    _transform_inv_update = true;
}

Matrix<4,4>& RigidBody::GetTransformInv() {
    if(_transform_inv_update) {
        //  transform to body frame
        const Matrix<4,4>& transform = GetTransform();
        _transform_inv(0,0) = transform(0,0);
        _transform_inv(0,1) = transform(1,0);
        _transform_inv(0,2) = transform(2,0);

        _transform_inv(1,0) = transform(0,1);
        _transform_inv(1,1) = transform(1,1);
        _transform_inv(1,2) = transform(2,1);

        _transform_inv(2,0) = transform(0,2);
        _transform_inv(2,1) = transform(1,2);
        _transform_inv(2,2) = transform(2,2);

        _transform_inv(3,0) = 0.0;
        _transform_inv(3,1) = 0.0;
        _transform_inv(3,2) = 0.0;
        _transform_inv(0,3) = 0.0;
        _transform_inv(1,3) = 0.0;
        _transform_inv(2,3) = 0.0;
        _transform_inv(3,3) = 0.0;
    }

    return _transform_inv;
}

RigidBodyCuboid::RigidBodyCuboid(RenderingFramework3D::Mesh& mesh, float mass, float width, float height, float length) 
    :
    RigidBody(mesh, mass, Vec<3>({mass*(height*height+length*length), mass*(width*width+length*length), mass*(height*height+width*width)})/12)
{
    SetScale(width, height, length);
}

void RigidBodyCuboid::OnRescale(float x, float y, float z) {
    _transform_inv_update = true;
    _rot_inertia(0) =_mass*(y*y + z*z)/12;
    _rot_inertia(1) =_mass*(x*x + z*z)/12;
    _rot_inertia(2) =_mass*(y*y + x*x)/12;
}

}