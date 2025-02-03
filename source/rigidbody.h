#pragma once

#include <queue>
#include "define.h"
#include "matrix.h"
#include "renderer.h"

namespace PhysicsSim {

class RigidBody : public RenderingFramework3D::WorldObject {
public:

    RigidBody(RenderingFramework3D::Mesh& mesh);
    RigidBody(RenderingFramework3D::Mesh& mesh, float mass, MathUtil::Vec<3> principalMomentsOfInertia);
    virtual ~RigidBody() {}

    MathUtil::Vec<3> GetVelocity() const;
    MathUtil::Vec<3> GetMomentum() const;
        
    MathUtil::Vec<3> GetAngularVelocity() const;
    MathUtil::Vec<3> GetAngularMomentum() const;

    void SetVelocity(const MathUtil::Vec<3>& velocity_mps);
    void SetAngularVelocity(const MathUtil::Vec<3>& velocity_radps);

    void AddForce(const MathUtil::Vec<3>& force, const MathUtil::Vec<3> pointOfImpact);
        
    void Step(float interval_ms);

protected:
    MathUtil::Matrix<4,4>& GetTransformInv();

private:
	virtual void OnRescale(float x, float y, float z) override;
	virtual void OnRotate(const MathUtil::Vec<3>& axis, float radians) override;
	virtual void OnMove(const MathUtil::Vec<3>& position) override;

protected:
    struct ForceEntry {
        MathUtil::Vec<3> force;
        MathUtil::Vec<3> poi;
    };

    MathUtil::Vec<3> _position;
    MathUtil::Vec<3> _velocity;
    MathUtil::Vec<3> _momentum;
    
    MathUtil::Vec<4> _angular_velocity_b;
    MathUtil::Vec<3> _angular_velocity_i;
    MathUtil::Vec<4> _angular_momentum;

    float _mass;
    MathUtil::Vec<3> _rot_inertia;

    bool _transform_inv_update;
    MathUtil::Matrix<4,4> _transform_inv;

    std::queue<ForceEntry> _pending_forces;
};

class RigidBodyCuboid : public RigidBody {
public:
    RigidBodyCuboid(RenderingFramework3D::Mesh& mesh, float mass, float width, float height, float length);

private:
    virtual void OnRescale(float x, float y, float z) override;
};
}