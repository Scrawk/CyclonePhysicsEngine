using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Rigid.Forces
{

    ///<summary>
    /// A force generator that applies a gravitational force. One instance
    /// can be used for multiple rigid bodies.
    ///</summary>
    public class RigidGravityForce : RigidForceArea
    {

        ///<summary>
        /// Holds the acceleration due to gravity.
        ///<summary>
        Vector3d m_gravity;

        public RigidGravityForce(double gravity)
        {
            m_gravity = new Vector3d(0, gravity, 0);
        }

        public RigidGravityForce(Vector3d gravity)
        {
            m_gravity = gravity;
        }

        ///<summary>
        /// Applies the gravitational force to the given rigid body.
        ///</summary>
        public override void UpdateForce(RigidBody body, double dt)
        {
            // Check that we do not have infinite mass
            if (body.HasInfiniteMass) return;

            // Apply the mass-scaled force to the body
            body.AddForce(m_gravity * body.GetMass());
        }
    }

}