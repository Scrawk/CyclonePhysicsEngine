using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Rigid.Forces
{

    ///<summary>
    /// 
    ///</summary>
    public class RigidSpringForce : RigidForce
    {

        ///<summary>
        /// The bodies at the other end of the spring.
        ///</summary>
        private RigidBody m_bodyA, m_bodyB;

        ///<summary>
        /// The point of connection of the spring, in local coordinates.
        ///</summary>
        private Vector3d m_connectionA, m_connectionB;

        ///<summary>
        /// Holds the sprint constant
        ///</summary>
        private double m_springConstant;

        ///<summary>
        /// Holds the rest length of the spring
        ///</summary>
        private double m_restLength;

        ///<summary>
        ///
        ///</summary>
        public RigidSpringForce(RigidBody a, RigidBody b, Vector3d connectionA, Vector3d connectionB, double springConstant, double restLength)
        {
            m_bodyA = a;
            m_bodyB = b;
            m_connectionA = connectionA;
            m_connectionB = connectionB;
            m_springConstant = springConstant;
            m_restLength = restLength;
        }

        public override void UpdateForce(double dt)
        {
            UpdateForce(m_bodyA, m_bodyB, m_connectionA, m_connectionB, dt);
            UpdateForce(m_bodyB, m_bodyA, m_connectionB, m_connectionA, dt);
        }

        private void UpdateForce(RigidBody body, RigidBody other, Vector3d connection, Vector3d otherConnection, double dt)
        {
            if (body.HasInfiniteMass) return;

            // Calculate the two ends in world space
            Vector3d lws = body.GetPointInWorldSpace(connection);
            Vector3d ows = other.GetPointInWorldSpace(otherConnection);

            // Calculate the vector of the spring
            Vector3d force = lws - ows;

            // Calculate the magnitude of the force
            double magnitude = force.Magnitude;
            // Calculate the magnitude of the force
            magnitude = (m_restLength - magnitude) * m_springConstant;

            // Calculate the final force and apply it
            force.Normalize();
            force *= magnitude;
            body.AddForceAtPoint(force, lws);
            
        }
    }

}