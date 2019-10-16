using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Rigid.Constraints
{

    ///<summary>
    /// Joints link together two rigid bodies and make sure they do not
    /// separate.  In a general phyiscs engine there may be many
    /// different types of joint, that reduce the number of relative
    /// degrees of freedom between two objects. This joint is a common
    /// position joint: each object has a location (given in
    /// body-coordinates) that will be kept at the same point in the
    /// simulation.
    ///</summary>
    public class RigidJointConstraint : RigidConstraint
    {

        ///<summary>
        /// Holds the two rigid bodies that are connected by this joint.
        ///</summary>
        private RigidBody[] m_body;

        ///<summary>
        /// Holds the relative location of the connection for each
        /// body, given in local coordinates.
        ///</summary>
        private Vector3d[] m_position;

        ///<summary>
        /// Holds the maximum displacement at the joint before the
        /// joint is considered to be violated. This is normally a
        /// small, epsilon value.  It can be larger, however, in which
        /// case the joint will behave as if an inelastic cable joined
        /// the bodies at their joint locations.
        ///</summary>
        private double m_error;

        ///<summary>
        /// Configures the joint in one go.
        ///</summary>
        public RigidJointConstraint(RigidBody a, Vector3d posA, RigidBody b, Vector3d posB, double error)
        {
            m_body = new RigidBody[] { a, b };
            m_position = new Vector3d[] { posA, posB };
            m_error = error;
        }

        ///<summary>
        /// Generates the contacts required to restore the joint if it
        /// has been violated.
        ///</summary>
        public override int AddContact(IList<RigidBody> bodies, IList<RigidContact> contacts, int next)
        {
            // Calculate the position of each connection point in world coordinates
            Vector3d a_pos_world = m_body[0].GetPointInWorldSpace(m_position[0]);
            Vector3d b_pos_world = m_body[1].GetPointInWorldSpace(m_position[1]);

            // Calculate the length of the joint
            Vector3d a_to_b = b_pos_world - a_pos_world;
            Vector3d normal = a_to_b;
            normal.Normalize();
            double length = a_to_b.Magnitude;

            // Check if it is violated
            if (Math.Abs(length) > m_error)
            {
                var contact = contacts[next];
                contact.Body[0] = m_body[0];
                contact.Body[1] = m_body[1];
                contact.ContactNormal = normal;
                contact.ContactPoint = (a_pos_world + b_pos_world) * 0.5;
                contact.Penetration = length - m_error;
                contact.Friction = 1.0;
                contact.Restitution = 0;
                return 1;
            }

            return 0;
        }
    }

}