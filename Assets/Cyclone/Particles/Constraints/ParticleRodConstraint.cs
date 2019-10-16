using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Particles.Constraints
{
    ///<summary>
    /// Rods link a pair of particles, generating a contact if they
    /// stray too far apart or too close.
    ///</summary>
    public class ParticleRodConstraint : ParticleConstraint
    {

        /// <summary>
        /// The particle at the other end of the rod.
        /// </summary>
        private Particle m_particleA, m_particleB;

        ///<summary>
        /// Holds the length of the rod.
        ///</summary>
        private double m_length;

        public ParticleRodConstraint(Particle a, Particle b, double length)
        {
            m_particleA = a;
            m_particleB = b;
            m_length = length;
        }

        ///<summary>
        /// Fills the given contact structure with the contact needed
        /// to keep the rod from extending or compressing.
        ///</summary>
        public override int AddContact(IList<Particle> particles, IList<ParticleContact> contacts, int next)
        {
            // Find the length of the cable
            double currentLen = Vector3d.Distance(m_particleA.Position, m_particleB.Position);

            // Check if we're over-extended
            if (currentLen == m_length) return 0;

            var contact = contacts[next];

            // Otherwise return the contact
            contact.Particles[0] = m_particleA;
            contact.Particles[1] = m_particleB;

            // Calculate the normal
            Vector3d normal = m_particleB.Position - m_particleA.Position;
            normal.Normalize();

            // The contact normal depends on whether we're extending or compressing
            if (currentLen > m_length)
            {
                contact.ContactNormal = normal;
                contact.Penetration = currentLen - m_length;
            }
            else
            {
                contact.ContactNormal = normal * -1;
                contact.Penetration = m_length - currentLen;
            }

            // Always use zero restitution (no bounciness)
            contact.Restitution = 0;

            return 1;
        }

    }
}
