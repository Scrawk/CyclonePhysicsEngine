using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Particles.Constraints
{

    ///<summary>
    /// Cables link a pair of particles, generating a contact if they
    /// stray too far apart.
    ///</summary>
    public class ParticleCableConstraint : ParticleConstraint
    {

        /// <summary>
        /// The particle at the other end of the cable.
        /// </summary>
        private Particle m_particleA, m_particleB;

        ///<summary>
        /// Holds the maximum length of the cable.
        ///</summary>
        private double m_maxLength;

        ///<summary>
        /// Holds the restitution (bounciness) of the cable.
        ///</summary>
        private double m_restitution;

        public ParticleCableConstraint(Particle a, Particle b, double maxLength, double restitution = 0)
        {
            m_particleA = a;
            m_particleB = b;
            m_maxLength = maxLength;
            m_restitution = restitution;
        }

        ///<summary>
        /// Fills the given contact structure with the contact needed
        /// to keep the cable from over-extending.
        ///</summary>
        public override int AddContact(IList<Particle> particles, IList<ParticleContact> contacts, int next)
        {
            // Find the length of the cable
            double length = Vector3d.Distance(m_particleA.Position, m_particleB.Position);

            // Check if we're over-extended
            if (length < m_maxLength) return 0;

            var contact = contacts[next];

            // Otherwise return the contact
            contact.Particles[0] = m_particleA;
            contact.Particles[1] = m_particleB;

            // Calculate the normal
            Vector3d normal = m_particleB.Position - m_particleA.Position;
            normal.Normalize();
            contact.ContactNormal = normal;

            contact.Penetration = length - m_maxLength;
            contact.Restitution = m_restitution;

            return 1;
        }
    }
}
