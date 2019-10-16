using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Particles;

namespace Cyclone.Particles.Constraints
{
    /// <summary>
    /// A contact generator that takes an STL vector of particle pointers and
    /// collides them against the ground.
    /// </summary>
    public class PlaneConstraint : ParticleConstraint
    {

        public Vector3d m_origin;

        public PlaneConstraint(double height)
        {
            m_origin = new Vector3d(0, height, 0);
        }

        public override int AddContact(IList<Particle> particles, IList<ParticleContact> contacts, int next)
        {
            int count = 0;
            foreach(var p in particles)
            {
                double y = p.Position.y - m_origin.y;
                if (y < 0.0)
                {
                    var contact = contacts[next];

                    contact.ContactNormal = Vector3d.UnitY;
                    contact.Particles[0] = p;
                    contact.Particles[1] = null;
                    contact.Penetration = -y;
                    contact.Restitution = 0.2;
                    next++;
                    count++;
                }

                if (count >= contacts.Count) return count;
            }

            return count;
        }
    }
}
