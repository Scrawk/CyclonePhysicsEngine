using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Particles;

namespace Cyclone.Particles.Forces
{
    public class ParticleElasticForce : ParticleForce
    {
        /// <summary>
        /// The particle at the other end of the spring.
        /// </summary>
        private Particle m_particleA, m_particleB;

        /// <summary>
        /// Holds the sprint constant.
        /// </summary>
        private double m_springConstant;

        /// <summary>
        /// Holds the rest length of the spring.
        /// </summary>
        private double m_restLength;

        public ParticleElasticForce(Particle a, Particle b, double springConstant, double restLength)
        {
            m_particleA = a;
            m_particleB = b;
            m_springConstant = springConstant;
            m_restLength = restLength;
        }

        public override void UpdateForce(double dt)
        {
            UpdateForce(m_particleA, m_particleB);
            UpdateForce(m_particleB, m_particleA);
        }

        private void UpdateForce(Particle particle, Particle other)
        {
            if (particle.HasInfiniteMass) return;

            // Calculate the vector of the spring
            Vector3d force = particle.Position - other.Position;

            // Calculate the magnitude of the force
            double magnitude = force.Magnitude;
            if (magnitude <= m_restLength) return;

            // Calculate the magnitude of the force
            magnitude = (m_restLength - magnitude) * m_springConstant;

            // Calculate the final force and apply it
            force.Normalize();
            force *= magnitude;
            particle.AddForce(force);
        }
    }
}
