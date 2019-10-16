using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Particles;

namespace Cyclone.Particles.Forces
{
    public class ParticleGravityForce : ParticleForceArea
    {

        private Vector3d m_gravity;

        public ParticleGravityForce(double gravity)
        {
            m_gravity = new Vector3d(0, gravity, 0);
        }

        public ParticleGravityForce(Vector3d gravity)
        {
            m_gravity = gravity;
        }

        public override void UpdateForce(Particle particle, double dt)
        {
            // Check that we do not have infinite mass
            if (!particle.HasFiniteMass) return;

            // Apply the mass-scaled force to the particle
            particle.AddForce(m_gravity * particle.GetMass());
        }

    }
}
