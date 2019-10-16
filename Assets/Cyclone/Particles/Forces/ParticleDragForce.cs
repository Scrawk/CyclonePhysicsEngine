using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Particles;

namespace Cyclone.Particles.Forces
{
    public class ParticleDragForce : ParticleForceArea
    {
        //Holds the velocity drag coeffificent.
        private double k1;

        // Holds the velocity squared drag coeffificent. 
        private double k2;

        public ParticleDragForce(double k1, double k2)
        {
            this.k1 = k1;
            this.k2 = k2;
        }

        public override void UpdateForce(Particle particle, double dt)
        {
            Vector3d force = particle.Velocity;

            // Calculate the total drag coefficient
            double dragCoeff = force.Magnitude;
            dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

            // Calculate the final force and apply it
            force.Normalize();
            force *= -dragCoeff;
            particle.AddForce(force);
        }
    }
}
