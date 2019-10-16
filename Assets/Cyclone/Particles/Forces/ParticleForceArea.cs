using System;
using System.Collections.Generic;
using System.Text;

namespace Cyclone.Particles.Forces
{
    /// <summary>
    /// A force that applies to any partice that enters the forces area.
    /// </summary>
    public abstract class ParticleForceArea
    {
        /// <summary>
        /// Overload this in implementations of the interface to calculate
        /// and update the force applied to the particle.
        /// </summary>
        public abstract void UpdateForce(Particle particle, double dt);

    }
}
