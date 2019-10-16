using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Particles;

namespace Cyclone.Particles.Forces
{

    /// <summary>
    /// A force that applies to one or more particles 
    /// during the duration of the simulation.
    /// </summary>
    public abstract class ParticleForce
    {
        /// <summary>
        /// Overload this in implementations of the interface to calculate
        /// and update the force applied to the particles.
        /// </summary>
        public abstract void UpdateForce(double dt);

    }
}
