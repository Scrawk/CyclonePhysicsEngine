using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Rigid;

namespace Cyclone.Rigid.Forces
{

    /// <summary>
    /// A force that applies to one or more bodies 
    /// during the duration of the simulation.
    /// </summary>
    public abstract class RigidForce
    {
        /// <summary>
        /// Overload this in implementations of the interface to calculate
        /// and update the force applied to the bodies.
        /// </summary>
        public abstract void UpdateForce(double dt);

    }
}
