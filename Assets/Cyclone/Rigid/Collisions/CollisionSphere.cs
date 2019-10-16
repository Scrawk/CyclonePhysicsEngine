using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Rigid;

namespace Cyclone.Rigid.Collisions
{
    ///<summary>
    /// Represents a rigid body that can be treated as a sphere
    /// for collision detection.
    ///</summary>
    public class CollisionSphere : CollisionPrimitive
    {
        public double Radius;

        public CollisionSphere(double radius)
        {
            Radius = radius;
        }
    }
}
