using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Rigid.Collisions
{
    /// <summary>
    /// The plane is not a primitive: it doesn't represent another
    /// rigid body.It is used for contacts with the immovable
    /// world geometry.
    /// </summary>
    public class CollisionPlane
    {
        /// <summary>
        /// The plane normal
        /// </summary>
        public Vector3d Direction;

        /// <summary>
        /// The distance of the plane from the origin.
        /// </summary>
        public double Offset;

        /// <summary>
        /// 
        /// </summary>
        public CollisionPlane(Vector3d direction, double offset)
        {
            Direction = direction;
            Offset = offset;
        }
    }
}
