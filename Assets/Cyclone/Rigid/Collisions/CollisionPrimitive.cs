using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Rigid.Collisions
{
    ///<summary>
    /// Represents a primitive to detect collisions against.
    ///</summary>
    public abstract class CollisionPrimitive
    {

        ///<summary>
        /// The rigid body that is represented by this primitive.
        ///</summary>
        public RigidBody Body;

        ///<summary>
        /// The offset of this primitive from the given rigid body.
        ///</summary>
        //public Matrix4 Offset;

        ///<summary>
        /// The resultant transform of the primitive. This is
        /// calculated by combining the offset of the primitive
        /// with the transform of the rigid body.
        ///</summary>
        public Matrix4 Transform;

        /// <summary>
        /// 
        /// </summary>
        public CollisionPrimitive()
        {
            Transform = Matrix4.Identity;
        }

        ///<summary>
        /// Calculates the internals for the primitive.
        ///</summary>
        public void CalculateInternals()
        {
            Transform = Body.Transform /* * Offset*/ ;
        }

        ///<summary>
        /// This is a convenience function to allow access to the
        /// axis vectors in the transform for this primitive.
        ///</summary>
        public Vector3d GetAxis(int index)
        {
            return Transform.GetAxisVector(index);
        }

    }
}
