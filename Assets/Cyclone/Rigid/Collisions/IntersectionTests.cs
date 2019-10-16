using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Rigid;

namespace Cyclone.Rigid.Collisions
{
    /// <summary>
    /// A wrapper class that holds fast intersection tests. These
    /// can be used to drive the coarse collision detection system or
    /// as an early out in the full collision tests below.
    /// </summary>
    public static class IntersectionTests
    {
        public static bool SphereAndHalfSpace(CollisionSphere sphere, CollisionPlane plane)
        {
            // Find the distance from the origin
            double ballDistance = Vector3d.Dot(plane.Direction, sphere.GetAxis(3)) - sphere.Radius;

            // Check for the intersection
            return ballDistance <= plane.Offset;
        }

        public static bool SphereAndSphere(CollisionSphere one, CollisionSphere two)
        {
            // Find the vector between the objects
            Vector3d midline = one.GetAxis(3) - two.GetAxis(3);

            // See if it is large enough.
            return midline.SqrMagnitude < (one.Radius + two.Radius) * (one.Radius + two.Radius);
        }

        public static bool BoxAndBox(CollisionBox one, CollisionBox two)
        {
            // Find the vector between the two centres
            Vector3d toCentre = two.GetAxis(3) - one.GetAxis(3);

            if (!OverlapOnAxis(one, two, one.GetAxis(0), toCentre)) return false;
            if (!OverlapOnAxis(one, two, one.GetAxis(1), toCentre)) return false;
            if (!OverlapOnAxis(one, two, one.GetAxis(2), toCentre)) return false;

            if (!OverlapOnAxis(one, two, two.GetAxis(0), toCentre)) return false;
            if (!OverlapOnAxis(one, two, two.GetAxis(1), toCentre)) return false;
            if (!OverlapOnAxis(one, two, two.GetAxis(2), toCentre)) return false;

            if (!OverlapOnAxis(one, two, Vector3d.Cross(one.GetAxis(0), two.GetAxis(0)), toCentre)) return false;
            if (!OverlapOnAxis(one, two, Vector3d.Cross(one.GetAxis(0), two.GetAxis(1)), toCentre)) return false;
            if (!OverlapOnAxis(one, two, Vector3d.Cross(one.GetAxis(0), two.GetAxis(2)), toCentre)) return false;

            if (!OverlapOnAxis(one, two, Vector3d.Cross(one.GetAxis(1), two.GetAxis(0)), toCentre)) return false;
            if (!OverlapOnAxis(one, two, Vector3d.Cross(one.GetAxis(1), two.GetAxis(1)), toCentre)) return false;
            if (!OverlapOnAxis(one, two, Vector3d.Cross(one.GetAxis(1), two.GetAxis(2)), toCentre)) return false;

            if (!OverlapOnAxis(one, two, Vector3d.Cross(one.GetAxis(2), two.GetAxis(0)), toCentre)) return false;
            if (!OverlapOnAxis(one, two, Vector3d.Cross(one.GetAxis(2), two.GetAxis(1)), toCentre)) return false;
            if (!OverlapOnAxis(one, two, Vector3d.Cross(one.GetAxis(2), two.GetAxis(2)), toCentre)) return false;

            return true;
        }

        public static bool BoxAndHalfSpace(CollisionBox box, CollisionPlane plane)
        {
            // Work out the projected radius of the box onto the plane direction
            double projectedRadius = TransformToAxis(box, plane.Direction);

            // Work out how far the box is from the origin
            double boxDistance = Vector3d.Dot(plane.Direction, box.GetAxis(3)) - projectedRadius;

            // Check for the intersection
            return boxDistance <= plane.Offset;
        }

        /// <summary>
        /// This function checks if the two boxes overlap
        /// along the given axis.The final parameter toCentre
        /// is used to pass in the vector between the boxes centre
        /// points, to avoid having to recalculate it each time.
        /// </summary>
        private static bool OverlapOnAxis(CollisionBox one, CollisionBox two, Vector3d axis, Vector3d toCentre)
        {
            // Project the half-size of one onto axis
            double oneProject = TransformToAxis(one, axis);
            double twoProject = TransformToAxis(two, axis);

            // Project this onto the axis
            double distance = Vector3d.AbsDot(toCentre, axis);

            // Check for overlap
            return distance < oneProject + twoProject;
        }

        private static double TransformToAxis(CollisionBox box, Vector3d axis)
        {
            return
            box.HalfSize.x * Vector3d.AbsDot(axis, box.GetAxis(0)) +
            box.HalfSize.y * Vector3d.AbsDot(axis, box.GetAxis(1)) +
            box.HalfSize.z * Vector3d.AbsDot(axis, box.GetAxis(2));
        }
    }
}
