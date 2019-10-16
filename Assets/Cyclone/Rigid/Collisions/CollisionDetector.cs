using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Rigid.Collisions
{
    /// <summary>
    /// A wrapper class that holds the fine grained collision detection
    /// routines.
    ///
    /// Each of the functions has the same format: it takes the details
    /// of two objects, and a pointer to a contact array to fill.It
    /// returns the number of contacts it wrote into the array.
    /// </summary>
    public static class CollisionDetector
    {

        public static void SphereAndHalfSpace(CollisionSphere sphere, CollisionPlane plane, CollisionData data)
        {
            // Make sure we have contacts
            if (data.NoMoreContacts()) return;

            // Cache the sphere position
            Vector3d position = sphere.GetAxis(3);

            // Find the distance from the plane
            double dist = Vector3d.Dot(plane.Direction, position) - sphere.Radius - plane.Offset;

            if (dist >= 0) return;

            // Create the contact - it has a normal in the plane direction.
            var contact = data.GetContact();
            contact.ContactNormal = plane.Direction;
            contact.Penetration = -dist;
            contact.ContactPoint = position - plane.Direction * (dist + sphere.Radius);
            contact.SetBodyData(sphere.Body, null, data.Friction, data.Restitution);
        }

        public static void SphereAndTruePlane(CollisionSphere sphere, CollisionPlane plane, CollisionData data)
        {
            // Make sure we have contacts
            if (data.NoMoreContacts()) return;

            // Cache the sphere position
            Vector3d position = sphere.GetAxis(3);

            // Find the distance from the plane
            double centreDistance = Vector3d.Dot(plane.Direction, position) - plane.Offset;

            // Check if we're within radius
            if (centreDistance * centreDistance > sphere.Radius * sphere.Radius)
                return;

            // Check which side of the plane we're on
            Vector3d normal = plane.Direction;
            double penetration = -centreDistance;
            if (centreDistance < 0)
            {
                normal *= -1;
                penetration = -penetration;
            }
            penetration += sphere.Radius;

            // Create the contact - it has a normal in the plane direction.
            var contact = data.GetContact();
            contact.ContactNormal = normal;
            contact.Penetration = penetration;
            contact.ContactPoint = position - plane.Direction * centreDistance;
            contact.SetBodyData(sphere.Body, null, data.Friction, data.Restitution);
        }

        public static void SphereAndSphere(CollisionSphere one, CollisionSphere two, CollisionData data)
        {
            // Make sure we have contacts
            if (data.NoMoreContacts()) return;

            // Cache the sphere positions
            Vector3d positionOne = one.GetAxis(3);
            Vector3d positionTwo = two.GetAxis(3);

            // Find the vector between the objects
            Vector3d midline = positionOne - positionTwo;
            double size = midline.Magnitude;

            // See if it is large enough.
            if (size <= 0.0f || size >= one.Radius + two.Radius)
                return;

            // We manually create the normal, because we have the
            // size to hand.
            Vector3d normal = midline * (1.0 / size);

            var contact = data.GetContact();
            contact.ContactNormal = normal;
            contact.ContactPoint = positionOne + midline * 0.5;
            contact.Penetration = one.Radius + two.Radius - size;
            contact.SetBodyData(one.Body, two.Body, data.Friction, data.Restitution);
        }

        public static void BoxAndHalfSpace(CollisionBox box, CollisionPlane plane, CollisionData data)
        {
            // Make sure we have contacts
            if (data.NoMoreContacts()) return;

            // Check for intersection
            if (!IntersectionTests.BoxAndHalfSpace(box, plane))
                return;

            // We have an intersection, so find the intersection points. We can make
            // do with only checking vertices. If the box is resting on a plane
            // or on an edge, it will be reported as four or two contact points.

            // Go through each combination of + and - for each half-size
            int[,] mults = new int[,]
            {
                {1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
                {1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}
            };

            for (int i = 0; i < 8; i++)
            {
                // Calculate the position of each vertex
                Vector3d vertexPos = new Vector3d(mults[i, 0], mults[i, 1], mults[i, 2]);
                vertexPos *= box.HalfSize;
                vertexPos = box.Transform.Transform(vertexPos);

                // Calculate the distance from the plane
                double vertexDistance = Vector3d.Dot(vertexPos, plane.Direction);

                // Compare this to the plane's distance
                if (vertexDistance <= plane.Offset)
                {
                    // Create the contact data.

                    // The contact point is halfway between the vertex and the
                    // plane - we multiply the direction by half the separation
                    // distance and add the vertex location.
                    var contact = data.GetContact();
                    contact.ContactPoint = plane.Direction;
                    contact.ContactPoint *= (vertexDistance - plane.Offset);
                    contact.ContactPoint += vertexPos;
                    contact.ContactNormal = plane.Direction;
                    contact.Penetration = plane.Offset - vertexDistance;
                    contact.SetBodyData(box.Body, null, data.Friction, data.Restitution);

                    if (!data.HasMoreContacts()) return;
                }
            }
        }

        public static void BoxAndBox(CollisionBox one, CollisionBox two, CollisionData data)
        {
            // Make sure we have contacts
            if (data.NoMoreContacts()) return;
            //if (!IntersectionTests.BoxAndBox(one, two)) return;

            // Find the vector between the two centres
            Vector3d toCentre = two.GetAxis(3) - one.GetAxis(3);

            // We start assuming there is no contact
            double pen = double.MaxValue;
            int best = 0xffffff;

            // Now we check each axes, returning if it gives us
            // a separating axis, and keeping track of the axis with
            // the smallest penetration otherwise.
            if (!TryAxis(one, two, one.GetAxis(0), toCentre, 0, ref pen, ref best)) return;
            if (!TryAxis(one, two, one.GetAxis(1), toCentre, 1, ref pen, ref best)) return;
            if (!TryAxis(one, two, one.GetAxis(2), toCentre, 2, ref pen, ref best)) return;

            if (!TryAxis(one, two, two.GetAxis(0), toCentre, 3, ref pen, ref best)) return;
            if (!TryAxis(one, two, two.GetAxis(1), toCentre, 4, ref pen, ref best)) return;
            if (!TryAxis(one, two, two.GetAxis(2), toCentre, 5, ref pen, ref best)) return;

            // Store the best axis-major, in case we run into almost
            // parallel edge collisions later
            int bestSingleAxis = best;

            if (!TryAxis(one, two, Vector3d.Cross(one.GetAxis(0), two.GetAxis(0)), toCentre, 6, ref pen, ref best)) return;
            if (!TryAxis(one, two, Vector3d.Cross(one.GetAxis(0), two.GetAxis(1)), toCentre, 7, ref pen, ref best)) return;
            if (!TryAxis(one, two, Vector3d.Cross(one.GetAxis(0), two.GetAxis(2)), toCentre, 8, ref pen, ref best)) return;
            if (!TryAxis(one, two, Vector3d.Cross(one.GetAxis(1), two.GetAxis(0)), toCentre, 9, ref pen, ref best)) return;
            if (!TryAxis(one, two, Vector3d.Cross(one.GetAxis(1), two.GetAxis(1)), toCentre, 10, ref pen, ref best)) return;
            if (!TryAxis(one, two, Vector3d.Cross(one.GetAxis(1), two.GetAxis(2)), toCentre, 11, ref pen, ref best)) return;
            if (!TryAxis(one, two, Vector3d.Cross(one.GetAxis(2), two.GetAxis(0)), toCentre, 12, ref pen, ref best)) return;
            if (!TryAxis(one, two, Vector3d.Cross(one.GetAxis(2), two.GetAxis(1)), toCentre, 13, ref pen, ref best)) return;
            if (!TryAxis(one, two, Vector3d.Cross(one.GetAxis(2), two.GetAxis(2)), toCentre, 14, ref pen, ref best)) return;

            // Make sure we've got a result.
            if (best == 0xffffff)
                throw new Exception("best == 0xffffff");

            // We now know there's a collision, and we know which
            // of the axes gave the smallest penetration. We now
            // can deal with it in different ways depending on
            // the case.
            if (best < 3)
            {
                // We've got a vertex of box two on a face of box one.
                FillPointFaceBoxBox(one, two, toCentre, data, best, pen);
            }
            else if (best < 6)
            {
                // We've got a vertex of box one on a face of box two.
                // We use the same algorithm as above, but swap around
                // one and two (and therefore also the vector between their
                // centres).
                FillPointFaceBoxBox(two, one, toCentre * -1.0, data, best - 3, pen);
            }
            else
            {
                // We've got an edge-edge contact. Find out which axes
                best -= 6;
                int oneAxisIndex = best / 3;
                int twoAxisIndex = best % 3;
                Vector3d oneAxis = one.GetAxis(oneAxisIndex);
                Vector3d twoAxis = two.GetAxis(twoAxisIndex);
                Vector3d axis = Vector3d.Cross(oneAxis, twoAxis);
                axis.Normalize();

                // The axis should point from box one to box two.
                if (Vector3d.Dot(axis, toCentre) > 0) axis *= -1.0;

                // We have the axes, but not the edges: each axis has 4 edges parallel
                // to it, we need to find which of the 4 for each object. We do
                // that by finding the point in the centre of the edge. We know
                // its component in the direction of the box's collision axis is zero
                // (its a mid-point) and we determine which of the extremes in each
                // of the other axes is closest.
                Vector3d ptOnOneEdge = one.HalfSize;
                Vector3d ptOnTwoEdge = two.HalfSize;
                for (int i = 0; i < 3; i++)
                {
                    if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
                    else if (Vector3d.Dot(one.GetAxis(i), axis) > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

                    if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
                    else if (Vector3d.Dot(two.GetAxis(i), axis) < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
                }

                // Move them into world coordinates (they are already oriented
                // correctly, since they have been derived from the axes).
                ptOnOneEdge = one.Transform * ptOnOneEdge;
                ptOnTwoEdge = two.Transform * ptOnTwoEdge;

                // So we have a point and a direction for the colliding edges.
                // We need to find out point of closest approach of the two
                // line-segments.
                Vector3d vertex = ContactPoint(
                    ptOnOneEdge, oneAxis, one.HalfSize[oneAxisIndex],
                    ptOnTwoEdge, twoAxis, two.HalfSize[twoAxisIndex],
                    bestSingleAxis > 2
                    );

                // We can fill the contact.
                var contact = data.GetContact();

                contact.Penetration = pen;
                contact.ContactNormal = axis;
                contact.ContactPoint = vertex;
                contact.SetBodyData(one.Body, two.Body, data.Friction, data.Restitution);
            }
        }

        public static void BoxAndPoint(CollisionBox box, Vector3d point, CollisionData data)
        {
            // Transform the point into box coordinates
            Vector3d relPt = box.Transform.TransformInverse(point);

            Vector3d normal;

            // Check each axis, looking for the axis on which the
            // penetration is least deep.
            double min_depth = box.HalfSize.x - Math.Abs(relPt.x);
            if (min_depth < 0) return;
            normal = box.GetAxis(0) * ((relPt.x < 0) ? -1 : 1);

            double depth = box.HalfSize.y - Math.Abs(relPt.y);
            if (depth < 0) return;
            else if (depth < min_depth)
            {
                min_depth = depth;
                normal = box.GetAxis(1) * ((relPt.y < 0) ? -1 : 1);
            }

            depth = box.HalfSize.z - Math.Abs(relPt.z);
            if (depth < 0) return;
            else if (depth < min_depth)
            {
                min_depth = depth;
                normal = box.GetAxis(2) * ((relPt.z < 0) ? -1 : 1);
            }

            // Compile the contact
            var contact = data.GetContact();
            contact.ContactNormal = normal;
            contact.ContactPoint = point;
            contact.Penetration = min_depth;

            // Note that we don't know what rigid body the point
            // belongs to, so we just use NULL. Where this is called
            // this value can be left, or filled in.
            contact.SetBodyData(box.Body, null, data.Friction, data.Restitution);
        }

        public static void BoxAndSphere(CollisionBox box, CollisionSphere sphere, CollisionData data)
        {
            // Make sure we have contacts
            if (data.NoMoreContacts()) return;

            // Transform the centre of the sphere into box coordinates
            Vector3d centre = sphere.GetAxis(3);
            Vector3d relCentre = box.Transform.TransformInverse(centre);

            // Early out check to see if we can exclude the contact
            if (Math.Abs(relCentre.x) - sphere.Radius > box.HalfSize.x ||
                Math.Abs(relCentre.y) - sphere.Radius > box.HalfSize.y ||
                Math.Abs(relCentre.z) - sphere.Radius > box.HalfSize.z)
            {
                return;
            }

            Vector3d closestPt = new Vector3d(0,0,0);
            double dist;

            // Clamp each coordinate to the box.
            dist = relCentre.x;
            if (dist > box.HalfSize.x) dist = box.HalfSize.x;
            if (dist < -box.HalfSize.x) dist = -box.HalfSize.x;
            closestPt.x = dist;

            dist = relCentre.y;
            if (dist > box.HalfSize.y) dist = box.HalfSize.y;
            if (dist < -box.HalfSize.y) dist = -box.HalfSize.y;
            closestPt.y = dist;

            dist = relCentre.z;
            if (dist > box.HalfSize.z) dist = box.HalfSize.z;
            if (dist < -box.HalfSize.z) dist = -box.HalfSize.z;
            closestPt.z = dist;

            // Check we're in contact
            dist = (closestPt - relCentre).SqrMagnitude;
            if (dist > sphere.Radius * sphere.Radius) return;

            // Compile the contact
            Vector3d closestPtWorld = box.Transform.Transform(closestPt);

            var contact = data.GetContact();
            contact.ContactNormal = (closestPtWorld - centre).Normalized;
            contact.ContactPoint = closestPtWorld;
            contact.Penetration = sphere.Radius - Math.Sqrt(dist);
            contact.SetBodyData(box.Body, sphere.Body, data.Friction, data.Restitution);
        }

        private static bool TryAxis(CollisionBox one, CollisionBox two, Vector3d axis, Vector3d toCentre, int index, ref double smallestPenetration, ref int smallestCase)
        {
            // Make sure we have a normalized axis, and don't check almost parallel axes
            if (axis.SqrMagnitude < 0.0001) return true;
            axis.Normalize();

            double penetration = PenetrationOnAxis(one, two, axis, toCentre);

            if (penetration < 0) return false;
            if (penetration < smallestPenetration)
            {
                smallestPenetration = penetration;
                smallestCase = index;
            }
            return true;
        }

        /// <summary>
        /// This function checks if the two boxes overlap
        /// along the given axis, returning the ammount of overlap.
        /// The final parameter toCentre
        /// is used to pass in the vector between the boxes centre
        /// points, to avoid having to recalculate it each time.
        /// </summary>
        private static double PenetrationOnAxis(CollisionBox one, CollisionBox two, Vector3d axis, Vector3d toCentre)
        {
            // Project the half-size of one onto axis
            double oneProject = TransformToAxis(one, axis);
            double twoProject = TransformToAxis(two, axis);

            // Project this onto the axis
            double distance = Vector3d.AbsDot(toCentre, axis);

            // Return the overlap (i.e. positive indicates
            // overlap, negative indicates separation).
            return oneProject + twoProject - distance;
        }

        private static double TransformToAxis(CollisionBox box, Vector3d axis)
        {
            return
            box.HalfSize.x * Vector3d.AbsDot(axis, box.GetAxis(0)) +
            box.HalfSize.y * Vector3d.AbsDot(axis, box.GetAxis(1)) +
            box.HalfSize.z * Vector3d.AbsDot(axis, box.GetAxis(2));
        }

        private static void FillPointFaceBoxBox(CollisionBox one, CollisionBox two, Vector3d toCentre, CollisionData data, int best, double pen)
        {
            // This method is called when we know that a vertex from
            // box two is in contact with box one.
            var contact = data.GetContact();

            // We know which axis the collision is on (i.e. best),
            // but we need to work out which of the two faces on
            // this axis.
            Vector3d normal = one.GetAxis(best);
            if (Vector3d.Dot(one.GetAxis(best), toCentre) > 0)
            {
                normal = normal * -1.0f;
            }

            // Work out which vertex of box two we're colliding with.
            // Using toCentre doesn't work!
            Vector3d vertex = two.HalfSize;
            if (Vector3d.Dot(two.GetAxis(0), normal) < 0) vertex.x = -vertex.x;
            if (Vector3d.Dot(two.GetAxis(1), normal) < 0) vertex.y = -vertex.y;
            if (Vector3d.Dot(two.GetAxis(2), normal) < 0) vertex.z = -vertex.z;

            // Create the contact data
            contact.ContactNormal = normal;
            contact.Penetration = pen;
            contact.ContactPoint = two.Transform * vertex;
            contact.SetBodyData(one.Body, two.Body, data.Friction, data.Restitution);
        }

        private static Vector3d ContactPoint(Vector3d pOne, Vector3d dOne, double oneSize, Vector3d pTwo, Vector3d dTwo, double twoSize, bool useOne)
        {
            // If useOne is true, and the contact point is outside
            // the edge (in the case of an edge-face contact) then
            // we use one's midpoint, otherwise we use two's.

            Vector3d toSt, cOne, cTwo;
            double dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
            double denom, mua, mub;

            smOne = dOne.SqrMagnitude;
            smTwo = dTwo.SqrMagnitude;
            dpOneTwo = Vector3d.Dot(dTwo, dOne);

            toSt = pOne - pTwo;
            dpStaOne = Vector3d.Dot(dOne, toSt);
            dpStaTwo = Vector3d.Dot(dTwo, toSt);

            denom = smOne * smTwo - dpOneTwo * dpOneTwo;

            // Zero denominator indicates parrallel lines
            if (Math.Abs(denom) < 0.0001)
                return useOne ? pOne : pTwo;

            mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
            mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

            // If either of the edges has the nearest point out
            // of bounds, then the edges aren't crossed, we have
            // an edge-face contact. Our point is on the edge, which
            // we know from the useOne parameter.
            if (mua > oneSize || mua < -oneSize || mub > twoSize || mub < -twoSize)
            {
                return useOne ? pOne : pTwo;
            }
            else
            {
                cOne = pOne + dOne * mua;
                cTwo = pTwo + dTwo * mub;

                return cOne * 0.5 + cTwo * 0.5;
            }
        }

    }
}
