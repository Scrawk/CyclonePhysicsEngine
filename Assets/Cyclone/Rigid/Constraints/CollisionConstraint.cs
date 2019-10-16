using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Rigid.Collisions;

namespace Cyclone.Rigid.Constraints
{

    /// <summary>
    /// Find all collisions between objects by directly testing each object
    /// against all others in a O(N^2) operation with no broad phase 
    /// or any other optimization. Simple but slow.
    /// </summary>
    public class CollisionConstraint : RigidConstraint
    {

        ///<summary> 
        /// Holds the friction value to write into any collisions. 
        ///</summary>
        public double Friction;

        ///<summary> 
        /// Holds the restitution value to write into any collisions. 
        ///</summary>
        public double Restitution;

        ///<summary>
        /// Holds the collision tolerance, even uncolliding objects this
        /// close should have collisions generated.
        ///</summary>
        public double Tolerance;

        public List<CollisionPlane> Planes;

        public List<CollisionPrimitive> Primatives;

        public CollisionConstraint()
        {
            Planes = new List<CollisionPlane>();
            Primatives = new List<CollisionPrimitive>();
        }

        public override int AddContact(IList<RigidBody> bodies, IList<RigidContact> contacts, int next)
        {
            var data = new CollisionData();
            data.Contacts = contacts;
            data.Reset(next);
            data.Friction = Friction;
            data.Restitution = Restitution;
            data.Tolerance = Tolerance;

            foreach (var primative in Primatives)
                primative.CalculateInternals();

            foreach(var primative in Primatives)
            {
                if (data.NoMoreContacts()) break;

                switch(primative)
                {
                    case CollisionSphere sphere:
                        DetectCollisions(sphere, data);
                        break;

                    case CollisionBox box:
                        DetectCollisions(box, data);
                        break;
                }
            }

            return data.ContactCount;
        }

        private void DetectCollisions(CollisionSphere sphere, CollisionData data)
        {
            foreach (var plane in Planes)
                CollisionDetector.SphereAndHalfSpace(sphere, plane, data);

            foreach (var primative in Primatives)
            {
                if (primative == sphere) continue;
                if (data.NoMoreContacts()) break;

                switch (primative)
                {
                    case CollisionSphere sphere2:
                        CollisionDetector.SphereAndSphere(sphere, sphere2, data);
                        break;

                    case CollisionBox box:
                        CollisionDetector.BoxAndSphere(box, sphere, data);
                        break;
                }
            }
        }

        private void DetectCollisions(CollisionBox box, CollisionData data)
        {
            foreach (var plane in Planes)
                CollisionDetector.BoxAndHalfSpace(box, plane, data);

            foreach (var primative in Primatives)
            {
                if (primative == box) continue;
                if (data.NoMoreContacts()) break;

                switch (primative)
                {
                    case CollisionSphere sphere:
                        CollisionDetector.BoxAndSphere(box, sphere, data);
                        break;

                    case CollisionBox box2:
                        CollisionDetector.BoxAndBox(box, box2, data);
                        break;
                }
            }
        }

    }
}
