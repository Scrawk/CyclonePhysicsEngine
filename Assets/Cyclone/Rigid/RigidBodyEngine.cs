using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Rigid.Constraints;
using Cyclone.Rigid.Forces;

namespace Cyclone.Rigid
{

    ///<summary>
    /// The world represents an independent simulation of physics.  It
    /// keeps track of a set of rigid bodies, and provides the means to
    /// update them all.
    ///</summary>
    public class RigidBodyEngine
    {
        ///<summary>
        /// The list of bodies.
        ///</summary>
        public List<RigidBody> Bodies;

        /// <summary>
        /// Holds the force areas for the in this world.
        /// </summary>
        public List<RigidForceArea> ForceAreas;

        /// <summary>
        /// Holds the forces for the particles in this world.
        /// </summary>
        public List<RigidForce> Forces;

        ///<summary>
        /// The list of contact generators.
        ///<summary>
        public List<RigidConstraint> Constraints;

        /// <summary>
        /// The collision geometry.
        /// </summary>
        public CollisionConstraint Collisions;

        ///<summary>
        /// Holds the resolver for sets of contacts.
        ///<summary>
        public RigidContactResolver Resolver;

        ///<summary>
        /// Holds an array of contacts, for filling by the contact
        /// generators.
        ///<summary>
        private RigidContact[] m_contacts;

        ///<summary>
        /// Creates a new simulator that can handle up to the given
        /// number of contacts per frame. You can also optionally give
        /// a number of contact-resolution iterations to use. If you
        /// don't give a number of iterations, then four times the
        /// number of detected contacts will be used for each frame.
        ///<summary>
        public RigidBodyEngine(int maxContacts)
        {
            Bodies = new List<RigidBody>();
            ForceAreas = new List<RigidForceArea>();
            Forces = new List<RigidForce>();
            Constraints = new List<RigidConstraint>();
            Resolver = new RigidContactResolver();

            Collisions = new CollisionConstraint();
            Constraints.Add(Collisions);

            m_contacts = new RigidContact[maxContacts];
            for (int i = 0; i < maxContacts; i++)
                m_contacts[i] = new RigidContact();
        }

        ///<summary>
        /// Initialises the world for a simulation frame. This clears
        /// the force and torque accumulators for bodies in the
        /// world. After calling this, the bodies can have their forces
        /// and torques for this frame added.
        ///<summary>
        public void StartFrame()
        {
            foreach (var body in Bodies)
            {
                // Remove all forces from the accumulator
                body.ClearAccumulators();
                body.CalculateDerivedData();
            }
        }

        ///<summary>
        /// Processes all the physics for the world.
        ///<summary>
        public void RunPhysics(double dt)
        {
            // First apply the force.s
            ApplyForces(dt);

            Integrate(dt);

            // Generate contacts
            int usedContacts = GenerateContacts();

            // And process them
            if (usedContacts > 0)
                Resolver.ResolveContacts(m_contacts, usedContacts, dt);
        }

        /// <summary>
        /// Apply the forces.
        /// </summary>
        /// <param name="dt"></param>
        private void ApplyForces(double dt)
        {
            foreach (var f in ForceAreas)
            {
                foreach (var b in Bodies)
                    f.UpdateForce(b, dt);
            }

            foreach (var f in Forces)
                f.UpdateForce(dt);
        }

        ///<summary>
        /// Calls each of the registered contact generators to report
        /// their contacts. Returns the number of generated contacts.
        ///<summary>
        private int GenerateContacts()
        {
            ClearContacts();

            int limit = m_contacts.Length;
            int nextContact = 0;

            foreach (var gen in Constraints)
            {
                int used = gen.AddContact(Bodies, m_contacts, nextContact);
                limit -= used;
                nextContact += used;

                // We've run out of contacts to fill. This means we're missing
                // contacts.
                if (limit <= 0) break;
            }

            // Return the number of contacts used.
            return m_contacts.Length - limit;
        }

        /// <summary>
        /// Integrates all the bodies in this world forward in time
        /// by the given dt.
        /// </summary>
        /// <param name="dt"></param>
        private void Integrate(double dt)
        {
            foreach (var b in Bodies)
                b.Integrate(dt);
        }

        /// <summary>
        /// 
        /// </summary>
        private void ClearContacts()
        {
            foreach (var contact in m_contacts)
                contact.Clear();
        }

    }

}