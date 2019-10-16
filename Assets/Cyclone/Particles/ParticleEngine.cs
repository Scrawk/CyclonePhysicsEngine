using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Particles.Forces;
using Cyclone.Particles.Constraints;

namespace Cyclone.Particles
{
    public class ParticleEngine
    {
        /// <summary>
        /// Holds the particles.
        /// </summary>
        public List<Particle> Particles;

        /// <summary>
        /// Holds the force areas for the in this world.
        /// </summary>
        public List<ParticleForceArea> ForceAreas;

        /// <summary>
        /// Holds the forces for the particles in this world.
        /// </summary>
        public List<ParticleForce> Forces;

        /// <summary>
        /// Contact generators.
        /// </summary>
        public List<ParticleConstraint> Constraints;

        /// <summary>
        /// Holds the resolver for contacts.
        /// </summary>
        public ParticleContactResolver Resolver;

        /// <summary>
        /// Holds the list of contacts.
        /// </summary>
        private ParticleContact[] m_contacts;

        /// <summary>
        /// Creates a new particle simulator that can handle up to the
        /// given number of contacts per frame.You can also optionally
        /// give a number of contact-resolution iterations to use. If you
        /// don't give a number of iterations, then twice the number of
        /// contacts will be used.
        /// </summary>
        public ParticleEngine(int maxContacts)
        {
            Particles = new List<Particle>();
            ForceAreas = new List<ParticleForceArea>();
            Forces = new List<ParticleForce>();
            Constraints = new List<ParticleConstraint>();
            Resolver = new ParticleContactResolver();

            m_contacts = new ParticleContact[maxContacts];
            for (int i = 0; i < maxContacts; i++)
                m_contacts[i] = new ParticleContact();
        }

        /// <summary>
        /// Initializes the world for a simulation frame. This clears
        /// the force accumulators for particles in the world.After
        /// calling this, the particles can have their forces for this
        /// frame added.
        /// </summary>
        public void StartFrame()
        {
            foreach (var p in Particles)
                p.ClearAccumulator();
        }

        /// <summary>
        /// Processes all the physics for the particle world.
        /// </summary>
        /// <param name="dt"></param>
        public void RunPhysics(double dt)
        {
            // First apply the forces
            ApplyForces(dt);

            // Then integrate the objects
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
                foreach(var p in Particles)
                    f.UpdateForce(p, dt);
            }
                
            foreach (var f in Forces)
                f.UpdateForce(dt);
        }

        /// <summary>
        /// Calls each of the registered contact generators to report
        /// their contacts.Returns the number of generated contacts.
        /// </summary>
        /// <returns></returns>
        private int GenerateContacts()
        {
            ClearContacts();

            int limit = m_contacts.Length;
            int nextContact = 0;

            foreach(var g in Constraints)
            {
                int used = g.AddContact(Particles, m_contacts, nextContact);
                limit -= used;
                nextContact += used;

                // We've run out of contacts to fill. This means we're missing contacts.
                if (limit <= 0) break;
            }

            // Return the number of contacts used.
            return m_contacts.Length - limit;
        }

        /// <summary>
        /// Integrates all the particles in this world forward in time
        /// by the given duration.
        /// </summary>
        /// <param name="dt"></param>
        private void Integrate(double dt)
        {
            foreach (var p in Particles)
                p.Integrate(dt);
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
