using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Particles;

namespace Cyclone.Particles.Constraints
{
    /// <summary>
    /// The contact resolution routine for particle contacts. One
    /// resolver instance can be shared for the whole simulation.
    /// 
    /// The resolver uses an iterative satisfaction algorithm; it loops
    /// through each contact and tries to resolve it.This is a very fast
    /// algorithm but can be unstable when the contacts are highly
    /// inter-related.
    /// </summary>
    public class ParticleContactResolver
    {

        /// <summary>
        /// 
        /// </summary>
        public ParticleContactResolver(int iterations = 0)
        {
            Iterations = iterations;
        }

        /// <summary>
        /// The number of iterations through the
        /// resolution algorithm.This should be at least the number of
        /// contacts (otherwise some constraints will not be resolved -
        /// although sometimes this is not noticable). If the
        /// iterations are not needed they will not be used, so adding
        /// more iterations may not make any difference.But in some
        /// cases you would need millions of iterations.Think about
        /// the number of iterations as a bound: if you specify a large
        /// number, sometimes the algorithm WILL use it, and you may
        /// drop frames.
        /// </summary>
        public int Iterations;

        /// <summary>
        /// This is a performance tracking value - we keep a record
        /// of the actual number of iterations used.
        /// </summary>
        public int IterationsUsed;

        /// <summary>
        /// Resolves a set of particle contacts for both penetration
        /// and velocity.
        ///
        /// Contacts that cannot interact with each other should be
        /// passed to separate calls to resolveContacts, as the
        /// resolution algorithm takes much longer for lots of contacts
        /// than it does for the same number of contacts in small sets.
        /// </summary>
        /// <param name="contacts"></param>
        /// <param name="dt"></param>
        public void ResolveContacts(IList<ParticleContact> contacts, int numContacts, double dt)
        {
            if (numContacts == 0) return;

            if (Iterations <= 0)
                Iterations = numContacts * 2;

            IterationsUsed = 0;
            while (IterationsUsed < Iterations)
            {
                // Find the contact with the largest closing velocity.
                double max = double.PositiveInfinity;
                int maxIndex = numContacts;
                for (int i = 0; i < numContacts; i++)
                {
                    double sepVel = contacts[i].CalculateSeparatingVelocity();
                    if (sepVel < max && (sepVel < 0 || contacts[i].Penetration > 0))
                    {
                        max = sepVel;
                        maxIndex = i;
                    }
                }

                // Do we have anything worth resolving?
                if (maxIndex == numContacts) break;

                // Resolve this contact
                contacts[maxIndex].Resolve(dt);

                // Update the interpenetrations for all particles
                Vector3d[] move = contacts[maxIndex].ParticlesMovement;
                for (int i = 0; i < numContacts; i++)
                {
                    if (contacts[i].Particles[0] == contacts[maxIndex].Particles[0])
                    {
                        contacts[i].Penetration -= Vector3d.Dot(move[0], contacts[i].ContactNormal);
                    }
                    else if (contacts[i].Particles[0] == contacts[maxIndex].Particles[1])
                    {
                        contacts[i].Penetration -= Vector3d.Dot(move[1], contacts[i].ContactNormal);
                    }

                    if (contacts[i].Particles[1] != null)
                    {
                        if (contacts[i].Particles[1] == contacts[maxIndex].Particles[0])
                        {
                            contacts[i].Penetration += Vector3d.Dot(move[0], contacts[i].ContactNormal);
                        }
                        else if (contacts[i].Particles[1] == contacts[maxIndex].Particles[1])
                        {
                            contacts[i].Penetration += Vector3d.Dot(move[1], contacts[i].ContactNormal);
                        }
                    }
                }

                IterationsUsed++;
            }
        }
    }
}
