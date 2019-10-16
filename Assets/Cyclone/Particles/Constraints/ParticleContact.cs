using System;
using System.Collections.Generic;

using Cyclone.Core;
using Cyclone.Particles;

namespace Cyclone.Particles.Constraints
{
    /// <summary>
    /// A Contact represents two objects in contact (in this case
    /// ParticleContact representing two Particles). Resolving a
    /// contact removes their interpenetration, and applies sufficient
    /// impulse to keep them apart.Colliding bodies may also rebound.
    ///
    /// The contact has no callable functions, it just holds the
    /// contact details. To resolve a set of contacts, use the particle
    /// contact resolver class.
    /// </summary>
    public class ParticleContact
    {
        /// <summary>
        /// Holds the particles that are involved in the contact. The
        /// second of these can be NULL, for contacts with the scenery.
        /// </summary>
        public Particle[] Particles;

        /// <summary>
        /// Holds the normal restitution coefficient at the contact.
        /// </summary>
        public double Restitution;

        /// <summary>
        /// Holds the direction of the contact in world coordinates.
        /// </summary>
        public Vector3d ContactNormal;

        /// <summary>
        /// Holds the depth of penetration at the contact.
        /// </summary>
        public double Penetration;

        /// <summary>
        /// Holds the amount each particle is moved by during 
        /// interpenetration resolution.
        /// </summary>
        public Vector3d[] ParticlesMovement;

        /// <summary>
        /// 
        /// </summary>
        public ParticleContact()
        {
            Particles = new Particle[2];
            ParticlesMovement = new Vector3d[2];
        }

        /// <summary>
        /// 
        /// </summary>
        public void Clear()
        {
            Particles[0] = null;
            Particles[1] = null;
        }

        /// <summary>
        /// Resolves this contact, for both velocity and interpenetration.
        /// </summary>
        /// <param name="dt"></param>
        public void Resolve(double dt)
        {
            ResolveVelocity(dt);
            ResolveInterpenetration(dt);
        }

        /// <summary>
        /// Calculates the separating velocity at this contact.
        /// </summary>
        /// <returns></returns>
        public double CalculateSeparatingVelocity()
        {
            Vector3d relativeVelocity = Particles[0].Velocity;
            if (Particles[1] != null)
                relativeVelocity -= Particles[1].Velocity;

            return Vector3d.Dot(relativeVelocity, ContactNormal);
        }

        /// <summary>
        /// Handles the impulse calculations for this collision.
        /// </summary>
        /// <param name="dt"></param>
        private void ResolveVelocity(double dt)
        {
            // Find the velocity in the direction of the contact
            double separatingVelocity = CalculateSeparatingVelocity();

            // Check if it needs to be resolved
            if (separatingVelocity > 0)
            {
                // The contact is either separating, or stationary - there's
                // no impulse required.
                return;
            }

            // Calculate the new separating velocity
            double newSepVelocity = -separatingVelocity * Restitution;

            // Check the velocity build-up due to acceleration only
            Vector3d accCausedVelocity = Particles[0].Acceleration;
            if (Particles[1] != null) accCausedVelocity -= Particles[1].Acceleration;
            double accCausedSepVelocity = Vector3d.Dot(accCausedVelocity, ContactNormal) * dt;

            // If we've got a closing velocity due to acceleration build-up,
            // remove it from the new separating velocity
            if (accCausedSepVelocity < 0)
            {
                newSepVelocity += Restitution * accCausedSepVelocity;

                // Make sure we haven't removed more than was
                // there to remove.
                if (newSepVelocity < 0) newSepVelocity = 0;
            }

            double deltaVelocity = newSepVelocity - separatingVelocity;

            // We apply the change in velocity to each object in proportion to
            // their inverse mass (i.e. those with lower inverse mass [higher
            // actual mass] get less change in velocity)..
            double totalInverseMass = Particles[0].InverseMass;
            if (Particles[1] != null) totalInverseMass += Particles[1].InverseMass;

            // If all m_particless have infinite mass, then impulses have no effect
            if (totalInverseMass <= 0) return;

            // Calculate the impulse to apply
            double impulse = deltaVelocity / totalInverseMass;

            // Find the amount of impulse per unit of inverse mass
            Vector3d impulsePerIMass = ContactNormal * impulse;

            // Apply impulses: they are applied in the direction of the contact,
            // and are proportional to the inverse mass.
            Particles[0].Velocity += impulsePerIMass * Particles[0].InverseMass;

            if (Particles[1] != null)
            {
                // Particle 1 goes in the opposite direction
                Particles[1].Velocity += impulsePerIMass * -Particles[1].InverseMass;
            }
        }

        /// <summary>
        /// Handles the interpenetration resolution for this contact.
        /// </summary>
        /// <param name="dt"></param>
        private void ResolveInterpenetration(double dt)
        {
            // If we don't have any penetration, skip this step.
            if (Penetration <= 0) return;

            // The movement of each object is based on their inverse mass, so
            // total that.
            double totalInverseMass = Particles[0].InverseMass;
            if (Particles[1] != null) totalInverseMass += Particles[1].InverseMass;

            // If all m_particless have infinite mass, then we do nothing
            if (totalInverseMass <= 0) return;

            // Find the amount of penetration resolution per unit of inverse mass
            Vector3d movePerIMass = ContactNormal * (Penetration / totalInverseMass);

            // Calculate the the movement amounts
            ParticlesMovement[0] = movePerIMass * Particles[0].InverseMass;
            if (Particles[1] != null)
                ParticlesMovement[1] = movePerIMass * -Particles[1].InverseMass;
            else
                ParticlesMovement[1] = Vector3d.Zero;

            // Apply the penetration resolution
            Particles[0].Position += ParticlesMovement[0];
            if (Particles[1] != null)
                Particles[1].Position += ParticlesMovement[1];
        }

    }
}
