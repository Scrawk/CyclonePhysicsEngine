using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Particles
{
    public class Particle
    {
        /// <summary>
        /// Holds the inverse of the mass of the particle. It
        /// is more useful to hold the inverse mass because
        /// integration is simpler, and because in real time
        /// simulation it is more useful to have objects with
        /// infinite mass(immovable) than zero mass
        /// (completely unstable in numerical simulation).
        /// </summary>
        public double InverseMass;

        /// <summary>
        /// Returns true if the mass of the particle is finite.
        /// </summary>
        public bool HasFiniteMass => InverseMass != 0;

        /// <summary>
        /// Returns true if the mass of the particle is infinite.
        /// </summary>
        public bool HasInfiniteMass => InverseMass == 0;

        /// <summary>
        /// Holds the amount of damping applied to linear
        /// motion. Damping is required to remove energy added
        /// through numerical instability in the integrator.
        /// </summary>
        public double Damping = 0.99;

        /// <summary>
        /// Holds the linear position of the particle in
        /// world space.
        /// </summary>
        public Vector3d Position;

        /// <summary>
        /// Holds the linear velocity of the particle in
        /// world space.
        /// </summary>
        public Vector3d Velocity;

        /// <summary>
        /// Holds the accumulated force to be applied at the next
        /// simulation iteration only.This value is zeroed at each
        /// integration step.
        /// </summary>
        public Vector3d ForceAccum;

        /// <summary>
        /// Holds the acceleration of the particle.  This value
        /// can be used to set acceleration due to gravity(its primary
        /// use), or any other constant acceleration.
        /// </summary>
        public Vector3d Acceleration;

        /// <summary>
        /// Get the mass of the particle.
        /// </summary>
        /// <returns></returns>
        public double GetMass()
        {
            if (InverseMass == 0)
                return double.PositiveInfinity;
            else
                return 1.0 / InverseMass;
        }

        /// <summary>
        /// Set the mass of the particle. 
        /// </summary>
        public void SetMass(double mass)
        {
            if (mass <= 0)
                InverseMass = 0;
            else
                InverseMass = 1.0 / mass;
        }

        /// <summary>
        /// Accumalate force.
        /// </summary>
        public void AddForce(Vector3d force)
        {
            ForceAccum += force;
        }

        /// <summary>
        /// Clears the forces applied to the particle. This will be
        /// called automatically after each integration step.
        /// </summary>
        public void ClearAccumulator()
        {
            ForceAccum = Vector3d.Zero;
        }

        /// <summary>
        /// Integrates the particle forward in time by the given amount.
        /// This function uses a Newton-Euler integration method, which is a
        /// linear approximation to the correct integral. For this reason it
        /// may be inaccurate in some cases.
        /// </summary>
        public void Integrate(double dt)
        {
            // We don't integrate things with zero mass.
            if (InverseMass <= 0.0) return;
            if (dt <= 0) return;

            // Update linear position.
            Position += Velocity * dt;

            // Work out the acceleration from the force
            Vector3d resultingAcc = Acceleration;
            resultingAcc += ForceAccum * InverseMass;

            // Update linear velocity from the acceleration.
            Velocity += resultingAcc * dt;

            // Impose drag.
            Velocity *= Math.Pow(Damping, dt);

            // Clear the forces.
            ClearAccumulator();
        }

    }
}
