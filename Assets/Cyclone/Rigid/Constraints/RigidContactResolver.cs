using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Rigid.Constraints
{

    ///<summary>
    /// The contact resolution routine. One resolver instance
    /// can be shared for the whole simulation, as long as you need
    /// roughly the same parameters each time (which is normal).
    ///
    /// @section algorithm Resolution Algorithm
    ///
    /// The resolver uses an iterative satisfaction algorithm; it loops
    /// through each contact and tries to resolve it. Each contact is
    /// resolved locally, which may in turn put other contacts in a worse
    /// position. The algorithm then revisits other contacts and repeats
    /// the process up to a specified iteration limit. It can be proved
    /// that given enough iterations, the simulation will get to the
    /// correct result. As with all approaches, numerical stability can
    /// cause problems that make a correct resolution impossible.
    ///
    /// @subsection strengths Strengths
    ///
    /// This algorithm is very fast, much faster than other physics
    /// approaches. Even using many more iterations than there are
    /// contacts, it will be faster than global approaches.
    ///
    /// Many global algorithms are unstable under high friction, this
    /// approach is very robust indeed for high friction and low
    /// restitution values.
    ///
    /// The algorithm produces visually believable behaviour. Tradeoffs
    /// have been made to err on the side of visual realism rather than
    /// computational expense or numerical accuracy.
    ///
    /// @subsection weaknesses Weaknesses
    ///
    /// The algorithm does not cope well with situations with many
    /// inter-related contacts: stacked boxes, for example. In this
    /// case the simulation may appear to jiggle slightly, which often
    /// dislodges a box from the stack, allowing it to collapse.
    ///
    /// Another issue with the resolution mechanism is that resolving
    /// one contact may make another contact move sideways against
    /// friction, because each contact is handled independently, this
    /// friction is not taken into account. If one object is pushing
    /// against another, the pushed object may move across its support
    /// without friction, even though friction is set between those bodies.
    ///
    /// In general this resolver is not suitable for stacks of bodies,
    /// but is perfect for handling impact, explosive, and flat resting
    /// situations.
    ///</summary>
    public class RigidContactResolver
    {

        ///<summary>
        /// Holds the number of iterations to perform when resolving
        /// velocity.
        ///</summary>
        public int VelocityIterations;

        ///<summary>
        /// Holds the number of iterations to perform when resolving
        /// position.
        ///</summary>
        public int PositionIterations;

        ///<summary>
        /// To avoid instability velocities smaller
        /// than this value are considered to be zero. Too small and the
        /// simulation may be unstable, too large and the bodies may
        /// interpenetrate visually. A good starting point is the default
        /// of 0.01.
        ///</summary>
        public double VelocityEpsilon;

        ///<summary>
        /// To avoid instability penetrations
        /// smaller than this value are considered to be not interpenetrating.
        /// Too small and the simulation may be unstable, too large and the
        /// bodies may interpenetrate visually. A good starting point is
        /// the default of 0.01.
        ///</summary>
        public double PositionEpsilon;

        ///<summary>
        /// Stores the number of velocity iterations used in the
        /// last call to resolve contacts.
        ///</summary>
        public int VelocityIterationsUsed;

        ///<summary>
        /// Stores the number of position iterations used in the
        /// last call to resolve contacts.
        ///</summary>
        public int PositionIterationsUsed;

        ///<summary>
        /// Creates a new contact resolver.
        ///</summary>
        public RigidContactResolver(int iterations = 0)
        {
            VelocityIterations = iterations;
            PositionIterations = iterations;
            VelocityEpsilon = 0.001;
            PositionEpsilon = 0.001;
        }

        ///<summary>
        /// Resolves a set of contacts for both penetration and velocity.
        ///
        /// Contacts that cannot interact with
        /// each other should be passed to separate calls to resolveContacts,
        /// as the resolution algorithm takes much longer for lots of
        /// contacts than it does for the same number of contacts in small
        /// sets.
        ///
        /// @param contactArray Pointer to an array of contact objects.
        ///
        /// @param numContacts The number of contacts in the array to resolve.
        ///
        /// @param numIterations The number of iterations through the
        /// resolution algorithm. This should be at least the number of
        /// contacts (otherwise some constraints will not be resolved -
        /// although sometimes this is not noticable). If the iterations are
        /// not needed they will not be used, so adding more iterations may
        /// not make any difference. In some cases you would need millions
        /// of iterations. Think about the number of iterations as a bound:
        /// if you specify a large number, sometimes the algorithm WILL use
        /// it, and you may drop lots of frames.
        ///
        /// @param dt The duration of the previous integration step.
        /// This is used to compensate for forces applied.
        ///</summary>
        public void ResolveContacts(IList<RigidContact> contacts, int numContacts, double dt)
        {
            // Make sure we have something to do.
            if (numContacts == 0) return;

            if (VelocityIterations <= 0)
                VelocityIterations = numContacts * 4;

            if(PositionIterations <= 0)
                PositionIterations = numContacts * 4;

            // Prepare the contacts for processing
            PrepareContacts(contacts, numContacts, dt);

            // Resolve the interpenetration problems with the contacts.
            AdjustPositions(contacts, numContacts, dt);

            // Resolve the velocity problems with the contacts.
            AdjustVelocities(contacts, numContacts, dt);
        }

        ///<summary>
        /// Sets up contacts ready for processing. This makes sure their
        /// internal data is configured correctly and the correct set of bodies
        /// is made alive.
        ///</summary>
        public void PrepareContacts(IList<RigidContact> contacts, int numContacts, double dt)
        {
            // Generate contact velocity and axis information.
            for (int i = 0; i < numContacts; i++)
            {
                // Calculate the internal contact data (inertia, basis, etc).
                contacts[i].CalculateInternals(dt);
            }
        }

        ///<summary>
        /// Resolves the velocity issues with the given array of constraints,
        /// using the given number of iterations.
        ///</summary>
        public void AdjustVelocities(IList<RigidContact> contacts, int numContacts, double dt)
        {
            var c = contacts;
            Vector3d[] velocityChange = new Vector3d[2];
            Vector3d[] rotationChange = new Vector3d[2];
            Vector3d deltaVel;

            // iteratively handle impacts in order of severity.
            VelocityIterationsUsed = 0;
            while (VelocityIterationsUsed < VelocityIterations)
            {
                // Find contact with maximum magnitude of probable velocity change.
                double max = VelocityEpsilon;
                int index = numContacts;
                for (int i = 0; i < numContacts; i++)
                {
                    if (c[i].DesiredDeltaVelocity > max)
                    {
                        max = c[i].DesiredDeltaVelocity;
                        index = i;
                    }
                }

                if (index == numContacts) break;

                // Match the awake state at the contact
                c[index].MatchAwakeState();

                // Do the resolution on the contact that came out top.
                c[index].ApplyVelocityChange(velocityChange, rotationChange);

                // With the change in velocity of the two bodies, the update of
                // contact velocities means that some of the relative closing
                // velocities need recomputing.
                for (int i = 0; i < numContacts; i++)
                {
                    // Check each body in the contact
                    for (int b = 0; b < 2; b++)
                    {
                        if (c[i].Body[b] == null) continue;

                        // Check for a match with each body in the newly
                        // resolved contact
                        for (int d = 0; d < 2; d++)
                        {
                            if (c[i].Body[b] == c[index].Body[d])
                            {
                                deltaVel = velocityChange[d] + Vector3d.Cross(rotationChange[d], c[i].RelativeContactPosition[b]);

                                // The sign of the change is negative if we're dealing
                                // with the second body in a contact.
                                c[i].ContactVelocity += c[i].ContactToWorld.TransformTranspose(deltaVel) * (b != 0 ? -1 : 1);
                                c[i].CalculateDesiredDeltaVelocity(dt);
                            }
                        }
                    }
                }

                VelocityIterationsUsed++;
            }
        }

        ///<summary>
        /// Resolves the positional issues with the given array of constraints,
        /// using the given number of iterations.
        ///</summary>
        public void AdjustPositions(IList<RigidContact> contacts, int numContacts, double dt)
        {
            var c = contacts;
            int i, index;
            Vector3d[] linearChange = new Vector3d[2];

            Vector3d[] angularChange = new Vector3d[2];
            double max;
            Vector3d deltaPosition;

            // iteratively resolve interpenetrations in order of severity.
            PositionIterationsUsed = 0;
            while (PositionIterationsUsed < PositionIterations)
            {
                // Find biggest penetration
                max = PositionEpsilon;
                index = numContacts;
                for (i = 0; i < numContacts; i++)
                {
                    if (c[i].Penetration > max)
                    {
                        max = c[i].Penetration;
                        index = i;
                    }
                }

                if (index == numContacts) break;

                // Match the awake state at the contact
                c[index].MatchAwakeState();

                // Resolve the penetration.
                c[index].ApplyPositionChange(linearChange, angularChange, max);

                // Again this action may have changed the penetration of other
                // bodies, so we update contacts.
                for (i = 0; i < numContacts; i++)
                {
                    // Check each body in the contact
                    for (int b = 0; b < 2; b++)
                    {

                        if (c[i].Body[b] == null) continue;

                        // Check for a match with each body in the newly
                        // resolved contact
                        for (int d = 0; d < 2; d++)
                        {
                            if (c[i].Body[b] == c[index].Body[d])
                            {
                                deltaPosition = linearChange[d] + Vector3d.Cross(angularChange[d], c[i].RelativeContactPosition[b]);

                                // The sign of the change is positive if we're
                                // dealing with the second body in a contact
                                // and negative otherwise (because we're
                                // subtracting the resolution)..
                                c[i].Penetration += Vector3d.Dot(deltaPosition, c[i].ContactNormal) * (b != 0 ? 1 : -1);
                            }
                        }
                    }
                }

                PositionIterationsUsed++;
            }
        }

    }

}











