using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Rigid.Constraints
{

    ///<summary>
    /// A contact represents two bodies in contact. Resolving a
    /// contact removes their interpenetration, and applies sufficient
    /// impulse to keep them apart. Colliding bodies may also rebound.
    /// Contacts can be used to represent positional joints, by making
    /// the contact constraint keep the bodies in their correct
    /// orientation.
    ///
    /// It can be a good idea to create a contact object even when the
    /// contact isn't violated. Because resolving one contact can violate
    /// another, contacts that are close to being violated should be
    /// sent to the resolver; that way if one resolution moves the body,
    /// the contact may be violated, and can be resolved. If the contact
    /// is not violated, it will not be resolved, so you only loose a
    /// small amount of execution time.
    ///</summary>
    public class RigidContact
    {

        ///<summary>
        /// Holds the bodies that are involved in the contact. The
        /// second of these can be NULL, for contacts with the scenery.
        ///</summary>
        public RigidBody[] Body = new RigidBody[2];

        ///<summary>
        /// Holds the lateral friction coefficient at the contact.
        ///</summary>
        public double Friction;

        ///<summary>
        /// Holds the normal restitution coefficient at the contact.
        ///</summary>
        public double Restitution;

        ///<summary>
        /// Holds the position of the contact in world coordinates.
        ///</summary>
        public Vector3d ContactPoint;

        ///<summary>
        /// Holds the direction of the contact in world coordinates.
        ///</summary>
        public Vector3d ContactNormal;

        ///<summary>
        /// Holds the depth of penetration at the contact point. If both
        /// bodies are specified then the contact point should be midway
        /// between the inter-penetrating points.
        ///</summary>
        public double Penetration;

        ///<summary>
        /// A transform matrix that converts co-ordinates in the contact's
        /// frame of reference to world co-ordinates. The columns of this
        /// matrix form an orthonormal set of vectors.
        ///</summary>
        public Matrix3 ContactToWorld;

        ///<summary>
        /// Holds the closing velocity at the point of contact. This is set
        /// when the calculateInternals function is run.
        ///</summary>
        public Vector3d ContactVelocity;

        ///<summary>
        /// Holds the required change in velocity for this contact to be
        /// resolved.
        ///</summary>
        public double DesiredDeltaVelocity;

        ///<summary>
        /// Holds the world space position of the contact point relative to
        /// centre of each body. This is set when the calculateInternals
        /// function is run.
        ///</summary>
        public Vector3d[] RelativeContactPosition = new Vector3d[2];

        ///<summary>
        /// Sets the data that doesn't normally depend on the position
        /// of the contact (i.e. the bodies, and their material properties).
        ///</summary>
        public void SetBodyData(RigidBody one, RigidBody two, double friction, double restitution)
        {
            Body[0] = one;
            Body[1] = two;
            Friction = friction;
            Restitution = restitution;
        }

        /// <summary>
        /// 
        /// </summary>
        public void Clear()
        {
            Body[0] = null;
            Body[1] = null;
        }

        ///<summary>
        /// Calculates internal data from state data. This is called before
        /// the resolution algorithm tries to do any resolution. It should
        /// never need to be called manually.
        ///</summary>
        public void CalculateInternals(double dt)
        {

            // Check if the first object is NULL, and swap if it is.
            if (Body[0] == null) SwapBodies();

            if (Body[0] == null)
                throw new NullReferenceException("Body 0 null");

            // Calculate an set of axis at the contact point.
            CalculateContactBasis();

            // Store the relative position of the contact relative to each body
            RelativeContactPosition[0] = ContactPoint - Body[0].Position;
            if (Body[1] != null)
                RelativeContactPosition[1] = ContactPoint - Body[1].Position;

            // Find the relative velocity of the bodies at the contact point.
            ContactVelocity = CalculateLocalVelocity(0, dt);
            if (Body[1] != null)
                ContactVelocity -= CalculateLocalVelocity(1, dt);

            // Calculate the desired change in velocity for resolution
            CalculateDesiredDeltaVelocity(dt);
        }

        ///<summary>
        /// Reverses the contact. This involves swapping the two rigid bodies
        /// and reversing the contact normal. The internal values should then
        /// be recalculated using calculateInternals (this is not done
        /// automatically).
        ///</summary>
        public void SwapBodies()
        {
            ContactNormal *= -1;

            RigidBody temp = Body[0];
            Body[0] = Body[1];
            Body[1] = temp;
        }

        ///<summary>
        /// Updates the awake state of rigid bodies that are taking
        /// place in the given contact. A body will be made awake if it
        /// is in contact with a body that is awake.
        ///</summary>
        public void MatchAwakeState()
        {
            // Collisions with the world never cause a body to wake up.
            if (Body[1] == null) return;

            bool body0awake = Body[0].GetAwake();
            bool body1awake = Body[1].GetAwake();

            // Wake up only the sleeping one
            if (body0awake ^ body1awake)
            {
                if (body0awake)
                    Body[1].SetAwake();
                else
                    Body[0].SetAwake();
            }
        }

        ///<summary>
        /// Calculates and sets the internal value for the desired delta
        /// velocity.
        ///</summary>
        public void CalculateDesiredDeltaVelocity(double dt)
        {
            double velocityLimit = 0.25;

            // Calculate the acceleration induced velocity accumulated this frame
            double velocityFromAcc = 0;

            if (Body[0].GetAwake())
                velocityFromAcc += Vector3d.Dot(Body[0].LastFrameAcceleration * dt, ContactNormal);

            if (Body[1] != null && Body[1].GetAwake())
                velocityFromAcc -= Vector3d.Dot(Body[1].LastFrameAcceleration * dt, ContactNormal);

            // If the velocity is very slow, limit the restitution
            double thisRestitution = Restitution;
            if (Math.Abs(ContactVelocity.x) < velocityLimit)
            {
                thisRestitution = 0;
            }

            // Combine the bounce velocity with the removed
            // acceleration velocity.
            DesiredDeltaVelocity = -ContactVelocity.x - thisRestitution * (ContactVelocity.x - velocityFromAcc);
        }

        ///<summary>
        /// Calculates and returns the velocity of the contact
        /// point on the given body.
        ///</summary>
        public Vector3d CalculateLocalVelocity(int bodyIndex, double dt)
        {
            RigidBody thisBody = Body[bodyIndex];

            // Work out the velocity of the contact point.
            Vector3d velocity = Vector3d.Cross(thisBody.Rotation, RelativeContactPosition[bodyIndex]);
            velocity += thisBody.Velocity;

            // Turn the velocity into contact-coordinates.
            Vector3d contactVelocity = ContactToWorld.TransformTranspose(velocity);

            // Calculate the ammount of velocity that is due to forces without
            // reactions.
            Vector3d accVelocity = thisBody.LastFrameAcceleration * dt;

            // Calculate the velocity in contact-coordinates.
            accVelocity = ContactToWorld.TransformTranspose(accVelocity);

            // We ignore any component of acceleration in the contact normal
            // direction, we are only interested in planar acceleration
            accVelocity.x = 0;

            // Add the planar velocities - if there's enough friction they will
            // be removed during velocity resolution
            contactVelocity += accVelocity;

            // And return it
            return contactVelocity;
        }

        ///<summary>
        /// Calculates an orthonormal basis for the contact point, based on
        /// the primary friction direction (for anisotropic friction) or
        /// a random orientation (for isotropic friction).
        ///</summary>
        public void CalculateContactBasis()
        {
            Vector3d contactTangent0 = new Vector3d();
            Vector3d contactTangent1 = new Vector3d();

            // Check whether the Z-axis is nearer to the X or Y axis
            if (Math.Abs(ContactNormal.x) > Math.Abs(ContactNormal.y))
            {
                // Scaling factor to ensure the results are normalised
                double s = 1.0 / Math.Sqrt(ContactNormal.z * ContactNormal.z + ContactNormal.x * ContactNormal.x);

                // The new X-axis is at right angles to the world Y-axis
                contactTangent0.x = ContactNormal.z * s;
                contactTangent0.y = 0;
                contactTangent0.z = -ContactNormal.x * s;

                // The new Y-axis is at right angles to the new X- and Z- axes
                contactTangent1.x = ContactNormal.y * contactTangent0.x;
                contactTangent1.y = ContactNormal.z * contactTangent0.x - ContactNormal.x * contactTangent0.z;
                contactTangent1.z = -ContactNormal.y * contactTangent0.x;
            }
            else
            {
                // Scaling factor to ensure the results are normalised
                double s = 1.0 / Math.Sqrt(ContactNormal.z * ContactNormal.z + ContactNormal.y * ContactNormal.y);

                // The new X-axis is at right angles to the world X-axis
                contactTangent0.x = 0;
                contactTangent0.y = -ContactNormal.z * s;
                contactTangent0.z = ContactNormal.y * s;

                // The new Y-axis is at right angles to the new X- and Z- axes
                contactTangent1.x = ContactNormal.y * contactTangent0.z - ContactNormal.z * contactTangent0.y;
                contactTangent1.y = -ContactNormal.x * contactTangent0.z;
                contactTangent1.z = ContactNormal.x * contactTangent0.y;
            }

            // Make a matrix from the three vectors.
            ContactToWorld.SetComponents( ContactNormal, contactTangent0, contactTangent1);
        }

        ///<summary>
        /// Performs an inertia-weighted impulse based resolution of this
        /// contact alone.
        ///</summary>
        public void ApplyVelocityChange(Vector3d[] velocityChange, Vector3d[] rotationChange)
        {
            // Get hold of the inverse mass and inverse inertia tensor, both in
            // world coordinates.
            Matrix3 inverseInertiaTensor0, inverseInertiaTensor1 = Matrix3.Identity;

            inverseInertiaTensor0 = Body[0].InverseInertiaTensorWorld;

            if (Body[1] != null)
                inverseInertiaTensor1 = Body[1].InverseInertiaTensorWorld;

            // We will calculate the impulse for each contact axis
            Vector3d impulseContact;

            if (Friction == 0.0)
            {
                // Use the short format for frictionless contacts
                impulseContact = CalculateFrictionlessImpulse(inverseInertiaTensor0, inverseInertiaTensor1);
            }
            else
            {
                // Otherwise we may have impulses that aren't in the direction of the
                // contact, so we need the more complex version.
                impulseContact = CalculateFrictionImpulse(inverseInertiaTensor0, inverseInertiaTensor1);
            }

            // Convert impulse to world coordinates
            Vector3d impulse = ContactToWorld.Transform(impulseContact);

            // Split in the impulse into linear and rotational components
            Vector3d impulsiveTorque = Vector3d.Cross(RelativeContactPosition[0], impulse);
            rotationChange[0] = inverseInertiaTensor0.Transform(impulsiveTorque);
            velocityChange[0] = impulse * Body[0].InverseMass;

            // Apply the changes
            Body[0].Velocity += velocityChange[0];
            Body[0].Rotation += rotationChange[0];

            if (Body[1] != null)
            {
                // Work out body one's linear and angular changes
                impulsiveTorque = Vector3d.Cross(impulse, RelativeContactPosition[1]);
                rotationChange[1] = inverseInertiaTensor1.Transform(impulsiveTorque);
                velocityChange[1] = impulse * -Body[1].InverseMass;

                // And apply them.
                Body[1].Velocity += velocityChange[1];
                Body[1].Rotation += rotationChange[1];
            }
        }

        ///<summary>
        /// Performs an inertia weighted penetration resolution of this
        /// contact alone.
        ///</summary>
        public void ApplyPositionChange(Vector3d[] linearChange, Vector3d[] angularChange, double penetration)
        {
            double angularLimit = 0.2;
            double[] angularMove = new double[2];
            double[] linearMove = new double[2];

            double totalInertia = 0;
            double[] linearInertia = new double[2];
            double[] angularInertia = new double[2];

            // We need to work out the inertia of each object in the direction
            // of the contact normal, due to angular inertia only.
            for (int i = 0; i < 2; i++)
            {
                if (Body[i] == null) continue;

                Matrix3 inverseInertiaTensor = Body[i].InverseInertiaTensorWorld;

                // Use the same procedure as for calculating frictionless
                // velocity change to work out the angular inertia.
                Vector3d angularInertiaWorld = Vector3d.Cross(RelativeContactPosition[i], ContactNormal);
                angularInertiaWorld = inverseInertiaTensor.Transform(angularInertiaWorld);
                angularInertiaWorld = Vector3d.Cross(angularInertiaWorld, RelativeContactPosition[i]);
                angularInertia[i] = Vector3d.Dot(angularInertiaWorld, ContactNormal);

                // The linear component is simply the inverse mass
                linearInertia[i] = Body[i].InverseMass;

                // Keep track of the total inertia from all components
                totalInertia += linearInertia[i] + angularInertia[i];

                // We break the loop here so that the totalInertia value is
                // completely calculated (by both iterations) before
                // continuing.
            }

            // Loop through again calculating and applying the changes
            for (int i = 0; i < 2; i++)
            {
                if (Body[i] == null) continue;

                // The linear and angular movements required are in proportion to
                // the two inverse inertias.
                double sign = (i == 0) ? 1 : -1;
                angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);
                linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);

                // To avoid angular projections that are too great (when mass is large
                // but inertia tensor is small) limit the angular move.
                Vector3d projection = RelativeContactPosition[i];
                projection += ContactNormal * Vector3d.Dot(-RelativeContactPosition[i], ContactNormal);

                // Use the small angle approximation for the sine of the angle (i.e.
                // the magnitude would be sine(angularLimit) * projection.magnitude
                // but we approximate sine(angularLimit) to angularLimit).
                double maxMagnitude = angularLimit * projection.Magnitude;

                if (angularMove[i] < -maxMagnitude)
                {
                    double totalMove = angularMove[i] + linearMove[i];
                    angularMove[i] = -maxMagnitude;
                    linearMove[i] = totalMove - angularMove[i];
                }
                else if (angularMove[i] > maxMagnitude)
                {
                    double totalMove = angularMove[i] + linearMove[i];
                    angularMove[i] = maxMagnitude;
                    linearMove[i] = totalMove - angularMove[i];
                }

                // We have the linear amount of movement required by turning
                // the rigid body (in angularMove[i]). We now need to
                // calculate the desired rotation to achieve that.
                if (angularMove[i] == 0)
                {
                    // Easy case - no angular movement means no rotation.
                    angularChange[i] = Vector3d.Zero;
                }
                else
                {
                    // Work out the direction we'd like to rotate in.
                    Vector3d targetAngularDirection = Vector3d.Cross(RelativeContactPosition[i], ContactNormal);

                    Matrix3 inverseInertiaTensor = Body[i].InverseInertiaTensorWorld;

                    // Work out the direction we'd need to rotate to achieve that
                    angularChange[i] = inverseInertiaTensor.Transform(targetAngularDirection) * (angularMove[i] / angularInertia[i]);
                }

                // Velocity change is easier - it is just the linear movement
                // along the contact normal.
                linearChange[i] = ContactNormal * linearMove[i];

                // Now we can start to apply the values we've calculated.
                // Apply the linear movement
                Vector3d pos = Body[i].Position;
                pos += ContactNormal * linearMove[i];
                Body[i].Position = pos;

                // And the change in orientation
                Quaternion q = Body[i].Orientation;
                q.AddScaledVector(angularChange[i], 1.0);
                q.Normalise();
                Body[i].Orientation = q;

                // We need to calculate the derived data for any body that is
                // asleep, so that the changes are reflected in the object's
                // data. Otherwise the resolution will not change the position
                // of the object, and the next collision detection round will
                // have the same penetration.
                if (!Body[i].GetAwake()) Body[i].CalculateDerivedData();
            }
        }

        ///<summary>
        /// Calculates the impulse needed to resolve this contact,
        /// given that the contact has no friction. A pair of inertia
        /// tensors - one for each contact object - is specified to
        /// save calculation time: the calling function has access to
        /// these anyway.
        ///</summary>
        public Vector3d CalculateFrictionlessImpulse(Matrix3 inverseInertiaTensor0, Matrix3 inverseInertiaTensor1)
        {
            Vector3d impulseContact;

            // Build a vector that shows the change in velocity in
            // world space for a unit impulse in the direction of the contact
            // normal.
            Vector3d deltaVelWorld = Vector3d.Cross(RelativeContactPosition[0], ContactNormal);
            deltaVelWorld = inverseInertiaTensor0.Transform(deltaVelWorld);
            deltaVelWorld = Vector3d.Cross(deltaVelWorld, RelativeContactPosition[0]);

            // Work out the change in velocity in contact coordiantes.
            double deltaVelocity = Vector3d.Dot(deltaVelWorld, ContactNormal);

            // Add the linear component of velocity change
            deltaVelocity += Body[0].InverseMass;

            // Check if we need to the second body's data
            if (Body[1] != null)
            {
                // Go through the same transformation sequence again
                deltaVelWorld = Vector3d.Cross(RelativeContactPosition[1], ContactNormal);
                deltaVelWorld = inverseInertiaTensor1.Transform(deltaVelWorld);
                deltaVelWorld = Vector3d.Cross(deltaVelWorld, RelativeContactPosition[1]);

                // Add the change in velocity due to rotation
                deltaVelocity += Vector3d.Dot(deltaVelWorld, ContactNormal);

                // Add the change in velocity due to linear motion
                deltaVelocity += Body[1].InverseMass;
            }

            // Calculate the required size of the impulse
            impulseContact.x = DesiredDeltaVelocity / deltaVelocity;
            impulseContact.y = 0;
            impulseContact.z = 0;
            return impulseContact;
        }

        ///<summary>
        /// Calculates the impulse needed to resolve this contact,
        /// given that the contact has a non-zero coefficient of
        /// friction. A pair of inertia tensors - one for each contact
        /// object - is specified to save calculation time: the calling
        /// function has access to these anyway.
        ///</summary>
        public Vector3d CalculateFrictionImpulse(Matrix3 inverseInertiaTensor0, Matrix3 inverseInertiaTensor1)
        {
            Vector3d impulseContact;
            double inverseMass = Body[0].InverseMass;

            // The equivalent of a cross product in matrices is multiplication
            // by a skew symmetric matrix - we build the matrix for converting
            // between linear and angular quantities.
            Matrix3 impulseToTorque = Matrix3.Identity;
            impulseToTorque.SetSkewSymmetric(RelativeContactPosition[0]);

            // Build the matrix to convert contact impulse to change in velocity
            // in world coordinates.
            Matrix3 deltaVelWorld = impulseToTorque;
            deltaVelWorld *= inverseInertiaTensor0;
            deltaVelWorld *= impulseToTorque;
            deltaVelWorld *= -1;

            // Check if we need to add body two's data
            if (Body[1] != null)
            {
                // Set the cross product matrix
                impulseToTorque.SetSkewSymmetric(RelativeContactPosition[1]);

                // Calculate the velocity change matrix
                Matrix3 deltaVelWorld2 = impulseToTorque;
                deltaVelWorld2 *= inverseInertiaTensor1;
                deltaVelWorld2 *= impulseToTorque;
                deltaVelWorld2 *= -1;

                // Add to the total delta velocity.
                deltaVelWorld += deltaVelWorld2;

                // Add to the inverse mass
                inverseMass += Body[1].InverseMass;
            }

            // Do a change of basis to convert into contact coordinates.
            Matrix3 deltaVelocity = ContactToWorld.Transpose();
            deltaVelocity *= deltaVelWorld;
            deltaVelocity *= ContactToWorld;

            // Add in the linear velocity change
            deltaVelocity[0] += inverseMass;
            deltaVelocity[4] += inverseMass;
            deltaVelocity[8] += inverseMass;

            // Invert to get the impulse needed per unit velocity
            Matrix3 impulseMatrix = deltaVelocity.Inverse();

            // Find the target velocities to kill
            Vector3d velKill = new Vector3d(DesiredDeltaVelocity, -ContactVelocity.y, -ContactVelocity.z);

            // Find the impulse to kill target velocities
            impulseContact = impulseMatrix.Transform(velKill);

            // Check for exceeding friction
            double planarImpulse = Math.Sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);

            if (planarImpulse > impulseContact.x * Friction)
            {
                // We need to use dynamic friction
                impulseContact.y /= planarImpulse;
                impulseContact.z /= planarImpulse;

                impulseContact.x = deltaVelocity[0] +
                    deltaVelocity[1] * Friction * impulseContact.y +
                    deltaVelocity[2] * Friction * impulseContact.z;

                impulseContact.x = DesiredDeltaVelocity / impulseContact.x;
                impulseContact.y *= Friction * impulseContact.x;
                impulseContact.z *= Friction * impulseContact.x;
            }
            return impulseContact;
        }
    }


}