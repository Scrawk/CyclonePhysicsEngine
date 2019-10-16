using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Cyclone.Rigid
{

    ///<summary>
    /// A rigid body is the basic simulation object in the physics
    /// core.
    ///
    /// It has position and orientation data, along with first
    /// derivatives. It can be integrated forward through time, and
    /// have forces, torques and impulses (linear or angular) applied
    /// to it. The rigid body manages its state and allows access
    /// through a set of methods.
    ///</summary>
    public class RigidBody
    {

        /// <summary>
        /// Holds the value for energy under which a body will be put to
        /// sleep. By default it is 0.1, which is fine for simulation when 
        /// gravity is about 20 units per second squared, masses are about 
        /// one, and other forces are around that of gravity.It may need 
        /// tweaking if your simulation is drastically different to this.
        /// </summary>
        public double SleepEpsilon = 0.1;

        ///<summary>
        /// Holds the inverse of the mass of the rigid body. It
        /// is more useful to hold the inverse mass because
        /// integration is simpler, and because in double time
        /// simulation it is more useful to have bodies with
        /// infinite mass (immovable) than zero mass
        /// (completely unstable in numerical simulation).
        ///</summary>
        public double InverseMass;

        ///<summary>
        /// Returns true if the mass of the body is not-infinite.
        ///</summary>
        public bool HasFiniteMass => InverseMass != 0;

        ///<summary>
        /// Returns true if the mass of the body is infinite.
        ///</summary>
        public bool HasInfiniteMass => InverseMass == 0;

        ///<summary>
        /// Holds the inverse of the body's inertia tensor. The
        /// inertia tensor provided must not be degenerate
        /// (that would mean the body had zero inertia for
        /// spinning along one axis). As long as the tensor is
        /// finite, it will be invertible. The inverse tensor
        /// is used for similar reasons to the use of inverse
        /// mass.
        ///
        /// The inertia tensor, unlike the other variables that
        /// define a rigid body, is given in body space.
        ///</summary>
        public Matrix3 InverseInertiaTensor;

        ///<summary>
        /// Holds the amount of damping applied to linear
        /// motion.  Damping is required to remove energy added
        /// through numerical instability in the integrator.
        ///</summary>
        public double LinearDamping = 0.99;

        ///<summary>
        /// Holds the amount of damping applied to angular
        /// motion.  Damping is required to remove energy added
        /// through numerical instability in the integrator.
        ///</summary>
        public double AngularDamping = 0.99;

        ///<summary>
        /// Holds the linear position of the rigid body in
        /// world space.
        ///</summary>
        public Vector3d Position;

        ///<summary>
        /// Holds the angular orientation of the rigid body in
        /// world space.
        ///</summary>
        public Quaternion Orientation;

        ///<summary>
        /// Holds the linear velocity of the rigid body in
        /// world space.
        ///</summary>
        public Vector3d Velocity;

        ///<summary>
        /// Holds the angular velocity, or rotation, or the
        /// rigid body in world space.
        ///</summary>
        public Vector3d Rotation;

        ///<summary>
        /// These data members hold information that is derived from
        /// the other data in the class.
        ///</summary>

        ///<summary>
        /// Holds the inverse inertia tensor of the body in world
        /// space. The inverse inertia tensor member is specified in
        /// the body's local space.
        ///</summary>
        public Matrix3 InverseInertiaTensorWorld;

        ///<summary>
        /// Holds the amount of motion of the body. This is a recency
        /// weighted mean that can be used to put a body to sleap.
        ///</summary>
        private double m_motion;

        ///<summary>
        /// A body can be put to sleep to avoid it being updated
        /// by the integration functions or affected by collisions
        /// with the world.
        ///</summary>
        private bool m_isAwake;

        ///<summary>
        /// Some bodies may never be allowed to fall asleep.
        /// User controlled bodies, for example, should be
        /// always awake.
        ///</summary>
        private bool m_canSleep;

        ///<summary>
        /// Holds a transform matrix for converting body space into
        /// world space and vice versa. This can be achieved by calling
        /// the getPointInSpace functions.
        ///</summary>
        public Matrix4 Transform;

        ///<summary>
        /// Force and Torque Accumulators
        ///
        /// These data members store the current force, torque and
        /// acceleration of the rigid body. Forces can be added to the
        /// rigid body in any order, and the class decomposes them into
        /// their constituents, accumulating them for the next
        /// simulation step. At the simulation step, the accelerations
        /// are calculated and stored to be applied to the rigid body.
        ///</summary>

        ///<summary>
        /// Holds the accumulated force to be applied at the next
        /// integration step.
        ///</summary>
        private Vector3d m_forceAccum;

        ///<summary>
        /// Holds the accumulated torque to be applied at the next
        /// integration step.
        ///</summary>
        private Vector3d m_torqueAccum;

        ///<summary>
        /// Holds the acceleration of the rigid body.  This value
        /// can be used to set acceleration due to gravity (its primary
        /// use), or any other constant acceleration.
        ///</summary>
        private Vector3d m_acceleration;

        ///<summary>
        /// Holds the linear acceleration of the rigid body, for the
        /// previous frame.
        ///</summary>
        public Vector3d LastFrameAcceleration;

        /// <summary>
        /// 
        /// </summary>
        public RigidBody()
        {
            Orientation = Quaternion.Identity;
            InverseInertiaTensor = Matrix3.Identity;
            Transform = Matrix4.Identity;
        }

        ///<summary>
        /// Calculates internal data from state data. This should be called
        /// after the body's state is altered directly (it is called
        /// automatically during integration). If you change the body's state
        /// and then intend to integrate before querying any data (such as
        /// the transform matrix), then you can ommit this step.
        ///</summary>
        public void CalculateDerivedData()
        {
            Orientation.Normalise();

            // Calculate the transform matrix for the body.
            CalculateTransformMatrix(ref Transform, Position, Orientation);

            // Calculate the inertiaTensor in world space.
            TransformInertiaTensor(ref InverseInertiaTensorWorld, InverseInertiaTensor, Transform);
        }

        ///<summary>
        /// Integrates the rigid body forward in time by the given amount.
        /// This function uses a Newton-Euler integration method, which is a
        /// linear approximation to the correct integral. For this reason it
        /// may be inaccurate in some cases.
        ///</summary>
        public void Integrate(double dt)
        {
            if (!m_isAwake) return;

            // Calculate linear acceleration from force inputs.
            LastFrameAcceleration = m_acceleration;
            LastFrameAcceleration += m_forceAccum * InverseMass;

            // Calculate angular acceleration from torque inputs.
            Vector3d angularAcceleration = InverseInertiaTensorWorld * m_torqueAccum;

            // Adjust velocities
            // Update linear velocity from both acceleration and impulse.
            Velocity += LastFrameAcceleration * dt;

            // Update angular velocity from both acceleration and impulse.
            Rotation += angularAcceleration * dt;

            // Impose drag.
            Velocity *= Math.Pow(LinearDamping, dt);
            Rotation *= Math.Pow(AngularDamping, dt);

            // Adjust positions
            // Update linear position.
            Position += Velocity * dt;

            // Update angular position.
            Orientation.AddScaledVector(Rotation, dt);

            // Normalise the orientation, and update the matrices with the new
            // position and orientation
            CalculateDerivedData();

            // Clear accumulators.
            ClearAccumulators();

            // Update the kinetic energy store, and possibly put the body to
            // sleep.
            if (m_canSleep)
            {
                double currentMotion = Vector3d.Dot(Velocity, Velocity) + Vector3d.Dot(Rotation, Rotation);

                double bias = Math.Pow(0.5, dt);
                m_motion = bias * m_motion + (1 - bias) * currentMotion;

                if (m_motion < SleepEpsilon)
                    SetAwake(false);
                else if (m_motion > 10 * SleepEpsilon)
                    m_motion = 10 * SleepEpsilon;
            }
        }

        ///<summary>
        /// Sets the mass of the rigid body.
        /// Small masses can produce unstable rigid bodies under
        /// simulation.
        ///</summary>
        public void SetMass(double mass)
        {
            if (mass <= 0)
                InverseMass = 0;
            else
                InverseMass = 1.0 / mass;
        }

        ///<summary>
        /// Gets the mass of the rigid body.
        ///</summary>
        public double GetMass()
        {
            if (InverseMass == 0)
                return double.PositiveInfinity;
            else
                return 1.0f / InverseMass;
        }

        ///<summary>
        /// Sets the intertia tensor for the rigid body.
        ///
        /// @param inertiaTensor The inertia tensor for the rigid
        /// body. This must be a full rank matrix and must be
        /// invertible.
        ///</summary>
        public void SetInertiaTensor(Matrix3 inertiaTensor)
        {
            InverseInertiaTensor = inertiaTensor.Inverse();
        }

        ///<summary>
        /// Gets a copy of the current inertia tensor of the rigid body.
        ///
        /// @return A new matrix containing the current intertia
        /// tensor. The inertia tensor is expressed in the rigid body's
        /// local space.
        ///</summary>
        public Matrix3 GetInertiaTensor()
        {
            return InverseInertiaTensor.Inverse();
        }

        ///<summary>
        /// Gets a copy of the current inertia tensor of the rigid body.
        ///
        /// @return A new matrix containing the current intertia
        /// tensor. The inertia tensor is expressed in world space.
        ///</summary>
        public Matrix3 GetInertiaTensorWorld()
        {
            return InverseInertiaTensorWorld.Inverse();
        }

        ///<summary>
        /// Converts the given point from world space into the body's
        /// local space.
        ///
        /// @param point The point to covert, given in world space.
        ///
        /// @return The converted point, in local space.
        ///</summary>
        public Vector3d GetPointInLocalSpace(Vector3d point)
        {
            return Transform.TransformInverse(point);
        }

        ///<summary>
        /// Converts the given point from world space into the body's
        /// local space.
        ///
        /// @param point The point to covert, given in local space.
        ///
        /// @return The converted point, in world space.
        ///</summary>
        public Vector3d GetPointInWorldSpace(Vector3d point)
        {
            return Transform.Transform(point);
        }

        ///<summary>
        /// Converts the given direction from world space into the
        /// body's local space.
        ///
        /// @note When a direction is converted between frames of
        /// reference, there is no translation required.
        ///
        /// @param direction The direction to covert, given in world
        /// space.
        ///
        /// @return The converted direction, in local space.
        ///</summary>
        public Vector3d GetDirectionInLocalSpace(Vector3d direction)
        {
            return Transform.TransformInverseDirection(direction);
        }

        ///<summary>
        /// Converts the given direction from world space into the
        /// body's local space.
        ///
        /// @note When a direction is converted between frames of
        /// reference, there is no translation required.
        ///
        /// @param direction The direction to covert, given in local
        /// space.
        ///
        /// @return The converted direction, in world space.
        ///</summary>
        public Vector3d GetDirectionInWorldSpace(Vector3d direction)
        {
            return Transform.TransformDirection(direction);
        }

        ///<summary>
        /// Returns true if the body is awake and responding to
        /// integration.
        ///
        /// @return The awake state of the body.
        ///</summary>
        public bool GetAwake()
        {
            return m_isAwake;
        }

        ///<summary>
        /// Sets the awake state of the body. If the body is set to be
        /// not awake, then its velocities are also cancelled, since
        /// a moving body that is not awake can cause problems in the
        /// simulation.
        ///
        /// @param awake The new awake state of the body.
        ///</summary>
        public void SetAwake(bool awake = true)
        {
            if (awake)
            {
                m_isAwake = true;

                //Add a bit of motion to avoid it falling asleep immediately.
                m_motion = SleepEpsilon * 2.0;
            }
            else
            {
                m_isAwake = false;
                Velocity = Vector3d.Zero;
                Rotation = Vector3d.Zero;
            }
        }

        ///<summary>
        /// Returns true if the body is allowed to go to sleep at
        /// any time.
        ///</summary>
        public bool GetCanSleep()
        {
            return m_canSleep;
        }

        ///<summary>
        /// Sets whether the body is ever allowed to go to sleep. Bodies
        /// under the player's control, or for which the set of
        /// transient forces applied each frame are not predictable,
        /// should be kept awake.
        ///
        /// @param canSleep Whether the body can now be put to sleep.
        ///</summary>
        public void SetCanSleep(bool canSleep = true)
        {
            m_canSleep = canSleep;
            if (!m_canSleep && !m_isAwake) SetAwake();
        }

        ///<summary>
        /// @name Retrieval Functions for Dynamic Quantities
        ///
        /// These functions provide access to the acceleration
        /// properties of the body. The acceleration is generated by
        /// the simulation from the forces and torques applied to the
        /// rigid body. Acceleration cannot be directly influenced, it
        /// is set during integration, and represent the acceleration
        /// experienced by the body of the previous simulation step.
        ///</summary>


        ///<summary>
        /// Clears the forces and torques in the accumulators. This will
        /// be called automatically after each intergration step.
        ///</summary>
        public void ClearAccumulators()
        {
            m_forceAccum = Vector3d.Zero;
            m_torqueAccum = Vector3d.Zero;
        }

        ///<summary>
        /// Adds the given force to centre of mass of the rigid body.
        /// The force is expressed in world-coordinates.
        ///
        /// @param force The force to apply.
        ///</summary>
        public void AddForce(Vector3d force)
        {
            m_forceAccum += force;
            m_isAwake = true;
        }

        ///<summary>
        /// Adds the given force to the given point on the rigid body.
        /// Both the force and the
        /// application point are given in world space. Because the
        /// force is not applied at the centre of mass, it may be split
        /// into both a force and torque.
        ///
        /// @param force The force to apply.
        ///
        /// @param point The location at which to apply the force, in
        /// world-coordinates.
        ///</summary>
        public void AddForceAtPoint(Vector3d force, Vector3d point)
        {
            // Convert to coordinates relative to center of mass.
            Vector3d pt = point - Position;

            m_forceAccum += force;
            m_torqueAccum += Vector3d.Cross(pt, force);
            m_isAwake = true;
        }

        ///<summary>
        /// Adds the given force to the given point on the rigid body.
        /// The direction of the force is given in world coordinates,
        /// but the application point is given in body space. This is
        /// useful for spring forces, or other forces fixed to the
        /// body.
        ///
        /// @param force The force to apply.
        ///
        /// @param point The location at which to apply the force, in
        /// body-coordinates.
        ///<summary>
        public void AddForceAtBodyPoint(Vector3d force, Vector3d point)
        {
            // Convert to coordinates relative to center of mass.
            Vector3d pt = GetPointInWorldSpace(point);
            AddForceAtPoint(force, pt);
        }

        ///<summary>
        /// Adds the given torque to the rigid body.
        /// The force is expressed in world-coordinates.
        ///
        /// @param torque The torque to apply.
        ///<summary>
        public void AddTorque(Vector3d torque)
        {
            m_torqueAccum += torque;
            m_isAwake = true;
        }

        ///<summary>
        /// Internal function to do an intertia tensor transform by a quaternion.
        /// Note that the implementation of this function was created by an
        /// automated code-generator and optimizer.
        ///</summary>
        private static void TransformInertiaTensor(ref Matrix3 iitWorld, Matrix3 iitBody, Matrix4 rotmat)
        {
            double t4 = rotmat.data0 * iitBody.data0 +
                rotmat.data1 * iitBody.data3 +
                rotmat.data2 * iitBody.data6;
            double t9 = rotmat.data0 * iitBody.data1 +
                rotmat.data1 * iitBody.data4 +
                rotmat.data2 * iitBody.data7;
            double t14 = rotmat.data0 * iitBody.data2 +
                rotmat.data1 * iitBody.data5 +
                rotmat.data2 * iitBody.data8;
            double t28 = rotmat.data4 * iitBody.data0 +
                rotmat.data5 * iitBody.data3 +
                rotmat.data6 * iitBody.data6;
            double t33 = rotmat.data4 * iitBody.data1 +
                rotmat.data5 * iitBody.data4 +
                rotmat.data6 * iitBody.data7;
            double t38 = rotmat.data4 * iitBody.data2 +
                rotmat.data5 * iitBody.data5 +
                rotmat.data6 * iitBody.data8;
            double t52 = rotmat.data8 * iitBody.data0 +
                rotmat.data9 * iitBody.data3 +
                rotmat.data10 * iitBody.data6;
            double t57 = rotmat.data8 * iitBody.data1 +
                rotmat.data9 * iitBody.data4 +
                rotmat.data10 * iitBody.data7;
            double t62 = rotmat.data8 * iitBody.data2 +
                rotmat.data9 * iitBody.data5 +
                rotmat.data10 * iitBody.data8;

            iitWorld.data0 = t4 * rotmat.data0 +
                t9 * rotmat.data1 +
                t14 * rotmat.data2;
            iitWorld.data1 = t4 * rotmat.data4 +
                t9 * rotmat.data5 +
                t14 * rotmat.data6;
            iitWorld.data2 = t4 * rotmat.data8 +
                t9 * rotmat.data9 +
                t14 * rotmat.data10;
            iitWorld.data3 = t28 * rotmat.data0 +
                t33 * rotmat.data1 +
                t38 * rotmat.data2;
            iitWorld.data4 = t28 * rotmat.data4 +
                t33 * rotmat.data5 +
                t38 * rotmat.data6;
            iitWorld.data5 = t28 * rotmat.data8 +
                t33 * rotmat.data9 +
                t38 * rotmat.data10;
            iitWorld.data6 = t52 * rotmat.data0 +
                t57 * rotmat.data1 +
                t62 * rotmat.data2;
            iitWorld.data7 = t52 * rotmat.data4 +
                t57 * rotmat.data5 +
                t62 * rotmat.data6;
            iitWorld.data8 = t52 * rotmat.data8 +
                t57 * rotmat.data9 +
                t62 * rotmat.data10;
        }

        ///<summary>
        /// Inline function that creates a transform matrix from a
        /// position and orientation.
        ///</summary>
        private static void CalculateTransformMatrix(ref Matrix4 transformMatrix, Vector3d position, Quaternion orientation)
        {
            transformMatrix[0] = 1 - 2 * orientation.j * orientation.j -
                2 * orientation.k * orientation.k;
            transformMatrix[1] = 2 * orientation.i * orientation.j -
                2 * orientation.r * orientation.k;
            transformMatrix[2] = 2 * orientation.i * orientation.k +
                2 * orientation.r * orientation.j;
            transformMatrix[3] = position.x;

            transformMatrix[4] = 2 * orientation.i * orientation.j +
                2 * orientation.r * orientation.k;
            transformMatrix[5] = 1 - 2 * orientation.i * orientation.i -
                2 * orientation.k * orientation.k;
            transformMatrix[6] = 2 * orientation.j * orientation.k -
                2 * orientation.r * orientation.i;
            transformMatrix[7] = position.y;

            transformMatrix[8] = 2 * orientation.i * orientation.k -
                2 * orientation.r * orientation.j;
            transformMatrix[9] = 2 * orientation.j * orientation.k +
                2 * orientation.r * orientation.i;
            transformMatrix[10] = 1 - 2 * orientation.i * orientation.i -
                2 * orientation.j * orientation.j;
            transformMatrix[11] = position.z;
        }
    }

}