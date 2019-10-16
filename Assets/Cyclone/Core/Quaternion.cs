using System;
using System.Collections.Generic;

namespace Cyclone.Core
{
    /// <summary>
    /// Holds a three degree of freedom orientation.
    ///
    /// Quaternions have
    /// several mathematical properties that make them useful for
    /// representing orientations, but require four items of data to
    /// hold the three degrees of freedom.These four items of data can
    /// be viewed as the coefficients of a complex number with three
    /// imaginary parts.The mathematics of the quaternion is then
    /// defined and is roughly correspondent to the math of 3D
    /// rotations.A quaternion is only a valid rotation if it is
    /// normalised: i.e.it has a length of 1.
    ///
    /// @note Angular velocity and acceleration can be correctly
    /// represented as vectors. Quaternions are only needed for
    /// orientation.
    /// </summary>
    public struct Quaternion
    {
        /// <summary>
        /// Holds the real component of the quaternion.
        /// </summary>
        public double r;

        /// <summary>
        /// Holds the first complex component of the quaternion.
        /// </summary>
        public double i;

        /// <summary>
        /// Holds the second complex component of the quaternion.
        /// </summary>
        public double j;

        /// <summary>
        /// Holds the third complex component of the quaternion.
        /// </summary>
        public double k;

        /// <summary>
        /// 
        /// </summary>
        public readonly static Quaternion Identity = new Quaternion(1, 0, 0, 0);

        /// <summary>
        /// The explicit constructor creates a quaternion with the given components.
        /// </summary>
        /// <param name="r">The real component of the rigid body's orientation quaternion.</param>
        /// <param name="i">The first complex component of the rigid body's orientation quaternion</param>
        /// <param name="j">The second complex component of the rigid body's orientation quaternion</param>
        /// <param name="k">The third complex component of the rigid body's orientation quaternion</param>
        public Quaternion(double r, double i, double j, double k)
        {
            this.r = r;
            this.i = i;
            this.j = j;
            this.k = k;
        }

        /// <summary>
        /// Normalises the quaternion to unit length, making it a valid
        /// orientation quaternion.
        /// </summary>
        public void Normalise()
        {
            double d = r * r + i * i + j * j + k * k;

            // Check for zero length quaternion, and use the no-rotation
            // quaternion in that case.
            if (d < DMath.EPS)
            {
                i = j = k = 0;
                r = 1;
                return;
            }

            d = 1.0 / Math.Sqrt(d);
            r *= d;
            i *= d;
            j *= d;
            k *= d;
        }

        /// <summary>
        /// Multiplies the quaternion by the given quaternion.
        /// </summary>
        public static Quaternion operator *(Quaternion q1, Quaternion q2)
        {
            Quaternion q = new Quaternion();

            q.r = q1.r * q2.r - q1.i * q2.i -
                q1.j * q2.j - q1.k * q2.k;
            q.i = q1.r * q2.i + q1.i * q2.r +
                q1.j * q2.k - q1.k * q2.j;
            q.j = q1.r * q2.j + q1.j * q2.r +
                q1.k * q2.i - q1.i * q2.k;
            q.k = q1.r * q2.k + q1.k * q2.r +
                q1.i * q2.j - q1.j * q2.i;

            return q;
        }

        /// <summary>
        /// Adds the given vector to this, scaled by the given amount.
        /// This is used to update the orientation quaternion by a rotation
        /// </summary>
        /// <param name="vector">The vector to add.</param>
        /// <param name="scale">The amount of the vector to add.</param>
        public void AddScaledVector(Vector3d vector, double scale)
        {
            Quaternion q = new Quaternion(0,
                vector.x * scale,
                vector.y * scale,
                vector.z * scale);

            q = q * this;
            r += q.r * 0.5;
            i += q.i * 0.5;
            j += q.j * 0.5;
            k += q.k * 0.5;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="vector"></param>
        public void RotateByVector(Vector3d vector)
        {
            Quaternion q = new Quaternion(0, vector.x, vector.y, vector.z);

            q = this * q;
            r = q.r;
            i = q.i;
            j = q.j;
            k = q.k;
        }
    }


}