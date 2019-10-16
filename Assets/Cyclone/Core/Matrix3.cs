using System;
using System.Collections.Generic;

namespace Cyclone.Core
{

    /// <summary>
    /// Holds an inertia tensor, consisting of a 3x3 row-major matrix.
    /// This matrix is not padding to produce an aligned structure, since
    /// it is most commonly used with a mass(single double) and two
    /// damping coefficients to make the 12-element characteristics array
    /// of a rigid body.
    /// </summary>
    public struct Matrix3
    {
        public double data0, data1, data2;
        public double data3, data4, data5;
        public double data6, data7, data8;

        /// <summary>
        /// Creates an identity matrix.
        /// </summary>
        static readonly public Matrix3 Identity = new Matrix3(1, 0, 0,
                                                              0, 1, 0,
                                                              0, 0, 1);


        public Matrix3(double m0, double m1, double m2,
                       double m3, double m4, double m5,
                       double m6, double m7, double m8)
        {
            data0 = m0; data1 = m1; data2 = m2;
            data3 = m3; data4 = m4; data5 = m5;
            data6 = m6; data7 = m7; data8 = m8;
        }

        /// <summary>
        /// Creates a new matrix with the given three vectors making
        /// up its columns.
        /// </summary>
        public Matrix3(Vector3d compOne, Vector3d compTwo, Vector3d compThree)
        {
            data0 = compOne.x; data1 = compTwo.x; data2 = compThree.x;
            data3 = compOne.y; data4 = compTwo.y; data5 = compThree.y;
            data6 = compOne.z; data7 = compTwo.z; data8 = compThree.z;
        }

        /// <summary>
        /// Access the varible at index i
        /// </summary>
        unsafe public double this[int i]
        {
            get
            {
                if ((uint)i >= 9)
                    throw new IndexOutOfRangeException("Matrix3 index out of range.");

                fixed (Matrix3* array = &this) { return ((double*)array)[i]; }
            }
            set
            {
                if ((uint)i >= 9)
                    throw new IndexOutOfRangeException("Matrix3 index out of range.");

                fixed (double* array = &data0) { array[i] = value; }
            }
        }

        /// <summary>
        /// Transform the given vector by this matrix.
        /// </summary>
        public static Vector3d operator *(Matrix3 m, Vector3d vector)
        {
            return new Vector3d(
                vector.x * m.data0 + vector.y * m.data1 + vector.z * m.data2,
                vector.x * m.data3 + vector.y * m.data4 + vector.z * m.data5,
                vector.x * m.data6 + vector.y * m.data7 + vector.z * m.data8
            );
        }

        /// <summary>
        /// Returns a matrix which is this matrix multiplied by the given
        /// other matrix.
        /// </summary>
        public static Matrix3 operator *(Matrix3 m1, Matrix3 m2)
        {
            return new Matrix3(
                m1.data0 * m2.data0 + m1.data1 * m2.data3 + m1.data2 * m2.data6,
                m1.data0 * m2.data1 + m1.data1 * m2.data4 + m1.data2 * m2.data7,
                m1.data0 * m2.data2 + m1.data1 * m2.data5 + m1.data2 * m2.data8,

                m1.data3 * m2.data0 + m1.data4 * m2.data3 + m1.data5 * m2.data6,
                m1.data3 * m2.data1 + m1.data4 * m2.data4 + m1.data5 * m2.data7,
                m1.data3 * m2.data2 + m1.data4 * m2.data5 + m1.data5 * m2.data8,

                m1.data6 * m2.data0 + m1.data7 * m2.data3 + m1.data8 * m2.data6,
                m1.data6 * m2.data1 + m1.data7 * m2.data4 + m1.data8 * m2.data7,
                m1.data6 * m2.data2 + m1.data7 * m2.data5 + m1.data8 * m2.data8
                );
        }

        /// <summary>
        /// Multiplies this matrix in place by the given scalar.
        /// </summary>
        public static Matrix3 operator *(Matrix3 m1, double scalar)
        {
            return new Matrix3(
             m1.data0 * scalar, m1.data1 * scalar, m1.data2 * scalar,
             m1.data3 * scalar, m1.data4 * scalar, m1.data5 * scalar,
             m1.data6 * scalar, m1.data7 * scalar, m1.data8 * scalar);
        }

        /// <summary>
        /// Do a component-wise addition of this matrix and the given
        /// matrix.
        /// </summary>
        public static Matrix3 operator +(Matrix3 m1, Matrix3 m2)
        {
            return new Matrix3(
            m1.data0 + m2.data0, m1.data1 + m2.data1, m1.data2 + m2.data2,
            m1.data3 + m2.data3, m1.data4 + m2.data4, m1.data5 + m2.data5,
            m1.data6 + m2.data6, m1.data7 + m2.data7, m1.data8 + m2.data8);
        }

        /// <summary>
        /// Do a component-wise subtraction of this matrix and the given
        /// matrix.
        /// </summary>
        public static Matrix3 operator -(Matrix3 m1, Matrix3 m2)
        {
            return new Matrix3(
            m1.data0 - m2.data0, m1.data1 - m2.data1, m1.data2 - m2.data2,
            m1.data3 - m2.data3, m1.data4 - m2.data4, m1.data5 - m2.data5,
            m1.data6 - m2.data6, m1.data7 - m2.data7, m1.data8 - m2.data8);
        }

        /// <summary>
        /// A matrix as a string.
        /// </summary>
        public override string ToString()
        {
            return data0 + "," + data1 + "," + data2 + "\n" +
                   data3 + "," + data4 + "," + data5 + "\n" +
                   data6 + "," + data7 + "," + data8;
        }

        /// <summary>
        /// Sets the value of the matrix from inertia tensor values.
        /// </summary>
        public void SetInertiaTensorCoeffs(double ix, double iy, double iz, double ixy = 0, double ixz = 0, double iyz = 0)
        {
            data0 = ix;
            data1 = data3 = -ixy;
            data2 = data6 = -ixz;
            data4 = iy;
            data5 = data7 = -iyz;
            data8 = iz;
        }

        /// <summary>
        /// Sets the value of the matrix as an inertia tensor of
        /// a rectangular block aligned with the body's coordinate
        /// system with the given axis half-sizes and mass.
        /// </summary>
        public void SetBlockInertiaTensor(Vector3d halfSizes, double mass)
        {
            Vector3d squares = halfSizes * halfSizes;
            SetInertiaTensorCoeffs(0.3 * mass * (squares.y + squares.z), 0.3 * mass * (squares.x + squares.z), 0.3 * mass * (squares.x + squares.y));
        }

        /// <summary>
        /// Sets the matrix to be a skew symmetric matrix based on
        /// the given vector.The skew symmetric matrix is the equivalent
        /// of the vector product.So if a, b are vectors.a x b = A_s b
        /// where A_s is the skew symmetric form of a.
        /// </summary>
        public void SetSkewSymmetric(Vector3d vector)
        {
            data0 = data4 = data8 = 0;
            data1 = -vector.z;
            data2 = vector.y;
            data3 = vector.z;
            data5 = -vector.x;
            data6 = -vector.y;
            data7 = vector.x;
        }

        /// <summary>
        /// Sets the matrix values from the given three vector components.
        /// These are arranged as the three columns of the vector.
        /// </summary>
        public void SetComponents(Vector3d compOne, Vector3d compTwo, Vector3d compThree)
        {
            data0 = compOne.x; data1 = compTwo.x; data2 = compThree.x;
            data3 = compOne.y; data4 = compTwo.y; data5 = compThree.y;
            data6 = compOne.z; data7 = compTwo.z; data8 = compThree.z;
        }

        /// <summary>
        /// Transform the given vector by this matrix.
        /// </summary>
        public Vector3d Transform(Vector3d vector)
        {
            return this * vector;
        }

        /// <summary>
        /// Transform the given vector by the transpose of this matrix.
        /// </summary>
        public Vector3d TransformTranspose(Vector3d vector)
        {
            return new Vector3d(
                vector.x * data0 + vector.y * data3 + vector.z * data6,
                vector.x * data1 + vector.y * data4 + vector.z * data7,
                vector.x * data2 + vector.y * data5 + vector.z * data8
            );
        }

        /// <summary>
        /// Gets a vector representing one row in the matrix.
        /// </summary>
        /// <param name="i">The row to return.</param>
        public Vector3d GetRowVector(int i)
        {
            return new Vector3d(this[i * 3], this[i * 3 + 1], this[i * 3 + 2]);
        }

        /// <summary>
        /// Gets a vector representing one axis (i.e. one column) in the matrix.
        /// </summary>
        /// <param name="i">The row to return.</param>
        public Vector3d GetAxisVector(int i)
        {
            return new Vector3d(this[i], this[i + 3], this[i + 6]);
        }

        /// <summary>
        /// Sets the matrix to be the inverse of the given matrix.
        /// </summary>
        public void SetInverse(Matrix3 m)
        {
            double t4 = m.data0 * m.data4;
            double t6 = m.data0 * m.data5;
            double t8 = m.data1 * m.data3;
            double t10 = m.data2 * m.data3;
            double t12 = m.data1 * m.data6;
            double t14 = m.data2 * m.data6;

            // Calculate the determinant
            double t16 = (t4 * m.data8 - t6 * m.data7 - t8 * m.data8 +
                        t10 * m.data7 + t12 * m.data5 - t14 * m.data4);

            // Make sure the determinant is non-zero.
            if (t16 == 0.0) return;
            double t17 = 1 / t16;

            data0 = (m.data4 * m.data8 - m.data5 * m.data7) * t17;
            data1 = -(m.data1 * m.data8 - m.data2 * m.data7) * t17;
            data2 = (m.data1 * m.data5 - m.data2 * m.data4) * t17;
            data3 = -(m.data3 * m.data8 - m.data5 * m.data6) * t17;
            data4 = (m.data0 * m.data8 - t14) * t17;
            data5 = -(t6 - t10) * t17;
            data6 = (m.data3 * m.data7 - m.data4 * m.data6) * t17;
            data7 = -(m.data0 * m.data7 - t12) * t17;
            data8 = (t4 - t8) * t17;
        }

        /// <summary>
        /// Returns a new matrix containing the inverse of this matrix.
        /// </summary>
        public Matrix3 Inverse()
        {
            Matrix3 result = Identity;
            result.SetInverse(this);
            return result;
        }

        /// <summary>
        /// Inverts the matrix.
        /// </summary>
        public void Invert()
        {
            SetInverse(this);
        }

        /// <summary>
        /// Sets the matrix to be the transpose of the given matrix.
        /// </summary>
        public void SetTranspose(Matrix3 m)
        {
            data0 = m.data0;
            data1 = m.data3;
            data2 = m.data6;
            data3 = m.data1;
            data4 = m.data4;
            data5 = m.data7;
            data6 = m.data2;
            data7 = m.data5;
            data8 = m.data8;
        }

        /// <summary>
        /// Returns a new matrix containing the transpose of this matrix. 
        /// </summary>
        public Matrix3 Transpose()
        {
            Matrix3 result = Identity;
            result.SetTranspose(this);
            return result;
        }

        /// <summary>
        /// Sets this matrix to be the rotation matrix corresponding to
        /// the given quaternion.
        /// </summary>
        public void SetOrientation(Quaternion q)
        {
            data0 = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            data1 = 2 * q.i * q.j + 2 * q.k * q.r;
            data2 = 2 * q.i * q.k - 2 * q.j * q.r;
            data3 = 2 * q.i * q.j - 2 * q.k * q.r;
            data4 = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
            data5 = 2 * q.j * q.k + 2 * q.i * q.r;
            data6 = 2 * q.i * q.k + 2 * q.j * q.r;
            data7 = 2 * q.j * q.k - 2 * q.i * q.r;
            data8 = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
        }

        /// <summary>
        /// Interpolates a couple of matrices.
        /// </summary>
        public static Matrix3 LinearInterpolate(Matrix3 a, Matrix3 b, double prop)
        {
            Matrix3 result = new Matrix3();
            for (int i = 0; i < 9; i++)
                result[i] = a[i] * (1 - prop) + b[i] * prop;

            return result;
        }
    }

}