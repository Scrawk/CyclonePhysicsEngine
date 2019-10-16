using System;
using System.Collections.Generic;

namespace Cyclone.Core
{

    /// <summary>
    /// Holds a transform matrix, consisting of a rotation matrix and
    /// a position.The matrix has 12 elements, it is assumed that the
    /// remaining four are(0,0,0,1); producing a homogenous matrix.
    /// </summary>
    public struct Matrix4
    {

        public double data0, data1, data2, data3;
        public double data4, data5, data6, data7;
        public double data8, data9, data10, data11;


        /// <summary>
        /// Creates an identity matrix.
        /// </summary>
        static readonly public Matrix4 Identity = new Matrix4(1, 0, 0, 0,
                                                              0, 1, 0, 0,
                                                              0, 0, 1, 0);


        public Matrix4(double m0, double m1, double m2, double m3,
                       double m4, double m5, double m6, double m7,
                       double m8, double m9, double m10, double m11)
        {
            data0 = m0; data1 = m1; data2 = m2; data3 = m3;
            data4 = m4; data5 = m5; data6 = m6; data7 = m7;
            data8 = m8; data9 = m9; data10 = m10; data11 = m11;
        }

        /// <summary>
        /// Access the varible at index i
        /// </summary>
        unsafe public double this[int i]
        {
            get
            {
                if ((uint)i >= 12)
                    throw new IndexOutOfRangeException("Matrix4 index out of range.");

                fixed (Matrix4* array = &this) { return ((double*)array)[i]; }
            }
            set
            {
                if ((uint)i >= 12)
                    throw new IndexOutOfRangeException("Matrix4 index out of range.");

                fixed (double* array = &data0) { array[i] = value; }
            }
        }

        /// <summary>
        /// Returns a matrix which is this matrix multiplied by the given
        /// other matrix.
        /// </summary>
        public static Matrix4 operator *(Matrix4 m1, Matrix4 m2)
        {
            Matrix4 result = new Matrix4();
            result.data0 = (m2.data0 * m1.data0) + (m2.data4 * m1.data1) + (m2.data8 * m1.data2);
            result.data4 = (m2.data0 * m1.data4) + (m2.data4 * m1.data5) + (m2.data8 * m1.data6);
            result.data8 = (m2.data0 * m1.data8) + (m2.data4 * m1.data9) + (m2.data8 * m1.data10);

            result.data1 = (m2.data1 * m1.data0) + (m2.data5 * m1.data1) + (m2.data9 * m1.data2);
            result.data5 = (m2.data1 * m1.data4) + (m2.data5 * m1.data5) + (m2.data9 * m1.data6);
            result.data9 = (m2.data1 * m1.data8) + (m2.data5 * m1.data9) + (m2.data9 * m1.data10);

            result.data2 = (m2.data2 * m1.data0) + (m2.data6 * m1.data1) + (m2.data10 * m1.data2);
            result.data6 = (m2.data2 * m1.data4) + (m2.data6 * m1.data5) + (m2.data10 * m1.data6);
            result.data10 = (m2.data2 * m1.data8) + (m2.data6 * m1.data9) + (m2.data10 * m1.data10);

            result.data3 = (m2.data3 * m1.data0) + (m2.data7 * m1.data1) + (m2.data11 * m1.data2) + m1.data3;
            result.data7 = (m2.data3 * m1.data4) + (m2.data7 * m1.data5) + (m2.data11 * m1.data6) + m1.data7;
            result.data11 = (m2.data3 * m1.data8) + (m2.data7 * m1.data9) + (m2.data11 * m1.data10) + m1.data11;

            return result;
        }

        /// <summary>
        /// Transform the given vector by this matrix.
        /// </summary>
        public static Vector3d operator *(Matrix4 m, Vector3d vector)
        {
            return new Vector3d(
                vector.x * m.data0 +
                vector.y * m.data1 +
                vector.z * m.data2 + m.data3,

                vector.x * m.data4 +
                vector.y * m.data5 +
                vector.z * m.data6 + m.data7,

                vector.x * m.data8 +
                vector.y * m.data9 +
                vector.z * m.data10 + m.data11
            );
        }

        /// <summary>
        /// A matrix as a string.
        /// </summary>
        public override string ToString()
        {
            return data0 + "," + data1 + "," + data2 + "," + data3 + "\n" +
                   data4 + "," + data5 + "," + data6 + "," + data7 + "\n" +
                   data8 + "," + data9 + "," + data10 + "," + data11;
        }

        /// <summary>
        /// Sets the matrix to be a diagonal matrix with the given coefficients.
        /// </summary>
        public void SetDiagonal(double a, double b, double c)
        {
            data0 = a;
            data5 = b;
            data10 = c;
        }

        /// <summary>
        /// Transform the given vector by this matrix.
        /// </summary>
        public Vector3d Transform(Vector3d vector)
        {
            return this * vector;
        }

        /// <summary>
        /// Returns the determinant of the matrix.
        /// </summary>
        public double GetDeterminant()
        {
            return -data8 * data5 * data2 +
            data4 * data9 * data2 +
            data8 * data1 * data6 -
            data0 * data9 * data6 -
            data4 * data1 * data10 +
            data0 * data5 * data10;
        }

        /// <summary>
        /// Sets the matrix to be the inverse of the given matrix.
        /// </summary>
        public void SetInverse(Matrix4 m)
        {
            // Make sure the determinant is non-zero.
            double det = m.GetDeterminant();
            if (det == 0) return;
            det = 1.0 / det;

            data0 = (-m.data9 * m.data6 + m.data5 * m.data10) * det;
            data4 = (m.data8 * m.data6 - m.data4 * m.data10) * det;
            data8 = (-m.data8 * m.data5 + m.data4 * m.data9) * det;

            data1 = (m.data9 * m.data2 - m.data1 * m.data10) * det;
            data5 = (-m.data8 * m.data2 + m.data0 * m.data10) * det;
            data9 = (m.data8 * m.data1 - m.data0 * m.data9) * det;

            data2 = (-m.data5 * m.data2 + m.data1 * m.data6) * det;
            data6 = (+m.data4 * m.data2 - m.data0 * m.data6) * det;
            data10 = (-m.data4 * m.data1 + m.data0 * m.data5) * det;

            data3 = (m.data9 * m.data6 * m.data3
                       - m.data5 * m.data10 * m.data3
                       - m.data9 * m.data2 * m.data7
                       + m.data1 * m.data10 * m.data7
                       + m.data5 * m.data2 * m.data11
                       - m.data1 * m.data6 * m.data11) * det;
            data7 = (-m.data8 * m.data6 * m.data3
                       + m.data4 * m.data10 * m.data3
                       + m.data8 * m.data2 * m.data7
                       - m.data0 * m.data10 * m.data7
                       - m.data4 * m.data2 * m.data11
                       + m.data0 * m.data6 * m.data11) * det;
            data11 = (m.data8 * m.data5 * m.data3
                       - m.data4 * m.data9 * m.data3
                       - m.data8 * m.data1 * m.data7
                       + m.data0 * m.data9 * m.data7
                       + m.data4 * m.data1 * m.data11
                       - m.data0 * m.data5 * m.data11) * det;
        }

        /// <summary>
        /// Returns a new matrix containing the inverse of this matrix.
        /// </summary>
        public Matrix4 Inverse()
        {
            Matrix4 result = Matrix4.Identity;
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
        /// Transform the given direction vector by this matrix.
        ///
        /// @note When a direction is converted between frames of
        /// reference, there is no translation required.
        /// </summary>
        public Vector3d TransformDirection(Vector3d vector)
        {
            return new Vector3d(
                vector.x * data0 +
                vector.y * data1 +
                vector.z * data2,

                vector.x * data4 +
                vector.y * data5 +
                vector.z * data6,

                vector.x * data8 +
                vector.y * data9 +
                vector.z * data10
            );
        }

        /// <summary>
        /// Transform the given direction vector by the
        /// transformational inverse of this matrix.
        ///
        /// @note This function relies on the fact that the inverse of
        /// a pure rotation matrix is its transpose. It separates the
        /// translational and rotation components, transposes the
        /// rotation, and multiplies out. If the matrix is not a
        /// scale and shear free transform matrix, then this function
        /// will not give correct results.
        ///
        /// @note When a direction is converted between frames of
        /// reference, there is no translation required.
        /// </summary>
        public Vector3d TransformInverseDirection(Vector3d vector)
        {
            return new Vector3d(
                vector.x * data0 +
                vector.y * data4 +
                vector.z * data8,

                vector.x * data1 +
                vector.y * data5 +
                vector.z * data9,

                vector.x * data2 +
                vector.y * data6 +
                vector.z * data10
            );
        }

        /// <summary>
        /// Transform the given vector by the transformational inverse
        /// of this matrix.
        ///
        /// @note This function relies on the fact that the inverse of
        /// a pure rotation matrix is its transpose. It separates the
        /// translational and rotation components, transposes the
        /// rotation, and multiplies out. If the matrix is not a
        /// scale and shear free transform matrix, then this function
        /// will not give correct results.
        /// </summary>
        public Vector3d TransformInverse(Vector3d vector)
        {
            Vector3d tmp = vector;
            tmp.x -= data3;
            tmp.y -= data7;
            tmp.z -= data11;
            return new Vector3d(
                tmp.x * data0 +
                tmp.y * data4 +
                tmp.z * data8,

                tmp.x * data1 +
                tmp.y * data5 +
                tmp.z * data9,

                tmp.x * data2 +
                tmp.y * data6 +
                tmp.z * data10
            );
        }

        /// <summary>
        /// Gets a vector representing one axis (i.e. one column) in the matrix.
        /// </summary>
        /// <param name="i">The row to return. Row 3 corresponds 
        /// to the position of the transform matrix.</param>
        /// <returns></returns>
        public Vector3d GetAxisVector(int i)
        {
            return new Vector3d(this[i], this[i + 4], this[i + 8]);
        }

        /**
         * Sets this matrix to be the rotation matrix corresponding to
         * the given quaternion.
         */
        public void SetOrientationAndPos(Quaternion q, Vector3d pos)
        {
            data0 = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            data1 = 2 * q.i * q.j + 2 * q.k * q.r;
            data2 = 2 * q.i * q.k - 2 * q.j * q.r;
            data3 = pos.x;

            data4 = 2 * q.i * q.j - 2 * q.k * q.r;
            data5 = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
            data6 = 2 * q.j * q.k + 2 * q.i * q.r;
            data7 = pos.y;

            data8 = 2 * q.i * q.k + 2 * q.j * q.r;
            data9 = 2 * q.j * q.k - 2 * q.i * q.r;
            data10 = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
            data11 = pos.z;
        }

    }

}