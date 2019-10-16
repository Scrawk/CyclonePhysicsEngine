using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Cyclone.Core;
using QUATERNION = Cyclone.Core.Quaternion;

namespace UnityEngine
{

    public static class MathExtension
    {

        public static QUATERNION ToQuaternion(this Quaternion q)
        {
            return new QUATERNION(q.w, q.x, q.y, q.z);
        }

        public static Quaternion ToQuaternion(this QUATERNION q)
        {
            return new Quaternion((float)q.i, (float)q.j, (float)q.k, (float)q.r);
        }

        public static Vector3 ToVector3(this Vector3d v)
        {
            return new Vector3((float)v.x, (float)v.y, (float)v.z);
        }

        public static Vector4 ToVector4(this Vector3d v)
        {
            return new Vector4((float)v.x, (float)v.y, (float)v.z, 1);
        }

        public static Vector3d ToVector3d(this Vector3 v)
        {
            return new Vector3d(v.x, v.y, v.z);
        }
    }

}
