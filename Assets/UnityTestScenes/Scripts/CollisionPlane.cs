using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Cyclone.Core;

using PLANE = Cyclone.Rigid.Collisions.CollisionPlane;

namespace CycloneUnityTestScenes
{
    public class CollisionPlane : MonoBehaviour
    {

        private PLANE m_plane;

        void Start()
        {
            double y = transform.position.y;

            m_plane = new PLANE(Vector3d.UnitY, y);

            RigidPhysicsEngine.Instance.Collisions.Planes.Add(m_plane);
        }

    }
}
