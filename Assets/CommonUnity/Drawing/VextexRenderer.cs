using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Cyclone.Core;

namespace Common.Unity.Drawing
{

    public class VertexRenderer : BaseRenderer
    {

        public VertexRenderer(float size)
        {
            Size = size;
        }

        public VertexRenderer(float size, DRAW_ORIENTATION orientation)
        {
            Size = size;
            Orientation = DRAW_ORIENTATION.XY;
        }

        public float Size = 0.1f;

        #region DOUBLE
        public  void Load(IEnumerable<Vector3d> vertices)
        {
            foreach (var v in vertices)
            {
                m_vertices.Add(v.ToVector4());
            }
        }

        public  void Load(Vector3d vertex)
        {
            m_vertices.Add(vertex.ToVector4());
        }

        #endregion

        #region UNITY
        public  void Load(IEnumerable<Vector3> vertices)
        {
            foreach (var v in vertices)
            {
                m_vertices.Add(v);
            }
        }

        public  void Load(IEnumerable<Vector2> vertices)
        {
            foreach (var v in vertices)
            {
                m_vertices.Add(v);
            }
        }
        #endregion

        #region DRAW
        public override void Draw(Camera camera, Matrix4x4 localToWorld)
        {
            GL.PushMatrix();

            GL.LoadIdentity();
            GL.modelview = camera.worldToCameraMatrix * LocalToWorld;
            GL.LoadProjectionMatrix(camera.projectionMatrix);

            Material.SetPass(0);
            GL.Begin(GL.QUADS);
            GL.Color(Color);

            switch (Orientation)
            {
                case DRAW_ORIENTATION.XY:
                    DrawXY();
                    break;

                case DRAW_ORIENTATION.XZ:
                    DrawXZ();
                    break;
            }

            GL.End();

            GL.PopMatrix();
        }

        private  void DrawXY()
        {
            float half = Size * 0.5f;
            for (int i = 0; i < m_vertices.Count; i++)
            {
                float x = m_vertices[i].x;
                float y = m_vertices[i].y;
                float z = m_vertices[i].z;

                GL.Vertex3(x + half, y + half, z);
                GL.Vertex3(x + half, y - half, z);
                GL.Vertex3(x - half, y - half, z);
                GL.Vertex3(x - half, y + half, z);
            }
        }

        private  void DrawXZ()
        {
            float half = Size * 0.5f;
            for (int i = 0; i < m_vertices.Count; i++)
            {
                float x = m_vertices[i].x;
                float y = m_vertices[i].y;
                float z = m_vertices[i].z;

                GL.Vertex3(x + half, y, z + half);
                GL.Vertex3(x + half, y, z - half);
                GL.Vertex3(x - half, y, z - half);
                GL.Vertex3(x - half, y, z + half);
            }
        }
        #endregion
    }

}
