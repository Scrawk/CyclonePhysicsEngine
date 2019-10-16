using UnityEngine;
using System;
using System.Collections.Generic;

using Cyclone.Core;

namespace Common.Unity.Drawing
{

    public enum LINE_MODE { LINES = 2, TRIANGLES = 3, TETRAHEDRON = 4 };

    public class SegmentRenderer : BaseRenderer
    {

        public SegmentRenderer()
        {
            LineMode = LINE_MODE.LINES;
        }

        public SegmentRenderer(LINE_MODE lineMode, DRAW_ORIENTATION orientation)
        {
            LineMode = LINE_MODE.LINES;
            Orientation = DRAW_ORIENTATION.XY;
        }

        public LINE_MODE LineMode { get; set; }

        #region DOUBLE

        public void Load(IList<Vector3d> vertices, IList<int> indices = null)
        {
            SetIndices(vertices.Count, indices);

            foreach (var v in vertices)
                m_vertices.Add(v.ToVector4());
        }

        public void Load(Vector3d a, Vector3d b)
        {
            SetIndices(2, null);

            m_vertices.Add(a.ToVector4());
            m_vertices.Add(b.ToVector4());
        }

        #endregion

        #region UNITY

        public void Load(IList<Vector4> vertices, IList<int> indices = null)
        {
            SetIndices(vertices.Count, indices);

            foreach (var v in vertices)
                m_vertices.Add(v);
        }

        public void Load(IList<Vector3> vertices, IList<int> indices = null)
        {
            SetIndices(vertices.Count, indices);

            foreach (var v in vertices)
                m_vertices.Add(v);
        }

        public void Load(Vector3 a, Vector3 b)
        {
            SetIndices(2, null);

            m_vertices.Add(a);
            m_vertices.Add(b);
        }

        public void Load(IList<Vector2> vertices, IList<int> indices = null)
        {
            SetIndices(vertices.Count, indices);

            foreach (var v in vertices)
                m_vertices.Add(v);
        }

        public void Load(Vector2 a, Vector2 b)
        {
            SetIndices(2, null);

            if (Orientation == DRAW_ORIENTATION.XY)
            {
                m_vertices.Add(a);
                m_vertices.Add(b);
            }
            else if (Orientation == DRAW_ORIENTATION.XZ)
            {
                m_vertices.Add(new Vector4(a.x, 0, a.y, 1));
                m_vertices.Add(new Vector4(b.x, 0, b.y, 1));
            }
        }

        #endregion

        #region DRAW

        public override void Draw(Camera camera, Matrix4x4 localToWorld)
        {
            switch (LineMode)
            {
                case LINE_MODE.LINES:
                    DrawVerticesAsLines(camera, localToWorld);
                    break;

                case LINE_MODE.TRIANGLES:
                    DrawVerticesAsTriangles(camera, localToWorld);
                    break;

                case LINE_MODE.TETRAHEDRON:
                    DrawVerticesAsTetrahedron(camera, localToWorld);
                    break;
            }
        }

        private void DrawVerticesAsLines(Camera camera, Matrix4x4 localToWorld)
        {
            GL.PushMatrix();

            GL.LoadIdentity();
            GL.modelview = camera.worldToCameraMatrix * localToWorld;
            GL.LoadProjectionMatrix(camera.projectionMatrix);

            Material.SetPass(0);
            GL.Begin(GL.LINES);
            GL.Color(Color);

            int vertexCount = m_vertices.Count;

            for (int i = 0; i < m_indices.Count / 2; i++)
            {
                int i0 = m_indices[i * 2 + 0];
                int i1 = m_indices[i * 2 + 1];

                if (i0 < 0 || i0 >= vertexCount) continue;
                if (i1 < 0 || i1 >= vertexCount) continue;

                GL.Vertex(m_vertices[i0]);
                GL.Vertex(m_vertices[i1]);
            }

            GL.End();

            GL.PopMatrix();
        }

        private void DrawVerticesAsTriangles(Camera camera, Matrix4x4 localToWorld)
        {
            GL.PushMatrix();

            GL.LoadIdentity();
            GL.MultMatrix(camera.worldToCameraMatrix * localToWorld);
            GL.LoadProjectionMatrix(camera.projectionMatrix);

            Material.SetPass(0);
            GL.Begin(GL.LINES);
            GL.Color(Color);

            int vertexCount = m_vertices.Count;

            for (int i = 0; i < m_indices.Count / 3; i++)
            {
                int i0 = m_indices[i * 3 + 0];
                int i1 = m_indices[i * 3 + 1];
                int i2 = m_indices[i * 3 + 2];

                if (i0 < 0 || i0 >= vertexCount) continue;
                if (i1 < 0 || i1 >= vertexCount) continue;
                if (i2 < 0 || i2 >= vertexCount) continue;

                GL.Vertex(m_vertices[i0]);
                GL.Vertex(m_vertices[i1]);

                GL.Vertex(m_vertices[i0]);
                GL.Vertex(m_vertices[i2]);

                GL.Vertex(m_vertices[i2]);
                GL.Vertex(m_vertices[i1]);
            }

            GL.End();

            GL.PopMatrix();
        }

        private void DrawVerticesAsTetrahedron(Camera camera, Matrix4x4 localToWorld)
        {
            GL.PushMatrix();

            GL.LoadIdentity();
            GL.MultMatrix(camera.worldToCameraMatrix * localToWorld);
            GL.LoadProjectionMatrix(camera.projectionMatrix);

            Material.SetPass(0);
            GL.Begin(GL.LINES);
            GL.Color(Color);

            int vertexCount = m_vertices.Count;

            for (int i = 0; i < m_indices.Count / 4; i++)
            {
                int i0 = m_indices[i * 4 + 0];
                int i1 = m_indices[i * 4 + 1];
                int i2 = m_indices[i * 4 + 2];
                int i3 = m_indices[i * 4 + 3];

                if (i0 < 0 || i0 >= vertexCount) continue;
                if (i1 < 0 || i1 >= vertexCount) continue;
                if (i2 < 0 || i2 >= vertexCount) continue;
                if (i3 < 0 || i3 >= vertexCount) continue;

                GL.Vertex(m_vertices[i0]);
                GL.Vertex(m_vertices[i1]);

                GL.Vertex(m_vertices[i0]);
                GL.Vertex(m_vertices[i2]);

                GL.Vertex(m_vertices[i0]);
                GL.Vertex(m_vertices[i3]);

                GL.Vertex(m_vertices[i1]);
                GL.Vertex(m_vertices[i2]);

                GL.Vertex(m_vertices[i3]);
                GL.Vertex(m_vertices[i2]);

                GL.Vertex(m_vertices[i1]);
                GL.Vertex(m_vertices[i3]);
            }

            GL.End();

            GL.PopMatrix();
        }

        #endregion

    }

}