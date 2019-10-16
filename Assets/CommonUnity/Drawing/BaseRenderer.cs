using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Common.Unity.Drawing
{

    public enum DRAW_ORIENTATION { XY, XZ };

    public abstract class BaseRenderer
    {
        public static readonly IList<int> CUBE_INDICES = new int[]
        {
            0, 1, 1, 2, 2, 3, 3, 0,
            4, 5, 5, 6, 6, 7, 7, 4,
            0, 4, 1, 5, 2, 6, 3, 7
        };

        public static readonly IList<int> SQUARE_INDICES = new int[]
        {
            0, 1, 1, 2, 2, 3, 3, 0
        };

        protected List<Vector4> m_vertices = new List<Vector4>();

        protected List<int> m_indices = new List<int>();

        public BaseRenderer()
        {
            LocalToWorld = Matrix4x4.identity;
            Orientation = DRAW_ORIENTATION.XY;
            Color = Color.white;
            Material = new Material(Shader.Find("Hidden/Internal-Colored"));
        }

        public Matrix4x4 LocalToWorld { get; set; }

        public DRAW_ORIENTATION Orientation { get; set;  }

        public Color Color { get; set; }

        public CompareFunction ZTest
        {
            get { return (CompareFunction)Material.GetInt("_ZTest"); }
            set { Material.SetInt("_ZTest", (int)CompareFunction.Always); }
        }

        protected Material Material { get; set; }

        public virtual void Clear()
        {
            m_vertices.Clear();
            m_indices.Clear();
        }

        public void SetIndices(int vertexCount, IList<int> indices)
        {
            int current = m_vertices.Count;

            if (indices == null)
            {
                for (int i = 0; i < vertexCount - 1; i++)
                {
                    m_indices.Add(i + current);
                    m_indices.Add(i + 1 + current);
                }
            }
            else
            {
                for (int i = 0; i < indices.Count; i++)
                        m_indices.Add(indices[i] + current);
            }
            
        }

        public void Draw(Camera camera)
        {
            Draw(camera, Matrix4x4.identity);
        }

        public abstract void Draw(Camera camera, Matrix4x4 localToWorld);

    }

}
