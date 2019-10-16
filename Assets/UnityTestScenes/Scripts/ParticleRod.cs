using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Common.Unity.Drawing;
using Cyclone.Core;
using Cyclone.Particles;
using Cyclone.Particles.Forces;
using Cyclone.Particles.Constraints;

namespace CycloneUnityTestScenes
{

    public class ParticleRod : MonoBehaviour
    {

        List<Particle> m_particles;

        SegmentRenderer m_lines;

        VertexRenderer m_verts;

        void Start()
        {
            m_lines = new SegmentRenderer();
            m_lines.Color = Color.red;

            m_verts = new VertexRenderer(0.05f);
            m_verts.Color = Color.yellow;

            CreateParticles();
        }

        private void CreateParticles()
        {
            double mass = 1;
            double damping = 0.5;
            double len = 1;

            var pos = transform.position.ToVector3d();

            var p0 = new Particle();
            p0.Position = pos + new Vector3d(0,0,0);
            p0.SetMass(0);
            p0.Damping = damping;

            var p1 = new Particle();
            p1.Position = pos + new Vector3d(1, 0, 0);
            p1.SetMass(mass);
            p1.Damping = damping;

            var p2 = new Particle();
            p2.Position = pos + new Vector3d(3, 0, 0);
            p2.SetMass(mass);
            p2.Damping = damping;

            var p3 = new Particle();
            p3.Position = pos + new Vector3d(3, 0, 0);
            p3.SetMass(mass);
            p3.Damping = damping;

            m_particles = new List<Particle>();
            m_particles.Add(p0);
            m_particles.Add(p1);
            m_particles.Add(p2);
            m_particles.Add(p3);

            ParticlePhysicsEngine.Instance.Particles.AddRange(m_particles);
            ParticlePhysicsEngine.Instance.Constraints.Add(new ParticleRodConstraint(p0, p1, len));
            ParticlePhysicsEngine.Instance.Constraints.Add(new ParticleRodConstraint(p1, p2, len));
            ParticlePhysicsEngine.Instance.Constraints.Add(new ParticleRodConstraint(p2, p3, len));
        }

        private void OnRenderObject()
        {
            var cam = Camera.current;
            if (cam == null) return;

            var points = new List<Vector3d>();
            foreach (var p in m_particles)
                points.Add(p.Position);

            m_lines.Clear();
            m_lines.Load(points);

            m_verts.Clear();
            m_verts.Load(points);

            m_lines.Draw(cam);
            m_verts.Draw(cam);
        }

    }

}
