using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Cyclone.Particles;
using Cyclone.Particles.Forces;
using Cyclone.Particles.Constraints;

namespace CycloneUnityTestScenes
{

    public class ParticlePhysicsEngine : MonoBehaviour
    {
        public int iterations = 0;

        public int maxContacts = 100;

        public static ParticleEngine Instance { get; private set; }

        private void Awake()
        {
            Instance = new ParticleEngine(maxContacts);
            Instance.Resolver.Iterations = iterations;

            Instance.ForceAreas.Add(new ParticleGravityForce(-9.81));

            //Instance.Constraints.Add(new PlaneConstraint(-2));
        }

        private void FixedUpdate()
        {
            double dt = Time.fixedDeltaTime;

            Instance.StartFrame();
            Instance.RunPhysics(dt);
        }
    }

}
