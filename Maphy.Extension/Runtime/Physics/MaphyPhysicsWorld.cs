using System.Collections.Generic;
using Maphy.Mathematics;
using Maphy.Physics;
using UnityEngine;
using UnityBounds = UnityEngine.Bounds;
using UnityRay = UnityEngine.Ray;

namespace Maphy.Unity
{
    [DefaultExecutionOrder(-100)]
    public sealed class MaphyPhysicsWorld : MonoBehaviour
    {
        [Header("Simulation")]
        [SerializeField] private bool simulateOnFixedUpdate = true;
        [SerializeField] private bool autoRegisterChildren = true;
        [SerializeField] private bool enableGravity = true;
        [SerializeField] private Vector3 gravity = new Vector3(0f, -10f, 0f);
        [SerializeField] private float fixedTimeStep = 1f / 60f;
        [SerializeField] private int maxSubSteps = 4;

        [Header("Capacity")]
        [SerializeField] private int bodyCapacity = 256;
        [SerializeField] private int colliderCapacity = 512;
        [SerializeField] private int pairCapacity = 2048;
        [SerializeField] private int contactManifoldCapacity = 2048;

        [Header("Gizmos")]
        [SerializeField] private bool drawRuntimeColliderBounds = true;
        [SerializeField] private bool drawContactNormals = true;
        [SerializeField] private float contactNormalLength = 0.25f;
        [SerializeField] private Color runtimeBoundsColor = new Color(0.2f, 0.7f, 1f, 0.7f);
        [SerializeField] private Color contactNormalColor = new Color(1f, 0.35f, 0.15f, 0.9f);

        private readonly List<MaphyBody> bodies = new List<MaphyBody>(128);
        private readonly List<MaphyBody> discoveryScratch = new List<MaphyBody>(128);
        private readonly List<MaphyCollider> colliderGizmoScratch = new List<MaphyCollider>(128);
        private PhysicsWorld world;

        public PhysicsWorld World => EnsureWorld();
        public PhysicsWorldStepStats LastStepStats => EnsureWorld().LastStepStats;
        public PhysicsWorldCapacity Capacity => new PhysicsWorldCapacity(
            bodyCapacity,
            colliderCapacity,
            pairCapacity,
            contactManifoldCapacity);

        private void Awake()
        {
            EnsureWorld();
            if (autoRegisterChildren)
            {
                RegisterBodiesInChildren();
            }
        }

        private void OnEnable()
        {
            EnsureWorld();
            if (autoRegisterChildren)
            {
                RegisterBodiesInChildren();
            }
        }

        private void OnDisable()
        {
            for (int i = bodies.Count - 1; i >= 0; i--)
            {
                if (bodies[i] != null)
                {
                    bodies[i].ReleaseFromWorld(this);
                }
            }

            bodies.Clear();
            world = null;
        }

        private void FixedUpdate()
        {
            if (simulateOnFixedUpdate)
            {
                Step(Time.fixedDeltaTime);
            }
        }

        private void OnValidate()
        {
            fixedTimeStep = Mathf.Max(0f, fixedTimeStep);
            maxSubSteps = Mathf.Max(0, maxSubSteps);
            bodyCapacity = Mathf.Max(0, bodyCapacity);
            colliderCapacity = Mathf.Max(0, colliderCapacity);
            pairCapacity = Mathf.Max(0, pairCapacity);
            contactManifoldCapacity = Mathf.Max(0, contactManifoldCapacity);
            contactNormalLength = Mathf.Max(0f, contactNormalLength);
        }

        public void RegisterBodiesInChildren()
        {
            discoveryScratch.Clear();
            GetComponentsInChildren(true, discoveryScratch);
            for (int i = 0; i < discoveryScratch.Count; i++)
            {
                RegisterBody(discoveryScratch[i]);
            }

            discoveryScratch.Clear();
        }

        public bool RegisterBody(MaphyBody body)
        {
            if (body == null)
            {
                return false;
            }

            if (!bodies.Contains(body))
            {
                bodies.Add(body);
            }

            return body.EnsureCreated(this);
        }

        public void UnregisterBody(MaphyBody body)
        {
            if (body == null)
            {
                return;
            }

            bodies.Remove(body);
            body.ReleaseFromWorld(this);
        }

        public void Step(float deltaTime)
        {
            PhysicsWorld physicsWorld = EnsureWorld();
            for (int i = 0; i < bodies.Count; i++)
            {
                if (bodies[i] != null)
                {
                    bodies[i].PushTransformBeforeStep();
                }
            }

            physicsWorld.Step(MaphyUnityConvert.ToFix(deltaTime), MaphyUnityConvert.ToFix(fixedTimeStep), maxSubSteps);

            for (int i = 0; i < bodies.Count; i++)
            {
                if (bodies[i] != null)
                {
                    bodies[i].PullTransformAfterStep();
                }
            }
        }

        public int QueryBoundsNonAlloc(UnityBounds bounds, ColliderHandle[] results)
        {
            return EnsureWorld().QueryBoundsNonAlloc(bounds, results);
        }

        public int QueryBoundsNonAlloc(UnityBounds bounds, int layerMask, ColliderHandle[] results)
        {
            return EnsureWorld().QueryBoundsNonAlloc(bounds, layerMask, results);
        }

        public int RaycastNonAlloc(UnityRay ray, float maxDistance, PhysicsWorldRaycastHit[] results)
        {
            return EnsureWorld().RaycastNonAlloc(ray, maxDistance, results);
        }

        public int RaycastNonAlloc(UnityRay ray, float maxDistance, int layerMask, PhysicsWorldRaycastHit[] results)
        {
            return EnsureWorld().RaycastNonAlloc(ray, maxDistance, layerMask, results);
        }

        public int ShapeCastNonAlloc(PhysicsShapeData movingShape, Vector3 delta, PhysicsWorldShapeCastHit[] results)
        {
            return EnsureWorld().ShapeCastNonAlloc(movingShape, delta, results);
        }

        public int ShapeCastNonAlloc(
            PhysicsShapeData movingShape,
            Vector3 delta,
            int layerMask,
            PhysicsWorldShapeCastHit[] results)
        {
            return EnsureWorld().ShapeCastNonAlloc(movingShape, delta, layerMask, results);
        }

        public int ShapeCastNonAlloc(ColliderHandle movingCollider, Vector3 delta, PhysicsWorldShapeCastHit[] results)
        {
            return EnsureWorld().ShapeCastNonAlloc(movingCollider, delta, results);
        }

        public int ShapeCastNonAlloc(
            ColliderHandle movingCollider,
            Vector3 delta,
            int layerMask,
            PhysicsWorldShapeCastHit[] results)
        {
            return EnsureWorld().ShapeCastNonAlloc(movingCollider, delta, layerMask, results);
        }

        public bool TryGetBody(BodyHandle body, out PhysicsWorldBody bodyState)
        {
            return EnsureWorld().TryGetBody(body, out bodyState);
        }

        public bool TryGetCollider(ColliderHandle collider, out PhysicsWorldCollider colliderState)
        {
            return EnsureWorld().TryGetCollider(collider, out colliderState);
        }

        public bool TryGetColliderBounds(ColliderHandle collider, out UnityBounds bounds)
        {
            return EnsureWorld().TryGetColliderBounds(collider, out bounds);
        }

        public void RecreateWorld()
        {
            for (int i = bodies.Count - 1; i >= 0; i--)
            {
                if (bodies[i] != null)
                {
                    bodies[i].ReleaseFromWorld(this);
                }
            }

            world = CreateWorld();
            if (autoRegisterChildren)
            {
                RegisterBodiesInChildren();
            }
        }

        private PhysicsWorld EnsureWorld()
        {
            if (world == null)
            {
                world = CreateWorld();
            }

            return world;
        }

        private PhysicsWorld CreateWorld()
        {
            PhysicsWorldSettings settings = new PhysicsWorldSettings(enableGravity);
            settings.gravity = MaphyUnityConvert.ToFix3(gravity);
            settings.timeStep = MaphyUnityConvert.ToFix(fixedTimeStep);
            settings.maxSubSteps = maxSubSteps;

            PhysicsWorld physicsWorld = new PhysicsWorld(settings);
            physicsWorld.Reserve(Capacity);
            return physicsWorld;
        }

        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying || world == null)
            {
                return;
            }

            if (drawRuntimeColliderBounds)
            {
                DrawRuntimeColliderBounds();
            }

            if (drawContactNormals)
            {
                DrawContactNormals();
            }
        }

        private void DrawRuntimeColliderBounds()
        {
            colliderGizmoScratch.Clear();
            GetComponentsInChildren(true, colliderGizmoScratch);
            Color previousColor = Gizmos.color;
            Gizmos.color = runtimeBoundsColor;
            for (int i = 0; i < colliderGizmoScratch.Count; i++)
            {
                MaphyCollider collider = colliderGizmoScratch[i];
                if (collider != null && collider.Handle.IsValid && world.TryGetColliderBounds(collider.Handle, out AABB bounds))
                {
                    Gizmos.DrawWireCube(MaphyUnityConvert.ToVector3(bounds.center), MaphyUnityConvert.ToVector3(bounds.size));
                }
            }

            Gizmos.color = previousColor;
            colliderGizmoScratch.Clear();
        }

        private void DrawContactNormals()
        {
            Color previousColor = Gizmos.color;
            Gizmos.color = contactNormalColor;
            for (int i = 0; i < world.ContactManifoldCount; i++)
            {
                if (!world.TryGetContactManifold(i, out PhysicsWorldManifold manifold))
                {
                    continue;
                }

                Vector3 normal = MaphyUnityConvert.ToVector3(manifold.normal) * contactNormalLength;
                for (int j = 0; j < manifold.contactCount; j++)
                {
                    Vector3 point = MaphyUnityConvert.ToVector3(manifold[j].position);
                    Gizmos.DrawLine(point, point + normal);
                }
            }

            Gizmos.color = previousColor;
        }
    }
}
