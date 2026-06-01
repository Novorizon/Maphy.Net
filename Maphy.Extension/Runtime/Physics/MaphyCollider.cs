using Maphy.Mathematics;
using Maphy.Physics;
using UnityEngine;
using PhysicsCollider = Maphy.Physics.Collider;
using PhysicsMaterial = Maphy.Physics.Material;
using UnityQuaternion = UnityEngine.Quaternion;

namespace Maphy.Unity
{
    public sealed class MaphyCollider : MonoBehaviour
    {
        [SerializeField] private MaphyBody body;
        [SerializeField] private MaphyUnityColliderShape shape = MaphyUnityColliderShape.Sphere;
        [SerializeField] private Vector3 center;
        [SerializeField] private Vector3 size = Vector3.one;
        [SerializeField] private Vector3 rotationEuler;
        [SerializeField] private float radius = 0.5f;
        [SerializeField] private float height = 2f;
        [SerializeField] private bool isTrigger;
        [SerializeField] private int layer;
        [SerializeField] private int collisionMask = PhysicsCollider.AllLayers;
        [SerializeField] private float density = 1f;
        [SerializeField] private float friction = 0.5f;
        [SerializeField] private float bounciness;

        [Header("Gizmos")]
        [SerializeField] private bool drawAuthoringGizmo = true;
        [SerializeField] private Color colliderGizmoColor = new Color(0.3f, 0.85f, 0.35f, 0.75f);
        [SerializeField] private Color triggerGizmoColor = new Color(1f, 0.85f, 0.2f, 0.75f);

        private ColliderHandle handle = ColliderHandle.Invalid;

        public ColliderHandle Handle => handle;
        public bool IsCreated => handle.IsValid;
        public MaphyBody BodyComponent => body;

        private void Reset()
        {
            body = GetComponentInParent<MaphyBody>();
        }

        private void OnEnable()
        {
            MaphyBody owner = ResolveBody();
            if (owner != null)
            {
                owner.RegisterCollider(this);
            }
        }

        private void OnDisable()
        {
            if (body != null)
            {
                body.UnregisterCollider(this);
            }
            else
            {
                ReleaseFromBody(null);
            }
        }

        private void OnValidate()
        {
            size = new Vector3(
                Mathf.Max(0f, size.x),
                Mathf.Max(0f, size.y),
                Mathf.Max(0f, size.z));
            radius = Mathf.Max(0f, radius);
            height = Mathf.Max(0f, height);
            layer = Mathf.Clamp(layer, 0, 31);
            density = Mathf.Max(0f, density);
            friction = Mathf.Max(0f, friction);
            bounciness = Mathf.Clamp01(bounciness);

            if (Application.isPlaying && handle.IsValid && isActiveAndEnabled)
            {
                RebuildCollider();
            }
        }

        public MaphyBody ResolveBody()
        {
            if (body == null)
            {
                body = GetComponentInParent<MaphyBody>();
            }

            return body;
        }

        public bool EnsureCreated(MaphyBody owner)
        {
            if (owner == null || !owner.IsCreated || owner.WorldComponent == null)
            {
                return false;
            }

            if (handle.IsValid && body == owner)
            {
                return ApplyColliderSettings();
            }

            ReleaseFromBody(body);
            body = owner;
            PhysicsWorld physicsWorld = owner.WorldComponent.World;
            bool created = false;
            switch (shape)
            {
                case MaphyUnityColliderShape.AABB:
                    created = physicsWorld.AddLocalAABB(owner.Handle, center, size, out handle);
                    break;
                case MaphyUnityColliderShape.OBB:
                    created = physicsWorld.AddLocalOBB(owner.Handle, center, size, UnityQuaternion.Euler(rotationEuler), out handle);
                    break;
                case MaphyUnityColliderShape.Capsule:
                    created = physicsWorld.AddLocalCapsule(owner.Handle, center, radius, height, UnityQuaternion.Euler(rotationEuler), out handle);
                    break;
                default:
                    created = physicsWorld.AddLocalSphere(owner.Handle, center, radius, out handle);
                    break;
            }

            if (!created)
            {
                handle = ColliderHandle.Invalid;
                return false;
            }

            return ApplyColliderSettings();
        }

        public bool ApplyColliderSettings()
        {
            PhysicsWorld physicsWorld = body != null && body.WorldComponent != null ? body.WorldComponent.World : null;
            if (physicsWorld == null || !handle.IsValid)
            {
                return false;
            }

            PhysicsMaterial material = new PhysicsMaterial(
                MaphyUnityConvert.ToFix(density),
                MaphyUnityConvert.ToFix(friction),
                MaphyUnityConvert.ToFix(bounciness));

            physicsWorld.SetColliderTrigger(handle, isTrigger);
            physicsWorld.SetColliderLayer(handle, layer);
            physicsWorld.SetColliderCollisionMask(handle, collisionMask);
            physicsWorld.SetColliderMaterial(handle, material);
            return true;
        }

        public void RebuildCollider()
        {
            MaphyBody owner = ResolveBody();
            ReleaseFromBody(owner);
            if (owner != null)
            {
                owner.RegisterCollider(this);
            }
        }

        public bool SetTrigger(bool value)
        {
            isTrigger = value;
            PhysicsWorld physicsWorld = body != null && body.WorldComponent != null ? body.WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetColliderTrigger(handle, value);
        }

        public bool SetLayer(int value)
        {
            layer = Mathf.Clamp(value, 0, 31);
            PhysicsWorld physicsWorld = body != null && body.WorldComponent != null ? body.WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetColliderLayer(handle, layer);
        }

        public bool SetCollisionMask(int value)
        {
            collisionMask = value;
            PhysicsWorld physicsWorld = body != null && body.WorldComponent != null ? body.WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetColliderCollisionMask(handle, value);
        }

        public bool SetMaterial(float densityValue, float frictionValue, float bouncinessValue)
        {
            density = Mathf.Max(0f, densityValue);
            friction = Mathf.Max(0f, frictionValue);
            bounciness = Mathf.Clamp01(bouncinessValue);
            return ApplyColliderSettings();
        }

        public bool TryGetRuntimeBounds(out Bounds bounds)
        {
            bounds = default;
            return body != null
                && body.WorldComponent != null
                && handle.IsValid
                && body.WorldComponent.TryGetColliderBounds(handle, out bounds);
        }

        public void ReleaseFromBody(MaphyBody owner)
        {
            MaphyBody currentBody = body != null ? body : owner;
            if (handle.IsValid && currentBody != null && currentBody.WorldComponent != null)
            {
                currentBody.WorldComponent.World.DestroyCollider(handle);
            }

            handle = ColliderHandle.Invalid;
        }

        private void OnDrawGizmosSelected()
        {
            if (!drawAuthoringGizmo)
            {
                return;
            }

            Color previousColor = Gizmos.color;
            Matrix4x4 previousMatrix = Gizmos.matrix;
            Gizmos.color = isTrigger ? triggerGizmoColor : colliderGizmoColor;

            switch (shape)
            {
                case MaphyUnityColliderShape.AABB:
                    Gizmos.matrix = transform.localToWorldMatrix;
                    Gizmos.DrawWireCube(center, size);
                    break;
                case MaphyUnityColliderShape.OBB:
                    Gizmos.matrix = transform.localToWorldMatrix * Matrix4x4.TRS(center, UnityQuaternion.Euler(rotationEuler), Vector3.one);
                    Gizmos.DrawWireCube(Vector3.zero, size);
                    break;
                case MaphyUnityColliderShape.Capsule:
                    Gizmos.matrix = transform.localToWorldMatrix * Matrix4x4.TRS(center, UnityQuaternion.Euler(rotationEuler), Vector3.one);
                    DrawLocalCapsuleGizmo();
                    break;
                default:
                    Gizmos.matrix = transform.localToWorldMatrix;
                    Gizmos.DrawWireSphere(center, radius);
                    break;
            }

            Gizmos.matrix = previousMatrix;
            Gizmos.color = previousColor;
        }

        private void DrawLocalCapsuleGizmo()
        {
            float safeRadius = Mathf.Max(0f, radius);
            float halfSegment = Mathf.Max(0f, height * 0.5f - safeRadius);
            Vector3 top = Vector3.up * halfSegment;
            Vector3 bottom = -top;

            Gizmos.DrawWireSphere(top, safeRadius);
            Gizmos.DrawWireSphere(bottom, safeRadius);
            Gizmos.DrawLine(top + Vector3.right * safeRadius, bottom + Vector3.right * safeRadius);
            Gizmos.DrawLine(top - Vector3.right * safeRadius, bottom - Vector3.right * safeRadius);
            Gizmos.DrawLine(top + Vector3.forward * safeRadius, bottom + Vector3.forward * safeRadius);
            Gizmos.DrawLine(top - Vector3.forward * safeRadius, bottom - Vector3.forward * safeRadius);
        }
    }
}
