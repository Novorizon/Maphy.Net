using System.Collections.Generic;
using Maphy.Mathematics;
using Maphy.Physics;
using UnityEngine;

namespace Maphy.Unity
{
    [DisallowMultipleComponent]
    public sealed class MaphyBody : MonoBehaviour
    {
        [SerializeField] private MaphyPhysicsWorld world;
        [SerializeField] private RigidType bodyType = RigidType.Dynamic;
        [SerializeField] private MaphyUnityTransformSyncMode transformSync = MaphyUnityTransformSyncMode.Dynamic;
        [SerializeField] private bool useGravity = true;
        [SerializeField] private bool allowSleep = true;
        [SerializeField] private bool useCCD = true;
        [SerializeField] private bool autoMass = true;
        [SerializeField] private bool autoInertia = true;
        [SerializeField] private float mass = 1f;
        [SerializeField] private Vector3 inertia = Vector3.one;
        [SerializeField] private Vector3 initialVelocity;
        [SerializeField] private Vector3 initialAngularVelocity;

        private readonly List<MaphyCollider> colliderScratch = new List<MaphyCollider>(4);
        private MaphyPhysicsWorld registeredWorld;
        private BodyHandle handle = BodyHandle.Invalid;

        public BodyHandle Handle => handle;
        public bool IsCreated => handle.IsValid;
        public MaphyPhysicsWorld WorldComponent => registeredWorld != null ? registeredWorld : world;
        public MaphyUnityTransformSyncMode TransformSync
        {
            get { return transformSync; }
            set { transformSync = value; }
        }

        private void Reset()
        {
            world = GetComponentInParent<MaphyPhysicsWorld>();
        }

        private void OnEnable()
        {
            MaphyPhysicsWorld owner = ResolveWorld();
            if (owner != null)
            {
                owner.RegisterBody(this);
            }
        }

        private void OnDisable()
        {
            if (registeredWorld != null)
            {
                registeredWorld.UnregisterBody(this);
            }
            else
            {
                ReleaseFromWorld(world);
            }
        }

        private void OnValidate()
        {
            mass = Mathf.Max(0f, mass);
            inertia = new Vector3(
                Mathf.Max(0f, inertia.x),
                Mathf.Max(0f, inertia.y),
                Mathf.Max(0f, inertia.z));

            if (Application.isPlaying && handle.IsValid)
            {
                ApplyBodySettings(applyInitialVelocity: false);
            }
        }

        public bool EnsureCreated(MaphyPhysicsWorld owner)
        {
            if (owner == null)
            {
                return false;
            }

            if (handle.IsValid && registeredWorld == owner)
            {
                return true;
            }

            ReleaseFromWorld(registeredWorld);
            registeredWorld = owner;
            world = owner;

            PhysicsWorld physicsWorld = owner.World;
            MaphyUnityConvert.ReadTransform(transform, out fix3 position, out quaternion rotation);
            if (!physicsWorld.CreateBody(position, rotation, bodyType, out handle))
            {
                handle = BodyHandle.Invalid;
                return false;
            }

            ApplyBodySettings();
            RegisterChildColliders();
            return true;
        }

        public bool ApplyBodySettings()
        {
            return ApplyBodySettings(applyInitialVelocity: true);
        }

        public bool SetVelocity(Vector3 velocity)
        {
            initialVelocity = velocity;
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetBodyVelocity(handle, MaphyUnityConvert.ToFix3(velocity));
        }

        public bool SetAngularVelocity(Vector3 angularVelocity)
        {
            initialAngularVelocity = angularVelocity;
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetBodyAngularVelocity(handle, MaphyUnityConvert.ToFix3(angularVelocity));
        }

        public bool Teleport(Vector3 position, Quaternion rotation)
        {
            transform.SetPositionAndRotation(position, rotation);
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null
                && handle.IsValid
                && physicsWorld.SetBodyTransform(
                    handle,
                    MaphyUnityConvert.ToFix3(position),
                    MaphyUnityConvert.ToNormalizedFixQuaternion(rotation));
        }

        public bool SetBodyType(RigidType type)
        {
            bodyType = type;
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetBodyType(handle, type);
        }

        public bool SetUseGravity(bool value)
        {
            useGravity = value;
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetBodyGravity(handle, value);
        }

        public bool SetAllowSleep(bool value)
        {
            allowSleep = value;
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetBodyAllowSleep(handle, value);
        }

        public bool SetCCD(bool value)
        {
            useCCD = value;
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetBodyCCD(handle, value);
        }

        public bool SetMass(float value)
        {
            mass = Mathf.Max(0f, value);
            autoMass = false;
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetBodyMass(handle, MaphyUnityConvert.ToFix(mass));
        }

        public bool SetInertia(Vector3 value)
        {
            inertia = new Vector3(
                Mathf.Max(0f, value.x),
                Mathf.Max(0f, value.y),
                Mathf.Max(0f, value.z));
            autoInertia = false;
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetBodyInertia(handle, MaphyUnityConvert.ToFix3(inertia));
        }

        public bool SetAutoMassProperties(bool autoMassValue, bool autoInertiaValue)
        {
            autoMass = autoMassValue;
            autoInertia = autoInertiaValue;
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SetBodyAutoMassProperties(handle, autoMass, autoInertia);
        }

        public bool Wake()
        {
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.WakeBody(handle);
        }

        public bool Sleep()
        {
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            return physicsWorld != null && handle.IsValid && physicsWorld.SleepBody(handle);
        }

        public bool TryGetState(out PhysicsWorldBody bodyState)
        {
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            if (physicsWorld == null || !handle.IsValid)
            {
                bodyState = default;
                return false;
            }

            return physicsWorld.TryGetBody(handle, out bodyState);
        }

        private bool ApplyBodySettings(bool applyInitialVelocity)
        {
            PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
            if (physicsWorld == null || !handle.IsValid)
            {
                return false;
            }

            physicsWorld.SetBodyType(handle, bodyType);
            physicsWorld.SetBodyGravity(handle, useGravity);
            physicsWorld.SetBodyAllowSleep(handle, allowSleep);
            physicsWorld.SetBodyCCD(handle, useCCD);
            physicsWorld.SetBodyAutoMassProperties(handle, autoMass, autoInertia);
            if (!autoMass)
            {
                physicsWorld.SetBodyMass(handle, MaphyUnityConvert.ToFix(mass));
            }

            if (!autoInertia)
            {
                physicsWorld.SetBodyInertia(handle, MaphyUnityConvert.ToFix3(inertia));
            }

            if (applyInitialVelocity)
            {
                physicsWorld.SetBodyVelocity(handle, MaphyUnityConvert.ToFix3(initialVelocity));
                physicsWorld.SetBodyAngularVelocity(handle, MaphyUnityConvert.ToFix3(initialAngularVelocity));
            }

            return true;
        }

        public void PushTransformBeforeStep()
        {
            if (transformSync == MaphyUnityTransformSyncMode.Kinematic)
            {
                PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
                if (physicsWorld != null && handle.IsValid)
                {
                    physicsWorld.PushTransform(handle, transform);
                }
            }
        }

        public void PullTransformAfterStep()
        {
            if (transformSync == MaphyUnityTransformSyncMode.Dynamic)
            {
                PhysicsWorld physicsWorld = WorldComponent != null ? WorldComponent.World : null;
                if (physicsWorld != null && handle.IsValid)
                {
                    physicsWorld.PullTransform(handle, transform);
                }
            }
        }

        public void ReleaseFromWorld(MaphyPhysicsWorld owner)
        {
            if (!handle.IsValid)
            {
                registeredWorld = registeredWorld == owner ? null : registeredWorld;
                return;
            }

            MaphyPhysicsWorld currentWorld = registeredWorld != null ? registeredWorld : owner;
            if (currentWorld != null)
            {
                currentWorld.World.DestroyBody(handle);
            }

            handle = BodyHandle.Invalid;
            registeredWorld = null;
        }

        internal bool RegisterCollider(MaphyCollider collider)
        {
            if (collider == null || !handle.IsValid || WorldComponent == null)
            {
                return false;
            }

            return collider.EnsureCreated(this);
        }

        internal void UnregisterCollider(MaphyCollider collider)
        {
            if (collider != null)
            {
                collider.ReleaseFromBody(this);
            }
        }

        internal MaphyPhysicsWorld ResolveWorld()
        {
            if (world == null)
            {
                world = GetComponentInParent<MaphyPhysicsWorld>();
            }

            return world;
        }

        private void RegisterChildColliders()
        {
            colliderScratch.Clear();
            GetComponentsInChildren(true, colliderScratch);
            for (int i = 0; i < colliderScratch.Count; i++)
            {
                MaphyCollider collider = colliderScratch[i];
                if (collider != null && collider.ResolveBody() == this)
                {
                    RegisterCollider(collider);
                }
            }

            colliderScratch.Clear();
        }
    }
}
