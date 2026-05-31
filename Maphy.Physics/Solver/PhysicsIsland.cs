using System.Collections.Generic;

namespace Maphy.Physics
{
    public sealed class PhysicsIsland
    {
        private readonly List<ulong> rigidIds = new List<ulong>();

        public IReadOnlyList<ulong> RigidIds => rigidIds;
        public bool sleeping { get; internal set; }

        internal void Clear()
        {
            rigidIds.Clear();
            sleeping = false;
        }

        internal void AddRigid(ulong rigidId)
        {
            rigidIds.Add(rigidId);
        }
    }
}
