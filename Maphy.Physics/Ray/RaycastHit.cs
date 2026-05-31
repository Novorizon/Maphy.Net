
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct RaycastHit
    {
        public long id { get;  set; }
        public ulong colliderId { get; set; }
        public ulong rigidId { get; set; }
        public fix distance { get; set; }
        public fix3 point { get; set; }
        public fix3 normal { get; set; }
        public Bounds bounds { get; set; }
        public Collider collider { get; set; }
    }
}
