
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct RaycastHit
    {
        public long id { get;  set; }
        public fix3 point { get; set; }
        public Bounds bounds { get; set; }
        //public fix distance { get; set; }
    }
}