
using Maphy.Mathematics;
using Maphy.Tree;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public enum ShapeType
    {
        None = 0,
        AABB = 1,
        OBB = 2,
        Sphere = 3,
        Capsule = 4,
    }

    public interface Shape
    {
        public ShapeType Type { get; }
    }
}
