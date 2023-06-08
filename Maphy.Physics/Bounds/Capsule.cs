
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct Capsule : Bounds
    {
        public fix3 Center { get; private set; }
        public fix Radius { get; private set; }
        public fix Height { get; private set; }
        public quaternion Orientation { get; private set; }
        public fix3 Axis { get; private set; }

        public fix Radius2 { get; private set; }
        public fix3 Center1 { get; private set; }
        public fix3 Center2 { get; private set; }

        fix3 axisOrigin;
        public AABB Bounds { get; private set; }

        public Capsule(fix3 center, fix radius, fix height, quaternion rotation, fix3 axisOrigin)
        {
            Center = center;
            Radius = radius;
            Height = height;
            Orientation = rotation;
            Axis = rotation * axisOrigin;
            this.axisOrigin = axisOrigin;

            Radius2 = radius * radius;
            Center1 = center + math.normalize(Axis) * (height / 2 - radius);
            Center2 = center - math.normalize(Axis) * (height / 2 - radius);
            Bounds = new AABB();
        }

        public void Update(fix3 center, quaternion rotation = default)
        {
            Center = center;
            Orientation = rotation;
            Axis = math.normalize(rotation * axisOrigin);
            Center1 = center + Axis * (Height / 2 - Radius);
            Center2 = center - Axis * (Height / 2 - Radius);
        }
    }
}

