using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap(OBB obb, fix3 point)
        {
            fix3 localPoint = quaternion.conjugate(obb.orientation) * (point - obb.center);
            return -obb.extents.x <= localPoint.x
                && obb.extents.x >= localPoint.x
                && -obb.extents.y <= localPoint.y
                && obb.extents.y >= localPoint.y
                && -obb.extents.z <= localPoint.z
                && obb.extents.z >= localPoint.z;
        }

        public static bool IsOverlap(OBB obb, Sphere sphere)
        {
            return IsOverlap(sphere, obb);
        }

        public static OBB FromMinMax(fix3 min, fix3 max, quaternion rotation)
        {
            return new OBB((min + max) * fix._0_5, max - min, rotation);
        }
    }
}