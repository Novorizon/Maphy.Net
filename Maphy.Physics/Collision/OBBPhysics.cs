
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap(OBB aabb, fix3 point)
        {
            fix3 test0 = point - aabb.PointNormals[0];
            for (int i = 0; i < 3; i++)
            {
                fix3 test1 = point - aabb.PointNormals[i + 1];
                fix3 n = aabb.Normals[i];

                if (math.dot(test0, n) * math.dot(test1, n) > 0)
                    return false;
            }
            return true;
        }

        public static fix3 CollidePoint(OBB obb, Sphere sphere)
        {
            fix3 p = sphere.Center - obb.Center;
            p = obb.Orientation * p;

            fix3 v = math.max(p, -p);
            fix3 u = math.max(v - obb.BevelRadius, fix3.zero);

            fix dis = math.length(u);
            if (dis <= sphere.Radius)
            {
                return sphere.Center - sphere.Radius * math.normalize(u);
            }
            return fix3.MinValue;
        }
    }
}
