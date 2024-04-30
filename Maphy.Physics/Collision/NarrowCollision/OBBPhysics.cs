
using Codice.Client.BaseCommands;
using Maphy.Mathematics;
using UnityEngine.UIElements;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap(OBB obb, fix3 point)
        {
            point = point - obb.center;
            point = obb.orientation * point;

            if (obb.min.x > point.x || obb.min.y > point.y || obb.min.z > point.z || obb.max.x < point.x || obb.max.y < point.y || obb.max.z < point.z)
                return false;
            return true;

            //fix3 test0 = point - obb.PointNormals[0];
            //for (int i = 0; i < 3; i++)
            //{
            //    fix3 test1 = point - obb.PointNormals[i + 1];
            //    fix3 n = obb.Normals[i];

            //    if (math.dot(test0, n) * math.dot(test1, n) > 0)
            //        return false;
            //}
            //return true;
        }

        public static bool IsOverlap(OBB obb, Sphere sphere)
        {
            fix3 p = sphere.Center - obb.center;
            p = obb.orientation * p;

            fix3 v = math.max(p, -p);
            fix3 u = math.max(v - obb.extents, fix3.zero);

            fix dis = math.length(u);
            if (dis <= sphere.Radius)
            {
                return true;
            }
            return false;
        }


        public static OBB FromMinMax(fix3 min, fix3 max, quaternion rotation)
        {
            fix3 center = (min + max) / 2;
            fix3 size = (max - min) / 2;

            OBB obb = new OBB(center, size, rotation);
            return obb;
        }

    }
}
