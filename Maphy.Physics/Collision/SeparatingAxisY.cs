
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool SeparatingAxisTestY(AABB a, OBB b)
        {
            if (a.Center.y - a.BevelRadius.y >= b.Center.y + b.BevelRadius.y)
                return false;
            if (a.Center.y + a.BevelRadius.y <= b.Center.y - b.BevelRadius.y)
                return false;

            fix2 points0 = ExtremeProjectPoint(AABB.Normals[0], a.Points);
            fix2 points1 = ExtremeProjectPoint(AABB.Normals[0], b.Points);
            if (!IsOverlap(points0, points1))
                return false;

            points0 = ExtremeProjectPoint(AABB.Normals[2], a.Points);
            points1 = ExtremeProjectPoint(AABB.Normals[2], b.Points);
            if (!IsOverlap(points0, points1))
                return false;

            points0 = ExtremeProjectPoint(b.Normals[0], a.Points);
            points1 = ExtremeProjectPoint(b.Normals[0], b.Points);
            if (!IsOverlap(points0, points1))
                return false;

            points0 = ExtremeProjectPoint(b.Normals[2], a.Points);
            points1 = ExtremeProjectPoint(b.Normals[2], b.Points);
            if (!IsOverlap(points0, points1))
                return false;

            return true;
        }


        /// SeparatingAxisTest  OBB OBB
        internal static bool SeparatingAxisTestY(OBB a, OBB b)
        {
            if (a.Center.y - a.BevelRadius.y >= b.Center.y + b.BevelRadius.y)
                return false;
            if (a.Center.y + a.BevelRadius.y <= b.Center.y - b.BevelRadius.y)
                return false;

            fix2 points0 = ExtremeProjectPoint(a.Normals[0], a.Points);
            fix2 points1 = ExtremeProjectPoint(a.Normals[0], b.Points);
            if (!IsOverlap(points0, points1))
                return false;

            points0 = ExtremeProjectPoint(a.Normals[2], a.Points);
            points1 = ExtremeProjectPoint(a.Normals[2], b.Points);
            if (!IsOverlap(points0, points1))
                return false;

            points0 = ExtremeProjectPoint(b.Normals[0], a.Points);
            points1 = ExtremeProjectPoint(b.Normals[0], b.Points);
            if (!IsOverlap(points0, points1))
                return false;

            points0 = ExtremeProjectPoint(b.Normals[2], a.Points);
            points1 = ExtremeProjectPoint(b.Normals[2], b.Points);
            if (!IsOverlap(points0, points1))
                return false;

            return true;
        }

        internal static bool SeparatingAxisTestY(AABB a, Capsule b)
        {

            fix up = a.Center.y + a.BevelRadius.y;
            fix down = a.Center.y - a.BevelRadius.y;

            if (down > b.Center1.y + b.Radius)
            {
                return false;
            }
            else if (down > b.Center1.y && down <= b.Center1.y + b.Radius)
            {
                fix3 p = b.Center1 - a.Center;

                fix3 v = math.max(p, -p);
                fix3 u = math.max(v - a.BevelRadius, fix3.zero);
                return math.length(u) < b.Radius;

            }
            else if (up < b.Center2.y - b.Radius)
            {
                return false;
            }
            else if (up > b.Center2.y - b.Radius && up <= b.Center2.y)
            {
                fix3 p = b.Center2 - a.Center;

                fix3 v = math.max(p, -p);
                fix3 u = math.max(v - a.BevelRadius, fix3.zero);
                return math.length(u) < b.Radius;

            }
            else
            {
                fix2 CenterA = new fix2(a.Center.x, a.Center.z);
                fix2 CenterB = new fix2(b.Center.x, b.Center.z);
                fix2 BevelRadius = new fix2(a.BevelRadius.x, a.BevelRadius.z);
                fix2 p = CenterB - CenterA;

                fix2 v = math.max(p, -p);
                fix2 u = math.max(v - BevelRadius, fix2.zero);
                return math.length(u) < b.Radius;

            }
        }

        internal static bool SeparatingAxisTestY(OBB a, Capsule b)
        {

            fix up = a.Center.y + a.BevelRadius.y;
            fix down = a.Center.y - a.BevelRadius.y;

            if (down > b.Center1.y + b.Radius)
            {
                return false;
            }
            else if (down > b.Center1.y && down <= b.Center1.y + b.Radius)
            {
                fix3 p = b.Center1 - a.Center;

                fix3 v = math.max(p, -p);
                fix3 u = math.max(v - a.BevelRadius, fix3.zero);
                return math.length(u) < b.Radius;

            }
            else if (up < b.Center2.y - b.Radius)
            {
                return false;
            }
            else if (up > b.Center2.y - b.Radius && up <= b.Center2.y)
            {
                fix3 p = b.Center2 - a.Center;

                fix3 v = math.max(p, -p);
                fix3 u = math.max(v - a.BevelRadius, fix3.zero);
                return math.length(u) < b.Radius;

            }
            else
            {
                fix2 CenterA = new fix2(a.Center.x, a.Center.z);
                fix2 CenterB = new fix2(b.Center.x, b.Center.z);
                fix2 BevelRadius = new fix2(a.BevelRadius.x, a.BevelRadius.z);
                fix2 p = CenterB - CenterA;

                fix2 v = math.max(p, -p);
                fix2 u = math.max(v - BevelRadius, fix2.zero);
                return math.length(u) < b.Radius;

            }
        }

        internal static bool SeparatingAxisTestY(Capsule capsule, Sphere sphere)
        {
            fix topS = sphere.Center.y + sphere.Radius;
            fix bottomS = sphere.Center.y - sphere.Radius;
            fix topC = capsule.Center1.y + capsule.Radius;
            fix bottomC = capsule.Center2.y - capsule.Radius;
            if (topS < bottomC || bottomS > topC)
            {
                return false;
            }
            else if (bottomS < topC && bottomS > capsule.Center1.y)
            {
                return math.distancesq(sphere.Center, capsule.Center1) <= (capsule.Radius2 + sphere.Radius) * (capsule.Radius2 + sphere.Radius);

            }
            else if (topS < capsule.Center2.y && topS > capsule.Center1.y)
            {
                return math.distancesq(sphere.Center, capsule.Center2) <= (capsule.Radius2 + sphere.Radius) * (capsule.Radius2 + sphere.Radius);

            }
            else
            {
                fix2 CenterA = new fix2(sphere.Center.x, sphere.Center.z);
                fix2 CenterB = new fix2(capsule.Center.x, capsule.Center.z);
                return math.distancesq(CenterA, CenterB) <= (sphere.Radius + capsule.Radius) * (sphere.Radius + capsule.Radius);
            }
        }

    }
}
