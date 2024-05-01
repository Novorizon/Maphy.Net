
using Maphy.Mathematics;
using static UnityEditor.IMGUI.Controls.PrimitiveBoundsHandle;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap(Capsule capsule, fix3 point)
        {
            //capsule中轴线上 距离点最近的点
            fix3 closestPointOnSegment = GetClosestPointOnSegment(capsule.Center1, capsule.Center2, point);

            fix3 pointToSegment = closestPointOnSegment - point;
            fix pointSegmentDistanceSquare = math.lengthsq(pointToSegment);
            if (pointSegmentDistanceSquare < capsule.Radius)
            {
                return true;
            }
            return false;
        }

        internal static bool IsOverlap(Capsule a, Capsule b)
        {
            //端点距离
            fix dis = (a.Radius + b.Radius) * (a.Radius + b.Radius);
            if (math.distancesq(a.Center1, b.Center1) <= dis)
                return true;
            if (math.distancesq(a.Center1, b.Center2) <= dis)
                return true;
            if (math.distancesq(a.Center2, b.Center1) <= dis)
                return true;
            if (math.distancesq(a.Center2, b.Center2) <= dis)
                return true;

            dis = a.Radius + b.Radius;
            if (math.dot(a.Center1 - b.Center2, b.Center1 - b.Center2) > 0 && math.dot(a.Center1 - b.Center1, b.Center1 - b.Center2) < 0)
            {
                if (Geometry.PointToLineDistance(a.Center1, b.Center1, b.Center2) <= dis)
                    return true;
            }
            if (math.dot(a.Center2 - b.Center2, b.Center1 - b.Center2) > 0 && math.dot(a.Center2 - b.Center1, b.Center1 - b.Center2) < 0)
            {
                if (Geometry.PointToLineDistance(a.Center2, b.Center1, b.Center2) <= dis)
                    return true;
            }
            if (math.dot(b.Center1 - a.Center2, a.Center1 - a.Center2) > 0 && math.dot(b.Center1 - a.Center1, a.Center1 - a.Center2) < 0)
            {
                if (Geometry.PointToLineDistance(b.Center1, a.Center1, a.Center2) <= dis)
                    return true;
            }
            if (math.dot(b.Center2 - a.Center2, a.Center1 - a.Center2) > 0 && math.dot(b.Center2 - a.Center1, a.Center1 - a.Center2) < 0)
            {
                if (Geometry.PointToLineDistance(b.Center2, a.Center1, a.Center2) <= dis)
                    return true;
            }

            return false;
        }

        //https://wickedengine.net/2020/04/capsule-collision-detection/
        internal static bool Overlaps(Capsule a, Capsule b)
        {// capsule A:
            fix3 a_Normal = math.normalize(a.Center2 - a.Center1);
            fix3 a_LineEndOffset = a_Normal * a.Radius;
            fix3 a_A = a.Center1 + a_LineEndOffset;
            fix3 a_B = a.Center2 - a_LineEndOffset;

            // capsule B:
            fix3 b_Normal = math.normalize(b.Center2 - b.Center1);
            fix3 b_LineEndOffset = b_Normal * b.Radius;
            fix3 b_A = b.Center1 + b_LineEndOffset;
            fix3 b_B = b.Center2 - b_LineEndOffset;

            // vectors between line endpoints:
            fix3 v0 = b_A - a_A;
            fix3 v1 = b_B - a_A;
            fix3 v2 = b_A - a_B;
            fix3 v3 = b_B - a_B;

            // squared distances:
            fix B1A1 = math.dot(v0, v0);
            fix B2A1 = math.dot(v1, v1);
            fix B1A2 = math.dot(v2, v2);
            fix B2A2 = math.dot(v3, v3);

            // select best potential endpoint on capsule A:
            fix3 bestA;
            if (B1A2 < B1A1 || B1A2 <B2A1  || B2A2 < B1A1 || B2A2 < B2A1)
            {
                bestA = a_B;
            }
            else
            {
                bestA = a_A;
            }

            // select point on capsule B line segment nearest to best potential endpoint on A capsule:
            fix3 bestB = GetClosestPointOnSegment(b_A, b_B, bestA);

            // now do the same for capsule A segment:
            bestA = GetClosestPointOnSegment(a_A, a_B, bestB);
            // We selected the two best possible candidates on both capsule axes.What remains is to place spheres on those points and perform the sphere intersection routine:

            fix3 penetration_normal = bestA - bestB;
            float len = math.length(penetration_normal);
            penetration_normal /= len;  // normalize
            float penetration_depth = a.Radius + b.Radius - len;
            bool intersects = penetration_depth > 0;
            return false;
        }
    }
}
