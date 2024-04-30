
using Maphy.Mathematics;

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
    }
}
