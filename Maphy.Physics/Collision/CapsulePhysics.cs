
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        //任意轴向
        internal static bool IsOverlap(Capsule capsule, fix3 point)
        {
            //判断与中心点1的距离
            if (math.distancesq(point, capsule.Center1) <= capsule.Radius2)
                return true;
            //判断与中心点2的距离
            if (math.distancesq(point, capsule.Center2) <= capsule.Radius2)
                return true;

            //如果point在向量(center1 - center2）上的投影点位于center1和center2之间，并且point到轴的距离小于等于Radius
            var p = math.dot(capsule.Center1 - capsule.Center2, point);
            fix2 extremePoints = ExtremeProjectPoint(capsule.Center1 - capsule.Center2, new fix3[] { capsule.Center1, capsule.Center2 });
            if (extremePoints.x <= p && extremePoints.y >= p)
            {
                if (Geometry.PointToLineDistance(point, capsule.Center1, capsule.Center2) <= capsule.Radius * capsule.Radius)
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
        internal static bool IsOverlapParallel(Capsule a, Capsule b)
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
