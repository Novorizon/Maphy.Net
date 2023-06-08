using System;

namespace Maphy.Mathematics
{
    public enum GeometryType
    {
        Polygon = 1,
        Triangle = 2,
        Rectangle = 3,
        Hexagon = 4,
        Circular = 5
    }

    public static partial class Geometry
    {
        //点到线段的距离
        public static fix PointToSegmentDistance(fix3 point, Segment a)
        {
            return PointToSegmentDistance(point, a.start, a.end);
        }

        //点到直线的距离
        public static fix PointToLineDistance(fix3 point, Line a)
        {
            return PointToSegmentDistance(point, a.point, a.point + a.direction);
        }

        //点到线段的距离
        public static fix PointToPlaneDistance(fix3 point, Plane a)
        {
            return PointToPlaneDistance(point, a.point, a.normal);
        }


        /// 二次贝塞尔
        public static fix3 Bezier2(fix3 p0, fix3 p1, fix3 p2, fix t)
        {
            return (1 - t) * ((1 - t) * p0 + t * p1) + t * ((1 - t) * p1 + t * p2);
        }

        /// 三次贝塞尔
        public static fix3 Bezier3(fix3 p0, fix3 p1, fix3 p2, fix3 p3, fix t)
        {
            return (1 - t) * ((1 - t) * ((1 - t) * p0 + t * p1) + t * ((1 - t) * p1 + t * p2)) + t * ((1 - t) * ((1 - t) * p1 + t * p2) + t * ((1 - t) * p2 + t * p3));
        }


        public static bool IsConvex(fix2[] points)
        {
            int Length = points.Length;
            for (int i = 0; i < Length; i++)
            {
                if (math.cross(points[i], points[(i + 1) % Length]) * math.cross(points[(i + 1) % Length], points[(i + 2) % Length]) <= 0)
                    return false;
            }
            return true;
        }


        public static bool IsConvex(fix3[] points)
        {
            int Length = points.Length;
            for (int i = 0; i < Length; i++)
            {
                if (math.cross(points[i], points[(i + 1) % Length]).y * math.cross(points[(i + 1) % Length], points[(i + 2) % Length]).y <= 0)
                    return false;
            }
            return true;
        }
    }

}

