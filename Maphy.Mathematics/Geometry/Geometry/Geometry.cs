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

        // Compute and return a point on segment from "segPointA" and "segPointB" that is closest to point "pointC"
        /// <summary>
        /// 在线段上找到一个点，使其距离给定点最近
        /// </summary>
        /// <param name="segPointA"></param>
        /// <param name="segPointB"></param>
        /// <param name="pointC"></param>
        /// <returns></returns>
        public static fix3 GetClosestPointOnSegmentToPoint(fix3 segPointA, fix3 segPointB, fix3 pointC)
        {

            fix3 ab = segPointB - segPointA;

            decimal abLengthSquare = math.lengthsq(ab);

            // If the segment has almost zero length
            if (abLengthSquare <math.Epsilon)
            {
                // Return one end-point of the segment as the closest point
                return segPointA;
            }

            // Project point C onto "AB" line
            decimal t = math.dot(pointC - segPointA, ab) / abLengthSquare;

            // If projected point onto the line is outside the segment, clamp it to the segment
            if (t < fix._0)
                t = fix._0;
            if (t > fix._1)
                t = fix._1;

            // Return the closest point on the segment
            return segPointA + t * ab;
        }



        // Compute the closest points between two segments
        // This method uses the technique described in the book Real-Time
        // collision detection by Christer Ericson.
        public static void GetClosestPointBetweenSegments(fix3 seg1PointA, fix3 seg1PointB, fix3 seg2PointA, fix3 seg2PointB, out fix3 closestPointSeg1, out fix3 closestPointSeg2)
        {

            fix3 d1 = seg1PointB - seg1PointA;
            fix3 d2 = seg2PointB - seg2PointA;
            fix3 r = seg1PointA - seg2PointA;
            decimal a = math.lengthsq(d1);
            decimal e = math.lengthsq(d2);
            decimal f = math.dot(d2, r);
            decimal s, t;

            // If both segments degenerate into points
            if (a <=math.Epsilon && e <=math.Epsilon)
            {

                closestPointSeg1 = seg1PointA;
                closestPointSeg2 = seg2PointA;
                return;
            }
            if (a <=math.Epsilon)
            {   // If first segment degenerates into a point

                s = fix._0;

                // Compute the closest point on second segment
                t = math.clamp(f / e, fix._0, fix._1);
            }
            else
            {

                fix c = math.dot(d1, r);

                // If the second segment degenerates into a point
                if (e <=math.Epsilon)
                {

                    t = fix._0;
                    s = math.clamp(-c / a, fix._0, fix._1);
                }
                else
                {

                    fix b = math.dot(d1, d2);
                    fix denom = a * e - b * b;

                    // If the segments are not parallel
                    if (denom != fix._0)
                    {

                        // Compute the closest point on line 1 to line 2 and
                        // clamp to first segment.
                        s = math.clamp((b * f - c * e) / denom, fix._0, fix._1);
                    }
                    else
                    {

                        // Pick an arbitrary point on first segment
                        s = fix._0;
                    }

                    // Compute the point on line 2 closest to the closest point
                    // we have just found
                    t = (b * s + f) / e;

                    // If this closest point is inside second segment (t in [0, 1]), we are done.
                    // Otherwise, we clamp the point to the second segment and compute again the
                    // closest point on segment 1
                    if (t < fix._0)
                    {
                        t = fix._0;
                        s = math.clamp(-c / a, fix._0, fix._1);
                    }
                    else if (t > fix._1)
                    {
                        t = fix._1;
                        s = math.clamp((b - c) / a, fix._0, fix._1);
                    }
                }
            }

            // Compute the closest points on both segments
            closestPointSeg1 = seg1PointA + d1 * s;
            closestPointSeg2 = seg2PointA + d2 * t;
        }
    }
}

