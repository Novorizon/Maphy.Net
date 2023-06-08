using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class Geometry
    {
        //public static bool IsOverlap(Polygon poly1, Polygon poly2) { return SeparatingAxisTest(poly1, poly2); }
        //public static bool IsOverlap(Triangle2D poly1, Triangle2D poly2) { return SeparatingAxisTest(poly1.polygon, poly2.polygon); }
        //public static bool IsOverlap(Rectangle poly1, Rectangle poly2) { return SeparatingAxisTest(poly1.polygon, poly2.polygon); }
        //public static bool IsOverlap(Hexagon poly1, Hexagon poly2) { return SeparatingAxisTest(poly1.polygon, poly2.polygon); }

        //public static bool IsOverlap(Polygon poly1, Triangle2D poly2) { return SeparatingAxisTest(poly1, poly2.polygon); }
        //public static bool IsOverlap(Triangle2D poly1, Polygon poly2) { return SeparatingAxisTest(poly1.polygon, poly2); }

        //public static bool IsOverlap(Polygon poly1, Rectangle poly2) { return SeparatingAxisTest(poly1, poly2.polygon); }
        //public static bool IsOverlap(Rectangle poly1, Polygon poly2) { return SeparatingAxisTest(poly1.polygon, poly2); }

        //public static bool IsOverlap(Polygon poly1, Hexagon poly2) { return SeparatingAxisTest(poly1, poly2.polygon); }
        //public static bool IsOverlap(Hexagon poly1, Polygon poly2) { return SeparatingAxisTest(poly1.polygon, poly2); }

        //public static bool IsOverlap(Polygon poly, Circular circular) { return SeparatingAxisTest(poly, circular); }
        //public static bool IsOverlap(Circular circular, Polygon poly) { return SeparatingAxisTest(poly, circular); }

        //public static bool IsOverlap(Triangle2D poly1, Rectangle poly2) { return SeparatingAxisTest(poly1.polygon, poly2.polygon); }
        //public static bool IsOverlap(Rectangle poly1, Triangle2D poly2) { return SeparatingAxisTest(poly1.polygon, poly2.polygon); }

        //public static bool IsOverlap(Triangle2D poly1, Hexagon poly2) { return SeparatingAxisTest(poly1.polygon, poly2.polygon); }
        //public static bool IsOverlap(Hexagon poly1, Triangle2D poly2) { return SeparatingAxisTest(poly1.polygon, poly2.polygon); }

        //public static bool IsOverlap(Triangle2D poly, Circular circular) { return SeparatingAxisTest(poly.polygon, circular); }
        //public static bool IsOverlap(Circular circular, Triangle2D poly) { return SeparatingAxisTest(poly.polygon, circular); }

        //public static bool IsOverlap(Rectangle poly1, Hexagon poly2) { return SeparatingAxisTest(poly1.polygon, poly2.polygon); }
        //public static bool IsOverlap(Hexagon poly1, Rectangle poly2) { return SeparatingAxisTest(poly1.polygon, poly2.polygon); }

        //public static bool IsOverlap(Rectangle poly, Circular circular) { return SeparatingAxisTest(poly.polygon, circular); }
        //public static bool IsOverlap(Circular circular, Rectangle poly) { return SeparatingAxisTest(poly.polygon, circular); }

        //public static bool IsOverlap(Hexagon poly, Circular circular) { return SeparatingAxisTest(poly.polygon, circular); }
        //public static bool IsOverlap(Circular circular, Hexagon poly) { return SeparatingAxisTest(poly.polygon, circular); }


        public static bool IsOverlap(Circular c1, Circular c2)
        {
            fix dis = math.distancesq(c1.center, c2.center);
            if (dis <= (c1.radius + c2.radius) * (c1.radius + c2.radius))
            {
                return true;
            }
            return false;
        }

        //点到a,b表示的直线的距离
        public static fix PointToLineDistance(fix2 point, fix2 a, fix2 b)
        {
            fix2 m = a - point;
            fix2 n = a - b;
            return PointToLineDistance(new fix3(point.x, point.y, fix._0), new fix3(m.x, m.y, fix._0), new fix3(n.x, n.y, fix._0));
        }



        /// 二次贝塞尔
        public static fix2 Bezier2(fix2 p0, fix2 p1, fix2 p2, fix t)
        {
            return (1 - t) * ((1 - t) * p0 + t * p1) + t * ((1 - t) * p1 + t * p2);
        }

        /// 三次贝塞尔
        public static fix2 Bezier3(fix2 p0, fix2 p1, fix2 p2, fix2 p3, fix t)
        {
            return (1 - t) * ((1 - t) * ((1 - t) * p0 + t * p1) + t * ((1 - t) * p1 + t * p2)) + t * ((1 - t) * ((1 - t) * p1 + t * p2) + t * ((1 - t) * p2 + t * p3));
        }
    }
}
