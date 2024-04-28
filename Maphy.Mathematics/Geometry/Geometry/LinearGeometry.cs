using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using Maphy.Physics;

namespace Maphy.Mathematics
{
    public enum SpatialRelationship
    {
        OVERLAPPING = 0,//      if the line is on the plane where the triangle lies;
        PARALLEL = 1,//         if the line is parallel to the triangle.
        DISJOINT = 2,//      if the line misses the triangle;
        INTERSECTING = 3,//      if the line intersects with the triangle.
    }

    public static partial class Geometry
    {
        //"Fast, minimum storage ray-triangle intersection." Journal of graphics tools, vol.2, no.1, pp.21-28, 1997
        static SpatialRelationship linearIntersectTriangle(fix3[] triangle, fix3 point, fix3 direction, out fix3 hit)
        {
            hit = fix3.MinValue;
            fix3 s = point - triangle[0];
            fix3 e1 = triangle[1] - triangle[0];
            fix3 e2 = triangle[2] - triangle[0];
            fix3 p = math.cross(direction, e2);
            fix tmp = math.dot(p, e1);
            //If the line is perpendicular to the normal of triangle.
            if (tmp == fix.Zero)
            {

                p = math.cross(e1, e2);  //The line is on the plane.
                if (math.dot(p, s) == 0)
                    return SpatialRelationship.OVERLAPPING;

                return SpatialRelationship.PARALLEL;         //The line is parallel to the plane.
            }
            fix u = math.dot(p, s) / tmp;
            if (u < 0 || u > 1)
                return SpatialRelationship.DISJOINT;
            fix3 q = math.cross(s, e1);
            fix v = math.dot(q, direction) / tmp;
            if (v < 0 || v > 1 || u + v > 1)
                return SpatialRelationship.DISJOINT;

            hit.x = u;
            hit.y = v;
            hit.z = math.dot(e2, q) / tmp;
            return SpatialRelationship.INTERSECTING;
        }

        //直线与三角形相交
        public static SpatialRelationship LineHitTriangle(Triangle triangle, Line line, out fix3 hit)
        {
            hit = fix3.MinValue;
            fix3[] points = new fix3[3] { triangle.points[0], triangle.points[1], triangle.points[2] };
            SpatialRelationship retFlag = linearIntersectTriangle(points, line.point, line.direction, out fix3 ret);
            if (retFlag != SpatialRelationship.INTERSECTING)
                return retFlag;

            hit = line.point + ret.z * line.direction;
            //check whether the intersection point is on the edge.
            //if( EQUAL(ret.x+ret.y-1.0,0) || 
            //  EQUAL(ret.x,0) || 
            //  EQUAL(ret.y,0) )
            //  return SR_INTERSECTING_EDGE;
            return SpatialRelationship.INTERSECTING;
        }

        //直线与三角形相交
        public static SpatialRelationship LineHitTriangle(fix3[] triangle, fix3[] line, out fix3 hit)
        {
            hit = fix3.MinValue;
            SpatialRelationship retFlag = linearIntersectTriangle(triangle, line[0], line[1], out fix3 ret);
            if (retFlag != SpatialRelationship.INTERSECTING)
                return retFlag;

            hit = line[0] + ret.z * line[1];
            return SpatialRelationship.INTERSECTING;
        }


        //射线与三角形相交
        public static SpatialRelationship RayHitTriangle(Triangle triangle, Ray ray, fix3 result)
        {
            fix3[] points = new fix3[3] { triangle.points[0], triangle.points[1], triangle.points[2] };
            SpatialRelationship retFlag = linearIntersectTriangle(points, ray.origin, ray.direction, out fix3 ret);
            if (retFlag != SpatialRelationship.INTERSECTING)
                return retFlag;

            //check t. t>=0
            if (ret.z < 0)
                return SpatialRelationship.DISJOINT;
            result = ray.origin + ret.z * ray.direction;
            //check whether the intersection point is on the edge.
            //if( EQUAL(ret.x+ret.y-1.0,0) || 
            //  EQUAL(ret.x,0) || 
            //  EQUAL(ret.y,0) )
            //  return SR_INTERSECTING_EDGE;
            return SpatialRelationship.INTERSECTING;
        }

        //射线与三角形相交
        public static SpatialRelationship RayHitTriangle(fix3[] triangle, fix3[] ray, fix3 result)
        {
            SpatialRelationship retFlag = linearIntersectTriangle(triangle, ray[0], ray[1], out fix3 ret);
            if (retFlag != SpatialRelationship.INTERSECTING)
                return retFlag;

            if (ret.z < 0)
                return SpatialRelationship.DISJOINT;
            result = ray[0] + ret.z * ray[1];
            return SpatialRelationship.INTERSECTING;
        }


        //线段与三角形相交
        public static SpatialRelationship SegmentHitTriangle(Triangle triangle, Segment segment, fix3 result)
        {
            fix3[] points = new fix3[3] { triangle.points[0], triangle.points[1], triangle.points[2] };
            fix3 direction = segment.start - segment.end;
            SpatialRelationship retFlag = linearIntersectTriangle(points, segment.start, direction, out fix3 ret);
            if (retFlag != SpatialRelationship.INTERSECTING)
                return retFlag;

            //check t. 0<= t <=1
            if (ret.z < 0 || ret.z > 1)
                return SpatialRelationship.DISJOINT;
            result = segment.start + ret.z * direction;
            //check whether the intersection point is on the edge.
            //if( EQUAL(ret.x+ret.y-1.0,0) || 
            //  EQUAL(ret.x,0) || 
            //  EQUAL(ret.y,0) )
            //  return SR_INTERSECTING_EDGE;
            return SpatialRelationship.INTERSECTING;
        }

        //线段与三角形相交
        public static SpatialRelationship SegmentHitTriangle(fix3[] triangle, fix3[] segment, fix3 result)
        {
            fix3 direction = segment[1] - segment[0];
            SpatialRelationship retFlag = linearIntersectTriangle(triangle, segment[0], direction, out fix3 ret);
            if (retFlag != SpatialRelationship.INTERSECTING)
                return retFlag;

            if (ret.z < 0 || ret.z > 1)
                return SpatialRelationship.DISJOINT;
            result = segment[0] + ret.z * direction;
            return SpatialRelationship.INTERSECTING;
        }

        public static bool Hit(AABB aabb, Ray ray, float tmin, float tmax)
        {
            for (var i = 0; i < 3; i++)
            {
                var t0 = math.min((aabb.min[i] - ray.origin[i]) / ray.direction[i],
                    (aabb.max[i] - ray.origin[i]) / ray.direction[i]);

                var t1 = math.max((aabb.min[i] - ray.origin[i]) / ray.direction[i],
                    (aabb.max[i] - ray.origin[i]) / ray.direction[i]);

                tmin = math.max(t0, tmin);
                tmax = math.min(t1, tmax);
                if (tmax <= tmin)
                    return false;
            }
            return true;
        }

    }
}
