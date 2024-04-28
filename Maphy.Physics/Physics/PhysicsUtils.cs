
using Maphy.Mathematics;
using System;
using UnityEngine.Profiling;
namespace Maphy.Physics
{
    public static partial class Physics
    {

        // Compute and return a point on segment from "segPointA" and "segPointB" that is closest to point "pointC"
        public static fix3 GetClosestPointOnSegment(fix3 segPointA, fix3 segPointB, fix3 pointC)
        {

            fix3 ab = segPointB - segPointA;

            decimal abLengthSquare = math.lengthsq(ab);

            // If the segment has almost zero length
            if (abLengthSquare < math.Epsilon)
            {
                // Return one end-point of the segment as the closest point
                return segPointA;
            }

            // Project point C onto "AB" line
            fix t = math.dot((pointC - segPointA), (ab)) / abLengthSquare;

            // If projected point onto the line is outside the segment, clamp it to the segment
            if (t < fix._0)
                t = fix._0;
            if (t > fix.One) t = fix.One;

            // Return the closest point on the segment
            return segPointA + t * ab;
        }

        // Compute the penetration depth between a face of the polyhedron and a sphere along the polyhedron face normal direction
        public static fix computePolyhedronFaceVsSpherePenetrationDepth(Sphere sphere, fix3 faceNormal, fix3 point)
        {
            fix3 sphereCenterToFacePoint = point - sphere.Center;
            //球心到面上一点的向量，在面法线上的投影（肯定<0)。加上球半径，结果小于0，说明球离得远，距离>球半径
            fix penetrationDepth = math.dot(sphereCenterToFacePoint, faceNormal) + sphere.Radius;

            return penetrationDepth;
        }


        public static Face GetFaceClosestToPointOnAABB(AABB a, ref fix3 point)
        {
            Face face = Face.None;
            if (point.x < a.min.x)
            {
                point.x = a.min.x;
                face |= Face.Left;
            }
            else if (point.x > a.max.x)
            {
                point.x = a.max.x;
                face |= Face.Right;
            }

            if (point.y < a.min.y)
            {
                point.y = a.min.y;
                face |= Face.Bottom;
            }
            else if (point.y > a.max.y)
            {
                point.y = a.max.y;
                face |= Face.Top;
            }

            if (point.z < a.min.z)
            {
                point.z = a.min.z;
                face |= Face.Front;
            }
            else if (point.z > a.max.z)
            {
                point = a.max.z;
                face |= Face.Back;
            }

            return face;
        }

        public static fix3 GetFaceNormalOnAABB(AABB aabb, Face face)
        {
            fix3 normal = fix3.zero;
            switch (face)
            {
                case Face.Left:
                    normal = fix3.left;
                    break;
                case Face.Right:
                    normal = fix3.right;
                    break;
                case Face.Top:
                    normal = fix3.up;
                    break;
                case Face.Bottom:
                    normal = fix3.down;
                    break;
                case Face.Front:
                    normal = fix3.forward;
                    break;
                case Face.Back:
                    normal = fix3.backward;
                    break;

                default:
                    break;
            }
            return normal;
        }

        public static ValueTuple<fix3, fix3> GetPointAndNormalByFaceOnAABB(AABB aabb, Face face)
        {
            fix3 testPoint = fix3.MinValue;
            fix3 normal = fix3.zero;
            switch (face)
            {
                case Face.Left:
                    testPoint = aabb.min;
                    normal = fix3.left;
                    break;
                case Face.Right:
                    testPoint = aabb.max;
                    normal = fix3.right;
                    break;
                case Face.Top:
                    testPoint = aabb.max;
                    normal = fix3.up;
                    break;
                case Face.Bottom:
                    testPoint = aabb.min;
                    normal = fix3.down;
                    break;
                case Face.Front:
                    testPoint = aabb.max;
                    normal = fix3.forward;
                    break;
                case Face.Back:
                    testPoint = aabb.min;
                    normal = fix3.backward;
                    break;

                default:
                    break;
            }
            ValueTuple<fix3, fix3> info = new(testPoint, normal);
            return info;
        }

        //一般凸多边形的支持点
        //多边形的支撑点是沿着给定方向最远的顶点。如果两个顶点在给定的方向上有相等的距离，选哪个都可以。
        public static fix2 GetSupportPoint(Polygon polygon, fix2 direction)
        {
            fix bestProjection = -fix.Max;
            fix2 bestVertex=fix2.zero;//TODO

            int count = polygon.points.Length;
            for (int i = 0; i < count; ++i)
            {
                fix2 v = polygon.points[i];
                fix projection = math.dot(v, direction);//用点积来找出沿给定方向的有符号距离

                if (projection > bestProjection)
                {
                    bestVertex = v;
                    bestProjection = projection;
                }
            }

            return bestVertex;
        }

        // Return a local support point in a given direction without the object margin
        public static fix3 GetSupportPoint(AABB aabb, fix3 direction)
        {
            fix x = direction.x < fix.Zero ? aabb.min.x : aabb.max.x;
            fix y = direction.y < fix.Zero ? aabb.min.y : aabb.max.y;
            fix z = direction.z < fix.Zero ? aabb.min.z : aabb.max.z;
            //fix x = direction.x < fix.Zero ? -aabb.extents.x : aabb.extents.x;
            //fix y = direction.y < fix.Zero ? -aabb.extents.y : aabb.extents.y;
            //fix z = direction.z < fix.Zero ? -aabb.extents.z : aabb.extents.z;
            fix3 point = new fix3(x, y, z);
            return point;
        }


        // Return a local support point in a given direction without the object margin
        public static fix3 GetSupportPoint(OBB aabb, fix3 direction)
        {
            fix x = direction.x < fix.Zero ? aabb.min.x : aabb.max.x;
            fix y = direction.y < fix.Zero ? aabb.min.y : aabb.max.y;
            fix z = direction.z < fix.Zero ? aabb.min.z : aabb.max.z;
            //fix x = direction.x < fix.Zero ? -aabb.extents.x : aabb.extents.x;
            //fix y = direction.y < fix.Zero ? -aabb.extents.y : aabb.extents.y;
            //fix z = direction.z < fix.Zero ? -aabb.extents.z : aabb.extents.z;
            fix3 point = new fix3(x, y, z);
            return point;
        }
    }
}
