using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        public static fix3 GetClosestPointOnSegment(fix3 segPointA, fix3 segPointB, fix3 pointC)
        {
            fix3 ab = segPointB - segPointA;
            fix abLengthSquare = math.lengthsq(ab);

            if (abLengthSquare < math.Epsilon)
            {
                return segPointA;
            }

            fix t = math.dot(pointC - segPointA, ab) / abLengthSquare;
            t = math.clamp(t, fix._0, fix._1);
            return segPointA + t * ab;
        }

        public static fix computePolyhedronFaceVsSpherePenetrationDepth(Sphere sphere, fix3 faceNormal, fix3 point)
        {
            fix3 sphereCenterToFacePoint = point - sphere.Center;
            return math.dot(sphereCenterToFacePoint, faceNormal) + sphere.Radius;
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
                point.z = a.max.z;
                face |= Face.Back;
            }

            return face;
        }

        public static fix3 GetFaceNormalOnAABB(AABB aabb, Face face)
        {
            switch (face)
            {
                case Face.Left:
                    return fix3.left;
                case Face.Right:
                    return fix3.right;
                case Face.Top:
                    return fix3.up;
                case Face.Bottom:
                    return fix3.down;
                case Face.Front:
                    return fix3.forward;
                case Face.Back:
                    return fix3.backward;
                default:
                    return fix3.zero;
            }
        }

        public static ValueTuple<fix3, fix3> GetPointAndNormalByFaceOnAABB(AABB aabb, Face face)
        {
            switch (face)
            {
                case Face.Left:
                    return new ValueTuple<fix3, fix3>(aabb.min, fix3.left);
                case Face.Right:
                    return new ValueTuple<fix3, fix3>(aabb.max, fix3.right);
                case Face.Top:
                    return new ValueTuple<fix3, fix3>(aabb.max, fix3.up);
                case Face.Bottom:
                    return new ValueTuple<fix3, fix3>(aabb.min, fix3.down);
                case Face.Front:
                    return new ValueTuple<fix3, fix3>(aabb.max, fix3.forward);
                case Face.Back:
                    return new ValueTuple<fix3, fix3>(aabb.min, fix3.backward);
                default:
                    return new ValueTuple<fix3, fix3>(fix3.MinValue, fix3.zero);
            }
        }

        public static fix2 GetSupportPoint(Polygon polygon, fix2 direction)
        {
            fix bestProjection = -fix.Max;
            fix2 bestVertex = fix2.zero;

            for (int i = 0; i < polygon.points.Length; ++i)
            {
                fix2 vertex = polygon.points[i];
                fix projection = math.dot(vertex, direction);
                if (projection > bestProjection)
                {
                    bestVertex = vertex;
                    bestProjection = projection;
                }
            }

            return bestVertex;
        }

        public static fix3 GetSupportPoint(AABB aabb, fix3 direction)
        {
            fix x = direction.x < fix.Zero ? aabb.min.x : aabb.max.x;
            fix y = direction.y < fix.Zero ? aabb.min.y : aabb.max.y;
            fix z = direction.z < fix.Zero ? aabb.min.z : aabb.max.z;
            return new fix3(x, y, z);
        }

        public static fix3 GetSupportPoint(OBB obb, fix3 direction)
        {
            AABB bounds = ComputeBounds(obb);
            return GetSupportPoint(bounds, direction);
        }
    }
}