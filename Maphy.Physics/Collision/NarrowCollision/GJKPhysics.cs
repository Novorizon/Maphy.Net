using System.Collections.Generic;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        private const int GJKMaxIterations = 24;
        private const int EPAMaxIterations = 64;

        public static bool GJKOverlaps(Shape shape0, Shape shape1)
        {
            return TryBuildGJKSimplex(shape0, shape1, out GJKSimplex _);
        }

        public static bool TryComputeGJKEPAContact(Shape shape0, Shape shape1, out CollisionInfo collision)
        {
            collision = default;
            if (shape0 != null
                && shape1 != null
                && shape0.Type == ShapeType.Sphere
                && shape1.Type == ShapeType.Sphere
                && GJKOverlaps(shape0, shape1))
            {
                return TryComputeGJKSphereSphereContact((Sphere)shape0, (Sphere)shape1, out collision);
            }

            if (!TryBuildGJKSimplex(shape0, shape1, out GJKSimplex simplex))
            {
                return false;
            }

            if (!TryEPA(shape0, shape1, simplex, out fix3 normal, out fix depth, out fix3 pointOnShape0, out fix3 pointOnShape1))
            {
                return false;
            }

            collision = new CollisionInfo(0, 0)
            {
                penetrationDepth = depth,
                normal = normal,
            };
            collision.AddContact(pointOnShape0, pointOnShape1, depth);
            return true;
        }

        private static bool TryComputeGJKSphereSphereContact(Sphere sphere0, Sphere sphere1, out CollisionInfo collision)
        {
            fix3 delta = sphere1.Center - sphere0.Center;
            fix distanceSq = math.lengthsq(delta);
            fix radius = sphere0.Radius + sphere1.Radius;
            if (distanceSq > radius * radius)
            {
                collision = default;
                return false;
            }

            fix distance = distanceSq > math.Epsilon ? math.sqrt(distanceSq) : fix.Zero;
            fix3 normal = distanceSq > math.Epsilon ? delta / distance : fix3.up;
            fix penetrationDepth = radius - distance;
            collision = new CollisionInfo(0, 0)
            {
                penetrationDepth = penetrationDepth,
                normal = normal,
            };
            collision.AddContact(sphere0.Center + normal * sphere0.Radius, sphere1.Center - normal * sphere1.Radius, penetrationDepth);
            return true;
        }

        private static bool TryBuildGJKSimplex(Shape shape0, Shape shape1, out GJKSimplex simplex)
        {
            simplex = default;
            if (shape0 == null || shape1 == null)
            {
                return false;
            }

            fix3 direction = GetShapeCenter(shape1) - GetShapeCenter(shape0);
            if (math.lengthsq(direction) <= math.Epsilon)
            {
                direction = fix3.right;
            }

            simplex.Add(GetMinkowskiSupport(shape0, shape1, direction));
            direction = -simplex.a.point;

            for (int i = 0; i < GJKMaxIterations; i++)
            {
                if (math.lengthsq(direction) <= math.Epsilon)
                {
                    return true;
                }

                GJKSupportPoint point = GetMinkowskiSupport(shape0, shape1, direction);
                if (math.dot(point.point, direction) < -math.Epsilon)
                {
                    return false;
                }

                simplex.Add(point);
                if (UpdateSimplex(ref simplex, ref direction))
                {
                    return simplex.count == 4;
                }
            }

            return false;
        }

        private static GJKSupportPoint GetMinkowskiSupport(Shape shape0, Shape shape1, fix3 direction)
        {
            fix3 support0 = GetSupportPointOnConvexShape(shape0, direction);
            fix3 support1 = GetSupportPointOnConvexShape(shape1, -direction);
            return new GJKSupportPoint(support0, support1);
        }

        private static fix3 GetSupportPointOnConvexShape(Shape shape, fix3 direction)
        {
            fix3 normal = NormalizeOrDefault(direction, fix3.right);
            switch (shape.Type)
            {
                case ShapeType.AABB:
                    return GetSupportPoint((AABB)shape, normal);
                case ShapeType.OBB:
                    return GetSupportPoint((OBB)shape, normal);
                case ShapeType.Sphere:
                    Sphere sphere = (Sphere)shape;
                    return sphere.Center + normal * sphere.Radius;
                case ShapeType.Capsule:
                    Capsule capsule = (Capsule)shape;
                    fix center1Dot = math.dot(capsule.Center1, normal);
                    fix center2Dot = math.dot(capsule.Center2, normal);
                    fix3 capCenter = center1Dot >= center2Dot ? capsule.Center1 : capsule.Center2;
                    return capCenter + normal * capsule.Radius;
                default:
                    return GetSupportPoint(ComputeBounds(shape), normal);
            }
        }

        private static bool TryEPA(
            Shape shape0,
            Shape shape1,
            GJKSimplex simplex,
            out fix3 normal,
            out fix depth,
            out fix3 pointOnShape0,
            out fix3 pointOnShape1)
        {
            normal = fix3.zero;
            depth = fix.Zero;
            pointOnShape0 = fix3.zero;
            pointOnShape1 = fix3.zero;

            if (simplex.count < 4)
            {
                return false;
            }

            List<EPAFace> faces = new List<EPAFace>(16);
            AddEPAFace(faces, simplex.a, simplex.b, simplex.c);
            AddEPAFace(faces, simplex.a, simplex.c, simplex.d);
            AddEPAFace(faces, simplex.a, simplex.d, simplex.b);
            AddEPAFace(faces, simplex.b, simplex.d, simplex.c);
            if (faces.Count < 4)
            {
                return false;
            }

            List<EPADirectedEdge> horizon = new List<EPADirectedEdge>(16);
            for (int iteration = 0; iteration < EPAMaxIterations; iteration++)
            {
                int bestFaceIndex = FindClosestEPAFace(faces);
                if (bestFaceIndex < 0)
                {
                    return false;
                }

                EPAFace bestFace = faces[bestFaceIndex];
                GJKSupportPoint support = GetMinkowskiSupport(shape0, shape1, bestFace.normal);
                fix supportDistance = math.dot(support.point, bestFace.normal);
                if (supportDistance - bestFace.distance <= fix._0_0001 || ContainsEPASupport(faces, support))
                {
                    normal = bestFace.normal;
                    depth = math.max(fix.Zero, bestFace.distance);
                    GetClosestFaceContact(bestFace, out pointOnShape0, out pointOnShape1);
                    RefineEPAContact(ref normal, ref depth, pointOnShape0, pointOnShape1);
                    return true;
                }

                horizon.Clear();
                for (int faceIndex = faces.Count - 1; faceIndex >= 0; faceIndex--)
                {
                    EPAFace face = faces[faceIndex];
                    if (math.dot(face.normal, support.point - face.a.point) <= fix._0_0001)
                    {
                        continue;
                    }

                    AddHorizonEdge(horizon, face.a, face.b);
                    AddHorizonEdge(horizon, face.b, face.c);
                    AddHorizonEdge(horizon, face.c, face.a);
                    faces.RemoveAt(faceIndex);
                }

                if (horizon.Count == 0)
                {
                    normal = bestFace.normal;
                    depth = math.max(fix.Zero, bestFace.distance);
                    GetClosestFaceContact(bestFace, out pointOnShape0, out pointOnShape1);
                    RefineEPAContact(ref normal, ref depth, pointOnShape0, pointOnShape1);
                    return true;
                }

                for (int edgeIndex = 0; edgeIndex < horizon.Count; edgeIndex++)
                {
                    EPADirectedEdge edge = horizon[edgeIndex];
                    AddEPAFace(faces, edge.start, edge.end, support);
                }
            }

            int fallbackFaceIndex = FindClosestEPAFace(faces);
            if (fallbackFaceIndex < 0)
            {
                return false;
            }

            EPAFace fallbackFace = faces[fallbackFaceIndex];
            normal = fallbackFace.normal;
            depth = math.max(fix.Zero, fallbackFace.distance);
            GetClosestFaceContact(fallbackFace, out pointOnShape0, out pointOnShape1);
            RefineEPAContact(ref normal, ref depth, pointOnShape0, pointOnShape1);
            return true;
        }

        private static void RefineEPAContact(ref fix3 normal, ref fix depth, fix3 pointOnShape0, fix3 pointOnShape1)
        {
            fix3 delta = pointOnShape0 - pointOnShape1;
            fix deltaLengthSq = math.lengthsq(delta);
            if (deltaLengthSq <= math.Epsilon)
            {
                return;
            }

            normal = delta / math.sqrt(deltaLengthSq);
            depth = math.max(fix.Zero, math.dot(delta, normal));
        }

        private static void AddEPAFace(List<EPAFace> faces, GJKSupportPoint a, GJKSupportPoint b, GJKSupportPoint c)
        {
            fix3 normal = math.cross(b.point - a.point, c.point - a.point);
            fix normalLengthSq = math.lengthsq(normal);
            if (normalLengthSq <= math.Epsilon)
            {
                return;
            }

            normal /= math.sqrt(normalLengthSq);
            fix distance = math.dot(normal, a.point);
            if (distance < fix.Zero)
            {
                GJKSupportPoint temp = b;
                b = c;
                c = temp;
                normal = -normal;
                distance = -distance;
            }

            faces.Add(new EPAFace(a, b, c, normal, distance));
        }

        private static int FindClosestEPAFace(List<EPAFace> faces)
        {
            int bestIndex = -1;
            fix bestDistance = fix.Max;
            for (int i = 0; i < faces.Count; i++)
            {
                if (faces[i].distance < bestDistance)
                {
                    bestDistance = faces[i].distance;
                    bestIndex = i;
                }
            }

            return bestIndex;
        }

        private static bool ContainsEPASupport(List<EPAFace> faces, GJKSupportPoint support)
        {
            for (int i = 0; i < faces.Count; i++)
            {
                EPAFace face = faces[i];
                if (SameSupportPoint(face.a, support)
                    || SameSupportPoint(face.b, support)
                    || SameSupportPoint(face.c, support))
                {
                    return true;
                }
            }

            return false;
        }

        private static bool SameSupportPoint(GJKSupportPoint a, GJKSupportPoint b)
        {
            return math.lengthsq(a.point - b.point) <= math.Epsilon;
        }

        private static void AddHorizonEdge(List<EPADirectedEdge> edges, GJKSupportPoint start, GJKSupportPoint end)
        {
            for (int i = 0; i < edges.Count; i++)
            {
                EPADirectedEdge edge = edges[i];
                if (edge.start.Equals(end) && edge.end.Equals(start))
                {
                    edges.RemoveAt(i);
                    return;
                }
            }

            edges.Add(new EPADirectedEdge(start, end));
        }

        private static void GetClosestFaceContact(EPAFace face, out fix3 pointOnShape0, out fix3 pointOnShape1)
        {
            fix3 closestPoint = face.normal * face.distance;
            GetBarycentric(closestPoint, face.a.point, face.b.point, face.c.point, out fix u, out fix v, out fix w);
            pointOnShape0 = face.a.support0 * u + face.b.support0 * v + face.c.support0 * w;
            pointOnShape1 = face.a.support1 * u + face.b.support1 * v + face.c.support1 * w;
        }

        private static void GetBarycentric(fix3 point, fix3 a, fix3 b, fix3 c, out fix u, out fix v, out fix w)
        {
            fix3 v0 = b - a;
            fix3 v1 = c - a;
            fix3 v2 = point - a;
            fix d00 = math.dot(v0, v0);
            fix d01 = math.dot(v0, v1);
            fix d11 = math.dot(v1, v1);
            fix d20 = math.dot(v2, v0);
            fix d21 = math.dot(v2, v1);
            fix denominator = d00 * d11 - d01 * d01;
            if (math.abs(denominator) <= math.Epsilon)
            {
                u = fix.One / 3;
                v = fix.One / 3;
                w = fix.One - u - v;
                return;
            }

            v = (d11 * d20 - d01 * d21) / denominator;
            w = (d00 * d21 - d01 * d20) / denominator;
            u = fix.One - v - w;
            u = math.clamp(u, fix.Zero, fix.One);
            v = math.clamp(v, fix.Zero, fix.One);
            w = math.clamp(w, fix.Zero, fix.One);
            fix sum = u + v + w;
            if (sum <= math.Epsilon)
            {
                u = fix.One / 3;
                v = fix.One / 3;
                w = fix.One - u - v;
                return;
            }

            u /= sum;
            v /= sum;
            w /= sum;
        }

        private static bool UpdateSimplex(ref GJKSimplex simplex, ref fix3 direction)
        {
            switch (simplex.count)
            {
                case 2:
                    return UpdateLine(ref simplex, ref direction);
                case 3:
                    return UpdateTriangle(ref simplex, ref direction);
                case 4:
                    return UpdateTetrahedron(ref simplex, ref direction);
                default:
                    direction = -simplex.a.point;
                    return false;
            }
        }

        private static bool UpdateLine(ref GJKSimplex simplex, ref fix3 direction)
        {
            GJKSupportPoint a = simplex.a;
            GJKSupportPoint b = simplex.b;
            fix3 ab = b.point - a.point;
            fix3 ao = -a.point;

            if (SameDirection(ab, ao))
            {
                direction = TripleCross(ab, ao, ab);
                if (math.lengthsq(direction) <= math.Epsilon)
                {
                    direction = GetPerpendicular(ab);
                }
            }
            else
            {
                simplex.Set(a);
                direction = ao;
            }

            return false;
        }

        private static bool UpdateTriangle(ref GJKSimplex simplex, ref fix3 direction)
        {
            GJKSupportPoint a = simplex.a;
            GJKSupportPoint b = simplex.b;
            GJKSupportPoint c = simplex.c;
            fix3 ab = b.point - a.point;
            fix3 ac = c.point - a.point;
            fix3 ao = -a.point;
            fix3 abc = math.cross(ab, ac);

            fix3 acPerp = math.cross(abc, ac);
            if (SameDirection(acPerp, ao))
            {
                if (SameDirection(ac, ao))
                {
                    simplex.Set(a, c);
                    direction = TripleCross(ac, ao, ac);
                    return false;
                }

                return ReduceTriangleToLineAB(ref simplex, ref direction, a, b, ab, ao);
            }

            fix3 abPerp = math.cross(ab, abc);
            if (SameDirection(abPerp, ao))
            {
                return ReduceTriangleToLineAB(ref simplex, ref direction, a, b, ab, ao);
            }

            if (SameDirection(abc, ao))
            {
                direction = abc;
            }
            else
            {
                simplex.Set(a, c, b);
                direction = -abc;
            }

            return math.lengthsq(direction) <= math.Epsilon;
        }

        private static bool ReduceTriangleToLineAB(
            ref GJKSimplex simplex,
            ref fix3 direction,
            GJKSupportPoint a,
            GJKSupportPoint b,
            fix3 ab,
            fix3 ao)
        {
            if (SameDirection(ab, ao))
            {
                simplex.Set(a, b);
                direction = TripleCross(ab, ao, ab);
                if (math.lengthsq(direction) <= math.Epsilon)
                {
                    direction = GetPerpendicular(ab);
                }

                return false;
            }

            simplex.Set(a);
            direction = ao;
            return false;
        }

        private static bool UpdateTetrahedron(ref GJKSimplex simplex, ref fix3 direction)
        {
            GJKSupportPoint a = simplex.a;
            GJKSupportPoint b = simplex.b;
            GJKSupportPoint c = simplex.c;
            GJKSupportPoint d = simplex.d;
            fix3 ao = -a.point;

            if (CheckTetrahedronFace(a, b, c, d, ao, ref simplex, ref direction))
            {
                return false;
            }

            if (CheckTetrahedronFace(a, c, d, b, ao, ref simplex, ref direction))
            {
                return false;
            }

            if (CheckTetrahedronFace(a, d, b, c, ao, ref simplex, ref direction))
            {
                return false;
            }

            return true;
        }

        private static bool CheckTetrahedronFace(
            GJKSupportPoint a,
            GJKSupportPoint b,
            GJKSupportPoint c,
            GJKSupportPoint opposite,
            fix3 ao,
            ref GJKSimplex simplex,
            ref fix3 direction)
        {
            fix3 normal = math.cross(b.point - a.point, c.point - a.point);
            if (SameDirection(normal, opposite.point - a.point))
            {
                normal = -normal;
            }

            if (!SameDirection(normal, ao))
            {
                return false;
            }

            simplex.Set(a, b, c);
            direction = normal;
            return true;
        }

        private static bool SameDirection(fix3 direction, fix3 target)
        {
            return math.dot(direction, target) > fix.Zero;
        }

        private static fix3 TripleCross(fix3 a, fix3 b, fix3 c)
        {
            return math.cross(math.cross(a, b), c);
        }

        private static fix3 GetPerpendicular(fix3 value)
        {
            fix3 axis = math.abs(value.x) < math.abs(value.y) ? fix3.right : fix3.up;
            return NormalizeOrDefault(math.cross(value, axis), fix3.forward);
        }

        private static fix3 NormalizeOrDefault(fix3 value, fix3 fallback)
        {
            fix lengthSq = math.lengthsq(value);
            return lengthSq > math.Epsilon ? value / math.sqrt(lengthSq) : fallback;
        }

        private readonly struct GJKSupportPoint
        {
            public readonly fix3 point;
            public readonly fix3 support0;
            public readonly fix3 support1;

            public GJKSupportPoint(fix3 support0, fix3 support1)
            {
                this.support0 = support0;
                this.support1 = support1;
                point = support0 - support1;
            }

            public bool Equals(GJKSupportPoint other)
            {
                return point == other.point && support0 == other.support0 && support1 == other.support1;
            }
        }

        private struct GJKSimplex
        {
            public GJKSupportPoint a;
            public GJKSupportPoint b;
            public GJKSupportPoint c;
            public GJKSupportPoint d;
            public int count;

            public void Add(GJKSupportPoint point)
            {
                d = c;
                c = b;
                b = a;
                a = point;
                if (count < 4)
                {
                    count++;
                }
            }

            public void Set(GJKSupportPoint point0)
            {
                a = point0;
                b = default;
                c = default;
                d = default;
                count = 1;
            }

            public void Set(GJKSupportPoint point0, GJKSupportPoint point1)
            {
                a = point0;
                b = point1;
                c = default;
                d = default;
                count = 2;
            }

            public void Set(GJKSupportPoint point0, GJKSupportPoint point1, GJKSupportPoint point2)
            {
                a = point0;
                b = point1;
                c = point2;
                d = default;
                count = 3;
            }
        }

        private readonly struct EPAFace
        {
            public readonly GJKSupportPoint a;
            public readonly GJKSupportPoint b;
            public readonly GJKSupportPoint c;
            public readonly fix3 normal;
            public readonly fix distance;

            public EPAFace(GJKSupportPoint a, GJKSupportPoint b, GJKSupportPoint c, fix3 normal, fix distance)
            {
                this.a = a;
                this.b = b;
                this.c = c;
                this.normal = normal;
                this.distance = distance;
            }
        }

        private readonly struct EPADirectedEdge
        {
            public readonly GJKSupportPoint start;
            public readonly GJKSupportPoint end;

            public EPADirectedEdge(GJKSupportPoint start, GJKSupportPoint end)
            {
                this.start = start;
                this.end = end;
            }
        }
    }
}
