
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap(AABB aabb, fix3 point)
        {
            if (aabb.min.x > point.x || aabb.min.y > point.y || aabb.min.z > point.z || aabb.max.x < point.x || aabb.max.y < point.y || aabb.max.z < point.z)
                return false;
            return true;
        }

        //�������һ���Բ���
        internal static bool IsOverlap(AABB a, AABB b)
        {
            return (a, b) switch
            {
                { a: _, b: _ } when (a.min.x >= b.max.x) => false,
                { a: _, b: _ } when (a.max.x <= b.min.x) => false,
                { a: _, b: _ } when (a.min.y >= b.max.y) => false,
                { a: _, b: _ } when (a.max.y <= b.min.y) => false,
                { a: _, b: _ } when (a.min.z >= b.max.z) => false,
                { a: _, b: _ } when (a.max.z <= b.min.z) => false,
                _ => true,
            };
        }

        public static bool IsOverlap(AABB aabb, Ray ray)
        {
            fix min;
            fix max;
            for (var i = 0; i < 3; i++)
            {
                min = fix.Min;
                max = fix.Max;
                fix t0 = math.min((aabb.min[i] - ray.origin[i]) / ray.direction[i],
                    (aabb.max[i] - ray.origin[i]) / ray.direction[i]);
                fix t1 = math.max((aabb.min[i] - ray.origin[i]) / ray.direction[i],
                    (aabb.max[i] - ray.origin[i]) / ray.direction[i]);
                min = math.max(t0, min);
                max = math.min(t1, max);
                if (max <= min)
                    return false;
            }
            return true;
        }


        public static bool Overlaps(AABB aabb, Sphere sphere)
        {
            return Overlaps(sphere, aabb);
        }

        public static bool Overlaps(AABB a, AABB b)
        {
            // �������AABB���ص����򷵻�false
            if (!IsOverlap(a, b))
                return false;

            // ��ʼ������face����ʾ�����ཻ����
            Face face = Face.None;
            // �����ཻ���������face��ֵ
            face = (a, b) switch
            {
                { a: _, b: _ } when (a.min.x <= b.max.x) => face | Face.Left,
                { a: _, b: _ } when (a.max.x >= b.min.x) => face | Face.Right,
                { a: _, b: _ } when (a.min.y <= b.max.y) => face | Face.Bottom,
                { a: _, b: _ } when (a.max.y >= b.min.y) => face | Face.Top,
                { a: _, b: _ } when (a.min.z <= b.max.z) => face | Face.Back,
                { a: _, b: _ } when (a.max.z >= b.min.z) => face | Face.Front,
                _ => face | Face.None,
            };

            // ���û���ཻ���棬�򷵻�false
            if (face == Face.None)
                return false;

            // ���������ཻ����
            while (face > 0)
            {
                // ��ȡ��ǰ��ķ�������
                fix3 normal = GetFaceNormalOnAABB(a, face);
                // ��ȡAABB��֧�ŵ�
                fix3 supportPoint = GetSupportPoint(a, normal);

                // ��ȡB�϶�Ӧ�ĵ�
                fix3 vertex = (face) switch
                {
                    Face.Left => b.max,
                    Face.Right => b.min,
                    Face.Top => b.max,
                    Face.Bottom => b.min,
                    Face.Front => b.max,
                    Face.Back => b.min,
                    _ => fix3.MinValue,
                };

                // ���㴩͸���
                decimal penetrationDepth = math.dot((vertex - supportPoint), normal);
                if (penetrationDepth > 0)
                {
                    if (needCollisionInfo)
                    {
                        // �����Ҫ��ײ��Ϣ���򴴽�CollisionInfo����
                        CollisionInfo collisionInfo = new CollisionInfo(0, 0);
                        collisionInfo.normal = normal;
                        collisionInfo.penetrationDepth = penetrationDepth;
                        //collisionInfo.contactPoint1 = -planeNormal * sphere.Center;
                        //collisionInfo.contactPoint2 = sphere.Center + planeNormal * (penetrationDepth - sphere.Radius);
                    }
                    return true;
                }
                face = (Face)((int)face >> 1);
            }
            return false;
        }


        public static bool Overlaps(AABB a, OBB b)
        {
            // �������AABB���ص����򷵻�false
            if (!IsOverlap(a, b))
                return false;

            // ��ʼ������face����ʾ�����ཻ����
            Face face = Face.None;
            // �����ཻ���������face��ֵ
            face = (a, b) switch
            {
                { a: _, b: _ } when (a.min.x <= b.max.x) => face | Face.Left,
                { a: _, b: _ } when (a.max.x >= b.min.x) => face | Face.Right,
                { a: _, b: _ } when (a.min.y <= b.max.y) => face | Face.Bottom,
                { a: _, b: _ } when (a.max.y >= b.min.y) => face | Face.Top,
                { a: _, b: _ } when (a.min.z <= b.max.z) => face | Face.Back,
                { a: _, b: _ } when (a.max.z >= b.min.z) => face | Face.Front,
                _ => face | Face.None,
            };

            // ���û���ཻ���棬�򷵻�false
            if (face == Face.None)
                return false;

            // ���������ཻ����
            while (face > 0)
            {
                // ��ȡ��ǰ��ķ�������
                fix3 normal = GetFaceNormalOnAABB(a, face);
                // ��ȡAABB��֧�ŵ�
                fix3 supportPoint = GetSupportPoint(a, normal);

                // ��ȡB�϶�Ӧ�ĵ�
                fix3 vertex = (face) switch
                {
                    Face.Left => b.max,
                    Face.Right => b.min,
                    Face.Top => b.max,
                    Face.Bottom => b.min,
                    Face.Front => b.max,
                    Face.Back => b.min,
                    _ => fix3.MinValue,
                };

                // ���㴩͸���
                decimal penetrationDepth = math.dot((vertex - supportPoint), normal);
                if (penetrationDepth > 0)
                {
                    if (needCollisionInfo)
                    {
                        // �����Ҫ��ײ��Ϣ���򴴽�CollisionInfo����
                        CollisionInfo collisionInfo = new CollisionInfo(0, 0);
                        collisionInfo.normal = normal;
                        collisionInfo.penetrationDepth = penetrationDepth;
                        //collisionInfo.contactPoint1 = -planeNormal * sphere.Center;
                        //collisionInfo.contactPoint2 = sphere.Center + planeNormal * (penetrationDepth - sphere.Radius);
                    }
                    return true;
                }
                face = (Face)((int)face >> 1);
            }
            return false;
        }

        public static bool Overlaps(AABB a, Capsule b)
        {
            return Overlaps(a, b);
        }


        public static AABB FromMinMax(fix3 min, fix3 max)
        {
            fix3 center = (min + max) / 2;
            fix3 size = (max - min) / 2;

            AABB aabb = new AABB(center, size);
            return aabb;
        }

        public static AABB Merge(AABB a, AABB b)
        {
            fix3 min = math.min(a.min, b.min);
            fix3 max = math.max(a.max, b.max);

            fix3 center = (min + max) / 2;
            fix3 size = (max - min) / 2;

            AABB aabb = new AABB(center, size);
            return aabb;
        }


    }

}
