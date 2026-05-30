using System;
using System.Collections.Generic;

namespace Maphy.Mathematics
{
    public struct AABB2D
    {
        // AABB2D 的中心点和大小
        public fix2 Center { get; set; }
        public fix2 Extents { get; set; }

        public fix2 center { get { return Center; } set { Center = value; } }
        public fix2 size { get { return Extents * 2; } set { Extents = value * 0.5f; } }
        public fix2 extents { get { return Extents; } set { Extents = value; } }
        public fix2 min { get { return center - extents; } set { SetMinMax(value, max); } }
        public fix2 max { get { return center + extents; } set { SetMinMax(min, value); } }

        public void SetMinMax(fix2 min, fix2 max)
        {
            extents = (max - min) * 0.5f;
            center = min + extents;
        }

        // AABB2D 类的构造函数
        public AABB2D(fix2 center, fix2 size)
        {
            Center = center;
            Extents = size * 0.5f;
        }

        public static bool Contains(AABB2D a, AABB2D b)
        {
            return a.min.x <= b.min.x
                && a.min.y <= b.min.y
                && a.max.x >= b.max.x
                && a.max.y >= b.max.y;
        }

        public static bool Intersects(AABB2D a, AABB2D b)
        {
            return a.min.x <= b.max.x
                && a.max.x >= b.min.x
                && a.min.y <= b.max.y
                && a.max.y >= b.min.y;
        }

        public static AABB2D Merge(AABB2D a, AABB2D b)
        {
            fix minX = math.min(a.min.x, b.min.x);
            fix minY = math.min(a.min.y, b.min.y);
            fix maxX = math.max(a.max.x, b.max.x);
            fix maxY = math.max(a.max.y, b.max.y);

            fix2 center = new fix2((minX + maxX) / 2, (minY + maxY) / 2);
            fix2 size = new fix2(maxX - minX, maxY - minY);
            return new AABB2D(center, size);
        }


    }
}