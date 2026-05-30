using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct AABB : Shape
    {
        public const int VERTEX = 8;
        public const int NORMAL = 3;

        public static readonly fix[] triangles = new fix[36] { 0, 1, 5, 0, 4, 5, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 1, 2, 6, 1, 5, 6, 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 };
        public static readonly fix3[] Normals = new fix3[3] { fix3.right, fix3.up, fix3.forward };

        public fix3 center { get; private set; }
        public fix3 _extents { get; private set; }

        public ShapeType Type => ShapeType.AABB;

        public fix3 size
        {
            get { return _extents * fix._2; }
            set { _extents = value * fix._0_5; }
        }

        public fix3 extents
        {
            get { return _extents; }
            set { _extents = value; }
        }

        public fix3 min
        {
            get { return center - _extents; }
            set { SetMinMax(value, max); }
        }

        public fix3 max
        {
            get { return center + _extents; }
            set { SetMinMax(min, value); }
        }

        public AABB(fix3 center, fix3 size)
        {
            _extents = size * fix._0_5;
            this.center = center;
        }

        public static AABB FromMinMax(fix3 min, fix3 max)
        {
            return new AABB((min + max) * fix._0_5, max - min);
        }

        public void SetMinMax(fix3 min, fix3 max)
        {
            _extents = (max - min) * fix._0_5;
            center = min + _extents;
        }

        public fix3[] GetPoints()
        {
            fix3 min = this.min;
            fix3 max = this.max;
            return new fix3[VERTEX]
            {
                new fix3(min.x, max.y, max.z),
                new fix3(min.x, max.y, min.z),
                new fix3(max.x, max.y, min.z),
                max,
                new fix3(min.x, min.y, max.z),
                min,
                new fix3(max.x, min.y, min.z),
                new fix3(max.x, min.y, max.z)
            };
        }
    }
}