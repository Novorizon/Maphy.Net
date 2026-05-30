using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    [Serializable]
    public struct OBB : Shape
    {
        public const int EDGE = 12;
        public const int FACE = 6;
        public const int VERTEX = 8;
        public const int NORMAL = 3;

        public static readonly fix[] Triangles = new fix[36] { 0, 1, 5, 0, 4, 5, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 1, 2, 6, 1, 5, 6, 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 };

        public ShapeType Type => ShapeType.OBB;
        public fix3 center { get; private set; }
        public fix3 _extents { get; private set; }
        public quaternion orientation { get; private set; }

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
            get { return center - WorldExtents(); }
            set { SetMinMax(value, max); }
        }

        public fix3 max
        {
            get { return center + WorldExtents(); }
            set { SetMinMax(min, value); }
        }

        public OBB(fix3 center, fix3 size, quaternion rotation)
        {
            this.center = center;
            _extents = size * fix._0_5;
            orientation = rotation;
        }

        public void Update(fix3 center, quaternion rotation)
        {
            this.center = center;
            orientation = rotation;
        }

        public void SetMinMax(fix3 min, fix3 max)
        {
            _extents = (max - min) * fix._0_5;
            center = min + _extents;
        }

        public fix3[] GetPoints()
        {
            fix3 x = (orientation * fix3.right) * _extents.x;
            fix3 y = (orientation * fix3.up) * _extents.y;
            fix3 z = (orientation * fix3.forward) * _extents.z;

            return new fix3[VERTEX]
            {
                center - x + y + z,
                center - x + y - z,
                center + x + y - z,
                center + x + y + z,
                center - x - y + z,
                center - x - y - z,
                center + x - y - z,
                center + x - y + z
            };
        }

        private fix3 WorldExtents()
        {
            fix3 axisX = orientation * fix3.right;
            fix3 axisY = orientation * fix3.up;
            fix3 axisZ = orientation * fix3.forward;
            return math.abs(axisX) * _extents.x
                + math.abs(axisY) * _extents.y
                + math.abs(axisZ) * _extents.z;
        }
    }
}