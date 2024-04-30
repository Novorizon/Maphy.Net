
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    [Serializable]
    public struct OBB : Shape
    {
        public ShapeType Type { get => ShapeType.OBB; }
        public fix3 center { get; private set; }
        public fix3 _extents { get; private set; }
        public quaternion orientation { get; private set; }
        public fix3 size
        {
            get { return extents * 2f; }
            set { extents = value * 0.5f; }
        }

        public fix3 extents
        {
            get { return extents; }
            set { extents = value; }
        }

        public fix3 min
        {
            get { return orientation*(center - extents); }
            set { SetMinMax(value, max); }
        }

        public fix3 max
        {
            get { return orientation * (center + extents); }
            set { SetMinMax(min, value); }
        }

        public void SetMinMax(fix3 min, fix3 max)
        {
            extents = (max - min) * 0.5f;
            center = min + extents;
        }


        public static readonly int EDGE = 12;
        public static readonly int FACE = 6;
        public static readonly int VERTEX = 8;
        public static readonly int NORMAL = 3;
        public static readonly fix[] Triangles = new fix[36] { 0, 1, 5, 0, 4, 5, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 1, 2, 6, 1, 5, 6, 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 };

        //public fix3 Size { get; private set; }
        //public fix3 BevelRadius { get; private set; }
        //public AABB Bounds { get; private set; }
        //public fix3 Min { get; private set; }
        //public fix3 Max { get; private set; }

        //public fix3[] Points { get; private set; }//上面左前 左后 右后 右前 下面左前 左后 右后 右前 
        //public fix3[] PointNormals { get; private set; }//用于判断和点的位置关系
        //public fix3[] Normals { get; private set; }//x轴 左右法线，y轴上下法线，z轴前后法线}

        //fix3[] points;


        public OBB(fix3 center, fix3 size,quaternion rotation)
        {
            this.center = center;
           _extents= size/2;
            orientation = rotation;
        }
        //public void Update(fix3 center, fix3 forward, fix3 up)
        //{
        //    center = center;
        //    orientation = quaternion.LookRotation(forward, up);

        //    //Min = fix3.MaxValue;
        //    //Max = fix3.MinValue;
        //    //for (int i = 0; i < VERTEX; i++)
        //    //{
        //    //    Points[i] = orientation * (points[i] - this.center) + center;
        //    //    Min = math.min(Min, Points[i]);
        //    //    Max = math.max(Max, Points[i]);
        //    //}

        //    //Normals[0] = orientation * fix3.right;
        //    //Normals[1] = orientation * fix3.up;
        //    //Normals[2] = orientation * fix3.forward;

        //    //PointNormals[0] = Points[0];
        //    //PointNormals[1] = Points[3];
        //    //PointNormals[2] = Points[4];
        //    //PointNormals[3] = Points[1];
        //    //Bounds = new AABB(Min, Max);
        //}

        //public void Update(fix3 center, quaternion rotation)
        //{
        //    center = center;
        //    orientation = rotation;

        //    Min = fix3.MaxValue;
        //    Max = fix3.MinValue;
        //    for (int i = 0; i < VERTEX; i++)
        //    {
        //        Points[i] = orientation * (points[i] - this.center) + center;
        //        Min = math.min(Min, Points[i]);
        //        Max = math.max(Max, Points[i]);
        //    }

        //    Normals[0] = orientation * fix3.right;
        //    Normals[1] = orientation * fix3.up;
        //    Normals[2] = orientation * fix3.forward;

        //    PointNormals[0] = Points[0];
        //    PointNormals[1] = Points[3];
        //    PointNormals[2] = Points[4];
        //    PointNormals[3] = Points[1];

        //    Bounds = new AABB(Min, Max);
        //}
    }
}

