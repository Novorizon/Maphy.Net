
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct AABB: Shape
    {
        public static readonly int VERTEX = 8;
        public static readonly int NORMAL = 3;

        public static readonly fix[] triangles = new fix[36] { 0, 1, 5, 0, 4, 5, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 1, 2, 6, 1, 5, 6, 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 }; //暂时用于与AABB的相交检测
        public static readonly fix3[] Normals = new fix3[3] { fix3.right, fix3.up, fix3.forward };//x轴 左右法线，y轴上下法线，z轴前后法线

        public fix3 center { get; private set; }
        public fix3 _extents { get; private set; }

        public fix3 size
        {
            get { return _extents * 2f; }
            set { _extents = value * 0.5f; }
        }

        public fix3 extents
        {
            get { return _extents; }
            set { _extents = value; }
        }

        public fix3 min
        {
            get { return center - extents; }
            set { SetMinMax(value, max); }
        }

        public fix3 max
        {
            get { return center + extents; }
            set { SetMinMax(min, value); }
        }

        public ShapeType Type { get => ShapeType.AABB;}

        public void SetMinMax(fix3 min, fix3 max)
        {
            extents = (max - min) * 0.5f;
            center = min + extents;
        }


        //public AABB Bounds { get { return this; } }
        //public fix3[] Points { get; private set; }//上面 左前 左后 右后 右前 //下面 左前 左后 右后 右前 

        public AABB(fix3 center, fix3 size)
        {
            //Points = new fix3[VERTEX];
            _extents = size/2;
            this.center = center;
        }

        //public void SetPoints(fix3 min, fix3 max)
        //{
        //    Points = new fix3[VERTEX];

        //    Points[0] = new fix3(min.x, max.y, max.z);
        //    Points[1] = new fix3(min.x, max.y, min.z);
        //    Points[2] = new fix3(max.x, max.y, min.z);
        //    Points[3] = max;
        //    Points[4] = new fix3(min.x, min.y, max.z);
        //    Points[5] = min;
        //    Points[6] = new fix3(max.x, min.y, min.z);
        //    Points[7] = new fix3(max.x, min.y, max.z);
        //}

        //public void Update(fix3 center)
        //{
        //    fix3 offset = center - this.center;
        //    this.center = center;
        //    min = this.center - extents;
        //    max = this.center + extents;
        //    for (int i = 0; i < VERTEX; i++)
        //    {
        //        Points[i] += offset;
        //    }
        //}

        public fix3[] GetPoints()
        {
            fix3[] points = new fix3[VERTEX];

            points[0] = new fix3(min.x, max.y, max.z);
            points[1] = new fix3(min.x, max.y, min.z);
            points[2] = new fix3(max.x, max.y, min.z);
            points[3] = max;
            points[4] = new fix3(min.x, min.y, max.z);
            points[5] = min;
            points[6] = new fix3(max.x, min.y, min.z);
            points[7] = new fix3(max.x, min.y, max.z);

            return points;
        }

    }
}