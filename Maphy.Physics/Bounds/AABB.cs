
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct AABB : Bounds
    {
        public static readonly int EDGE = 12;
        public static readonly int FACE = 6;
        public static readonly int VERTEX = 8;
        public static readonly int NORMAL = 3;

        public static readonly fix[] triangles = new fix[36] { 0, 1, 5, 0, 4, 5, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 1, 2, 6, 1, 5, 6, 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 }; //暂时用于与AABB的相交检测
        public static readonly fix3[] Normals = new fix3[3] { fix3.right, fix3.up, fix3.forward };//x轴 左右法线，y轴上下法线，z轴前后法线

        public fix3 Center { get; private set; }
        public fix3 BevelRadius { get; private set; }

        public fix3 Min { get; private set; }
        public fix3 Max { get; private set; }
        public AABB Bounds { get { return this; } }
        public fix3[] Points { get; private set; }//上面 左前 左后 右后 右前 //下面 左前 左后 右后 右前 

        public AABB(fix3 min, fix3 max)
        {
            Points = new fix3[VERTEX];

            Min = min;
            Max = max;

            Center = (min + max) / 2;
            BevelRadius = (max - min) / 2;

            Points[0] = new fix3(Min.x, Max.y, Max.z);
            Points[1] = new fix3(Min.x, Max.y, Min.z);
            Points[2] = new fix3(Max.x, Max.y, Min.z);
            Points[3] = Max;
            Points[4] = new fix3(Min.x, Min.y, Max.z);
            Points[5] = Min;
            Points[6] = new fix3(Max.x, Min.y, Min.z);
            Points[7] = new fix3(Max.x, Min.y, Max.z);
        }

        public AABB(AABB a, AABB b)
        {
            Points = new fix3[VERTEX];
            Min = new fix3(math.min(a.Min.x, b.Min.x), math.min(a.Min.y, b.Min.y), math.min(a.Min.z, b.Min.z));
            Max = new fix3(math.max(a.Max.x, b.Max.x), math.max(a.Max.y, b.Max.y), math.max(a.Max.z, b.Max.z));

            Center = (Min + Max) / 2;
            BevelRadius = (Max - Min) / 2;

            Points[0] = new fix3(Min.x, Max.y, Max.z);
            Points[1] = new fix3(Min.x, Max.y, Min.z);
            Points[2] = new fix3(Max.x, Max.y, Min.z);
            Points[3] = Max;
            Points[4] = new fix3(Min.x, Min.y, Max.z);
            Points[5] = Min;
            Points[6] = new fix3(Max.x, Min.y, Min.z);
            Points[7] = new fix3(Max.x, Min.y, Max.z);

        }
        public void InitAABB(fix3 center, fix3 size)
        {
            Points = new fix3[VERTEX];

            Center = center;
            BevelRadius = size / 2;
            Min = center - size / 2;
            Max = Min + size;

            Points[0] = new fix3(Min.x, Max.y, Max.z);
            Points[1] = new fix3(Min.x, Max.y, Min.z);
            Points[2] = new fix3(Max.x, Max.y, Min.z);
            Points[3] = Max;
            Points[4] = new fix3(Min.x, Min.y, Max.z);
            Points[5] = Min;
            Points[6] = new fix3(Max.x, Min.y, Min.z);
            Points[7] = new fix3(Max.x, Min.y, Max.z);
        }


        public void Update(fix3 center)
        {
            fix3 offset = center - Center;
            Center = center;
            Min = Center - BevelRadius;
            Max = Center + BevelRadius;
            for (int i = 0; i < VERTEX; i++)
            {
                Points[i] += offset;
            }
        }

        public void Update(fix3 min, fix3 max)
        {
            Min = min;
            Max = max;
            Center = (min + max) / 2;

            Points[0] = new fix3(Min.x, Max.y, Max.z);
            Points[1] = new fix3(Min.x, Max.y, Min.z);
            Points[2] = new fix3(Max.x, Max.y, Min.z);
            Points[3] = Max;
            Points[4] = new fix3(Min.x, Min.y, Max.z);
            Points[5] = Min;
            Points[6] = new fix3(Max.x, Min.y, Min.z);
            Points[7] = new fix3(Max.x, Min.y, Max.z);
        }

        public void UpdateBounds(fix3 min, fix3 max)
        {
            this = new AABB(min, max);
        }

    }
}