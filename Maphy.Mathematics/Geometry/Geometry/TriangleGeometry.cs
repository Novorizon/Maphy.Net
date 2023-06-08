using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class Geometry
    {
        /// 三角形是否相邻
        public static Tuple<bool, fix3, fix3> CoincideTriangle(Triangle TriangleA, Triangle TriangleB)
        {
            bool isCoincide = false;
            fix3 left = new fix3();
            fix3 right = new fix3();

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (Equals(TriangleA.points[i], TriangleB.points[(j + 1) % 3]) && Equals(TriangleA.points[(i + 1) % 3], TriangleB.points[j]))
                    {
                        left = TriangleA.points[i];
                        right = TriangleB.points[j];
                        isCoincide = true;
                        break;
                    }
                }
                if (isCoincide)
                    break;
            }

            if (isCoincide)
            {
                fix3 l = left - TriangleA.centroid;
                fix3 r = right - TriangleA.centroid;
                fix angle = AngleSigned(l, r, fix3.up);
                if (angle < 0)
                {
                    fix3 p = left;
                    left = right;
                    right = p;
                }
            }
            return new Tuple<bool, fix3, fix3>(isCoincide, left, right);
        }
    }
}
