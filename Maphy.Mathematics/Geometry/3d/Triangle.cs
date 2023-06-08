
using System;

namespace Maphy.Mathematics
{

    public struct Triangle
    {
        public static readonly int EDGE = 3;
        public fix3[] points;
        public fix3 normal;

        /// 三角形重心
        public fix3 centroid;

        public Triangle(fix3 a, fix3 b, fix3 c)
        {
            points = new fix3[EDGE];

            points[0] = a;
            points[1] = b;
            points[2] = c;

            normal = math.cross(a, b);

            centroid.x = (a.x + b.x + c.x) / 3;
            centroid.y = (a.y + b.y + c.y) / 3;
            centroid.z = (a.z + b.z + c.z) / 3;
        }

        public Triangle(fix3[] p)
        {
            points = new fix3[3];

            normal = math.cross(p[0], p[1]);
            for (int i = 0; i < EDGE; i++)
            {
                points[i] = p[i];
            }

            centroid.x = (p[0].x + p[1].x + p[2].x) / 3;
            centroid.y = (p[0].y + p[1].y + p[2].y) / 3;
            centroid.z = (p[0].z + p[1].z + p[2].z) / 3;
        }
    }
}

