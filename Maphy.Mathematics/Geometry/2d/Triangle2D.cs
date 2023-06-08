
using System;

namespace Maphy.Mathematics
{

    public struct Triangle2D
    {
        public static readonly int EDGE = 3;
        public fix2[] points;
        public fix2[] edges;
        public fix2[] normals;

        public fix2 a;
        public fix2 b;
        public fix2 c;
        /// 三角形重心
        public fix2 centroid;

        public Polygon polygon;
        public Triangle2D(fix2 a, fix2 b, fix2 c)
        {
            points = new fix2[EDGE];
            edges = new fix2[EDGE];
            normals = new fix2[EDGE];

            this.a = a;
            this.b = b;
            this.c = c;
            points[0] = a;
            points[1] = b;
            points[2] = c;

            for (int i = 0; i < EDGE; i++)
            {
                edges[i] = points[(i + 1) % 3] - points[i];
                normals[i] = new fix2(edges[i].y, -edges[i].x);
            }

            centroid.x = (a.x + b.x + c.x) / 3;
            centroid.y = (a.y + b.y + c.y) / 3;
            polygon = new Polygon(points);
        }

        public Triangle2D(fix2[] p)
        {
            points = new fix2[3];
            edges = new fix2[3];
            normals = new fix2[3];

            a = p[0];
            b = p[1];
            c = p[2];

            for (int i = 0; i < EDGE; i++)
            {
                points[i] = p[i];
                edges[i] = p[(i + 1) % 3] - p[i];
                normals[i] = new fix2(edges[i].y, -edges[i].x);
            }

            centroid.x = (a.x + b.x + c.x) / 3;
            centroid.y = (a.y + b.y + c.y) / 3;
            polygon = new Polygon(points);
        }
    }
}

