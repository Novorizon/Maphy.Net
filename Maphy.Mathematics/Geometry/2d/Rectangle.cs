
using System;
using System.Collections.Generic;

namespace Maphy.Mathematics
{
    [Serializable]
    public struct Rectangle
    {
        public static readonly int EDGE = 4;
        public fix2[] points;
        public fix2[] edges;
        public fix2[] normals;

        public fix2 a;
        public fix2 b;
        public fix2 c;
        public fix2 d;

        public fix2 bevelRadius;
        public fix2 centroid;
        public quaternion orientation;

        public Polygon polygon;


        public Rectangle(fix2 a, fix2 b, fix2 c, fix2 d, quaternion rotation = default)
        {
            points = new fix2[EDGE];
            edges = new fix2[EDGE];
            normals = new fix2[EDGE];

            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
            points[0] = a;
            points[1] = b;
            points[2] = c;
            points[3] = d;

            for (int i = 0; i < EDGE; i++)
            {
                edges[i] = points[(i + 1) % 4] - points[i];
                normals[i] = new fix2(edges[i].y, -edges[i].x);
            }

            centroid.x = (a.x + b.x + c.x + d.y) / 4;
            centroid.y = (a.y + b.y + c.y + d.y) / 4;
            bevelRadius = math.abs(points[0] - centroid);
            orientation = rotation;
            polygon = new Polygon(points);
        }


        public Rectangle(fix2[] p, quaternion rotation = default)
        {
            points = new fix2[4];
            edges = new fix2[4];
            normals = new fix2[4];

            a = p[0];
            b = p[1];
            c = p[2];
            d = p[3];

            points[0] = a;
            points[1] = a;
            points[2] = a;
            points[3] = d;

            for (int i = 0; i < EDGE; i++)
            {
                edges[i] = points[(i + 1) % 4] - points[i];
                normals[i] = new fix2(edges[i].y, -edges[i].x);
            }

            centroid.x = (a.x + b.x + c.x + d.y) / 4;
            centroid.y = (a.y + b.y + c.y + d.y) / 4;
            bevelRadius = math.abs(points[0] - centroid);
            orientation = rotation;
            polygon = new Polygon(points);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = a.GetHashCode();
                hashCode = (hashCode * 397) ^ b.GetHashCode();
                hashCode = (hashCode * 397) ^ c.GetHashCode();
                hashCode = (hashCode * 397) ^ d.GetHashCode();
                return hashCode;
            }
        }


    }
}