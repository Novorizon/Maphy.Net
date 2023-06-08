

namespace Maphy.Mathematics
{
    //Regular Hexagon
    public struct Hexagon
    {
        public static readonly int EDGE = 6;
        public static readonly fix INEX = 0.86602540378445;//ÄÚ¾¶:Íâ¾¶
        public fix2[] points;
        public fix2[] edges;
        public fix2[] normals;

        public fix2 a;
        public fix2 b;
        public fix2 c;
        public fix2 d;
        public fix2 e;
        public fix2 f;

        public fix2 center;
        public fix exradius;
        public fix inradius;

        public Polygon polygon;

        public Hexagon(fix2 a, fix2 b, fix2 c, fix2 d, fix2 e, fix2 f)
        {
            points = new fix2[EDGE];
            edges = new fix2[EDGE];
            normals = new fix2[EDGE];

            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
            this.e = e;
            this.f = f;
            points[0] = a;
            points[1] = b;
            points[2] = c;
            points[3] = d;
            points[4] = e;
            points[5] = f;

            for (int i = 0; i < points.Length; i++)
            {
                edges[i] = points[(i + 1) % 4] - points[i];
                normals[i] = new fix2(edges[i].y, -edges[i].x);
            }

            center.x = (a.x + c.x + e.x) / 3;
            center.y = (a.y + c.y + e.y) / 3;
            exradius = math.distance(center, a);
            inradius = INEX * exradius;

            polygon = new Polygon(points);
        }

        public Hexagon(fix2[] p)
        {
            points = new fix2[EDGE];
            edges = new fix2[EDGE];
            normals = new fix2[EDGE];
            a = p[0];
            b = p[1];
            c = p[2];
            d = p[3];
            e = p[4];
            f = p[5];

            for (int i = 0; i < EDGE; i++)
            {
                points[i] = p[i];
                edges[i] = p[(i + 1) % p.Length] - p[i];
                normals[i] = new fix2(edges[i].y, -edges[i].x);
            }
            center.x = (a.x + c.x + e.x) / 3;
            center.y = (a.y + c.y + e.y) / 3;
            exradius = math.distance(center, a);
            inradius = INEX * exradius;

            polygon = new Polygon(points);
        }
    }
}

