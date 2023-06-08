
namespace Maphy.Mathematics
{

    public struct Polygon
    {
        public static int EDGE;
        public fix2[] points;
        public fix2[] edges;
        public fix2[] normals;
        public Polygon(fix2[] p)
        {
            EDGE = p.Length;

            points = new fix2[EDGE];
            edges = new fix2[EDGE];
            normals = new fix2[EDGE];
            for (int i = 0; i < EDGE; i++)
            {
                points[i] = p[i];
                edges[i] = p[(i + 1) % EDGE] - p[i];
                normals[i] = new fix2(edges[i].y, -edges[i].x);
            }
        }
    }
}

