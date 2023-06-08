
using System;

namespace Maphy.Mathematics
{
    public struct Sector
    {
        public static readonly int VERTEX = 2;//Ô²»¡¶Ëµã
        public fix3 center;
        public fix radius;
        public fix radius2;
        public fix3[] points;

        public Sector(fix3 center, fix radius, fix3[] p)
        {
            points = new fix3[VERTEX];

            this.center = center;
            this.radius = radius;
            radius2 = radius * radius;
            points = p;
        }

        public Sector(fix3 center, fix radius, fix3 a, fix3 b)
        {
            points = new fix3[VERTEX];

            this.center = center;
            this.radius = radius;
            radius2 = radius * radius;
            points[0] = a;
            points[1] = b;
        }


        public override int GetHashCode()
        {
            return center.GetHashCode();
        }

    }
}

