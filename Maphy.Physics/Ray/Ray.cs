
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct Ray
    {
        public fix3 origin { get; set; }
        public fix3 direction { get; set; }

        public Ray(fix3 origin, fix3 direction)
        {
            this.origin = origin;
            this.direction = direction;
        }

        public fix3 GetPoint(fix distance)
        {
            return origin + distance * direction;
        }

        public override string ToString()
        {
            return origin.ToString() + " " + direction.ToString();
        }
    }
}