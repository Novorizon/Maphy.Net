
using System;

namespace Maphy.Mathematics
{
    public struct Line
    {
        public fix3 point { get; set; }
        public fix3 direction { get; set; }

        public Line(fix3 point, fix3 direction)
        {
            this.point = point;
            this.direction = direction;
        }
    }
}