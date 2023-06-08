
using System;

namespace Maphy.Mathematics
{
    public struct Segment
    {
        public fix3 start { get; private set; }
        public fix3 end { get; private set; }

        public Segment(fix3 start, fix3 end)
        {
            this.start = start;
            this.end = end;
        }
    }
}