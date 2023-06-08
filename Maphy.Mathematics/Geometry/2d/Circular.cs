

namespace Maphy.Mathematics
{

    public struct Circular
    {
        public fix2 center;
        public fix radius;
        public fix radius2;
        public Circular(fix2 center, fix radius)
        {

            this.center = center;
            this.radius = radius;
            radius2 = radius * radius;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return (center.GetHashCode() * 397) ^ radius.GetHashCode();
            }
        }

    }
}

