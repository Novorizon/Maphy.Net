using Maphy.Mathematics;

namespace Maphy.Physics
{
    public readonly struct ShapeCastHit
    {
        public readonly fix fraction;
        public readonly fix3 normal;
        public readonly fix3 point;

        public ShapeCastHit(fix fraction, fix3 normal, fix3 point)
        {
            this.fraction = fraction;
            this.normal = normal;
            this.point = point;
        }
    }
}
