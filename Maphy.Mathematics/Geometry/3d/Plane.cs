
namespace Maphy.Mathematics
{
    public struct Plane
    {
        public fix3 point;
        public fix3 normal;

        public Plane(fix3 point, fix3 normal)
        {
            this.point = point;
            this.normal = normal;
        }
    }
}

