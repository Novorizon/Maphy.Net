
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        public static bool Raycast(Ray ray, out RaycastHit hitInfo, fix maxDistance, int layerMask)
        {
            hitInfo = new RaycastHit();
            fix dis = 0;
            while (dis < maxDistance)
            {
                //hitInfo.point = geometry.IsOverlap(new AABB(), ray.origin);
                if (hitInfo.point != fix3.MinValue)
                {
                    return true;
                }
                dis += fix._0_1;
                ray.origin += dis * ray.direction;
            }
            return false;
        }

        public static bool Raycast(fix3 origin, fix3 direction, out RaycastHit hitInfo, fix maxDistance, int layerMask)
        {
            hitInfo = new RaycastHit();
            fix dis = 0;
            while (dis < maxDistance)
            {
                //hitInfo.point = geometry.IsOverlap(new Cuboid(), origin);
                if (hitInfo.point != fix3.MinValue)
                {
                    return true;
                }
                dis += fix._0_1;
                origin += dis * direction;
            }
            return false;
        }
    }
}