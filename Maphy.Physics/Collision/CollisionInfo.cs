using System.Collections;
using System.Collections.Generic;

using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct CollisionInfo
    {
        public fix penetrationDepth;
        public fix3 normal;

        public fix3 contactPoint1;
        public fix3 contactPoint2;

        public int id;
        public int otherId;

        public CollisionInfo(int id,int otherId)
        {
            this.id = id;
            this.otherId = otherId;

            penetrationDepth = 0;
            normal = new fix3();
            contactPoint1 = new fix3();
            contactPoint2 = new fix3();

        }
    }
}