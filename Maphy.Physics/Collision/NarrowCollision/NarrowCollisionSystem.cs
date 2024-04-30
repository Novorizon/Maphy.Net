using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UnityEditor.ShaderData;

namespace Maphy.Physics
{
    public class NarrowCollisionSystem
    {
        public struct CollisionPair
        {
            public Collider collider0;
            public Collider collider1;

            public CollisionPair(Collider collider0, Collider collider1)
            {
                this.collider0 = collider0;
                this.collider1 = collider1;
            }
        }

        public static void Collision()
        {
            List<BroadCollisionPair> pairs = CollisionSystem.pairs;
            foreach (BroadCollisionPair pair in pairs)
            {
                Collision(pair.collider0.shape, pair.collider1.shape);
            }
        }

        public static bool Collision(Shape shape0, Shape shape1)
        {

            return Physics.Overlaps(shape0, shape1);
        }
    }
}