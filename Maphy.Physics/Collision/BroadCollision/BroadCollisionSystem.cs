using Maphy.Tree;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Maphy.Physics
{
    public struct BroadCollisionPair
    {
        public Collider collider0;
        public Collider collider1;

        public BroadCollisionPair(Collider collider0, Collider collider1)
        {
            this.collider0 = collider0;
            this.collider1 = collider1;
        }
    }
    public class BroadCollisionSystem
    {
        static QuadTree quadTree;
        static Dictionary<int, List<Collider>> nodes = new Dictionary<int, List<Collider>>();
        static List<BroadCollisionPair> broadCollisionPairs = new List<BroadCollisionPair>();

        // 以叶子结点为单位，返回节点内的潜在碰撞对
        public static List<BroadCollisionPair> Collision()
        {
            List<List<Collider>> nodes = quadTree.GetCollidersInSameLeaf();

            foreach (var nodeList in nodes)
            {
                int count = nodeList.Count;
                for (int i = 0; i < count; i++)
                {
                    for (int j = i + 1; j < count; j++)
                    {
                        if (IsBroadCollision(nodeList[i], nodeList[j]))
                        {
                            BroadCollisionPair pairs = new BroadCollisionPair(nodeList[i], nodeList[j]);
                            broadCollisionPairs.Add(pairs);
                        }
                    }
                }
            }
            return broadCollisionPairs;
        }

        public static bool IsBroadCollision(Collider a, Collider b)
        {
            if (!quadTree.IsInSameLeaf(a, b))
                return false;

            return quadTree.IsOverlap(a, b);
        }
    }
}
