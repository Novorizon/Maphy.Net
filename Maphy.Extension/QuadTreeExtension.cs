using log4net.Core;
using Maphy.Mathematics;
using Maphy.Tree;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static Maphy.Tree.QuadTree;

namespace Maphy.Physics
{
    public static class QuadTreeExtension
    {
        public static List<Node> GetLeaves(this QuadTree quadTree)
        {
            List<Node> leaves = new List<Node>();
            //foreach (T t in leaves)
            //{
            //}
            return leaves;
        }

        public static List<List<Collider>> GetCollidersInSameLeaf(this QuadTree quadTree)
        {
            List<List<Collider>> colliders = new List<List<Collider>>();
            List<Node> leaves = GetLeaves(quadTree);
            foreach (Node leaf in leaves)
            {
                List<ulong> ids=leaf.nodeIds;
                Dictionary<ulong,Collider>nodeMap= new Dictionary<ulong,Collider>();

                List<Collider> colliderList = new List<Collider>();
                foreach (ulong id in ids) 
                {
                    if(nodeMap.ContainsKey(id))
                    {
                        colliderList.Add(nodeMap[id]);
                    }
                }
                colliders.Add(colliderList);
            }
            return colliders;
        }

        public static Node GetNode(this QuadTree quadTree, Collider a)
        {
            if (quadTree == null)
                return null;
            if(quadTree.Nodes.ContainsKey(a.id))
            {
                return quadTree.Nodes[a.id];
            }
            return null;
        }

        public static bool IsInSameLeaf(this QuadTree quadTree,Collider a, Collider b)
        {
            return true;
        }

        public static int GetLeaf(this QuadTree quadTree, Collider a)
        {
            return 0;
        }
        public static List<Collider> GetAdjacentColliders(this QuadTree quadTree, Collider a)
        {
            List < Collider >colliders = new List<Collider>();
            return colliders;
        }

        public static bool IsOverlap(this QuadTree quadTree, Collider a, Collider b)
        {
            Node node0=GetNode(quadTree, a);
            Node node1=GetNode(quadTree, b);


            return AABB2D.Intersects(node0.bounds, node1.bounds);
        }
    }
}