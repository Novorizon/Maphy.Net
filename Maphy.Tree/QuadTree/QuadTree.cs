
using Maphy.Mathematics;
using Maphy.Physics;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Maphy.Tree
{
    // QuadTree 类
    public class QuadTree
    {
        public enum Quadrant
        {
            None = -1,
            TopLeft = 0,//（左上象限）
            BottomLeft = 1,//（左下象限）
            BottomRight = 2,//（右下象限）
            TopRight = 3,//（右上象限）

        }

        // Node 类，表示四叉树的节点
        public class Node : IDisposable
        {
            private static ulong id = 100000000;
            public ulong Id { get; set; }


            // 节点的四个子节点
            public Node NorthWest;
            public Node NorthEast;
            public Node SouthWest;
            public Node SouthEast;

            //实际节点的id
            public List<ulong> nodeIds;
            //叶子结点
            public List<ulong> leaves;

            public Collider collider;
            // 节点的边界框
            public AABB2D bounds;

            public int level;
            public Quadrant quadrant;

            public Node(Collider collider, AABB2D bounds)
            {
                Id = collider.id;
                this.bounds = bounds;
                this.collider = collider;
            }

            public Node(AABB2D bounds, int level = 0, Quadrant quadrant = Quadrant.None)
            {
                Id = id++;
                this.bounds = bounds;
                this.level = level;
                this.quadrant = quadrant;
                nodeIds = new List<ulong>();
            }

            public Node(Node node)
            {
                Id = node.Id;
                bounds = node.bounds;
                collider = node.collider;
                level = node.level;
                quadrant = node.quadrant;
                nodeIds = new List<ulong>();
                int count = node.nodeIds.Count;
                for (int i = 0; i < count; i++)
                {
                    nodeIds.Add(node.nodeIds[i]);
                }
            }

            public void Dispose()
            {
                Id = 0;
                collider = default;
                level = 0;
                quadrant = Quadrant.None;
                nodeIds.Clear();
                nodeIds = null;
            }
        }

        Dictionary<ulong, Node> nodes = new Dictionary<ulong, Node>();
        Dictionary<ulong, Node> virtuals = new Dictionary<ulong, Node>();
        public Dictionary<ulong, Node> Nodes
        {
            get { return nodes; }
            private set { }
        }

        // 根节点
        private Node root;

        // 叶节点的最大容量
        private int capacity;


        public QuadTree(Collider value, AABB2D bounds)
        {
            capacity = 1;

            Node node = CreateNode(value, bounds);
            root = CreateVirtualNode(bounds, 0);
            root.nodeIds.Add(node.Id);

        }

        public QuadTree(AABB2D bounds)
        {
            capacity = 1;
            root = CreateVirtualNode(bounds, 0);
        }

        private Node CreateNode(Collider value, AABB2D bounds)
        {
            Node node = new Node(value, bounds);
            nodes.Add(node.Id, node);
            return node;
        }


        private Node CreateVirtualNode(AABB2D bounds, int level = 0, Quadrant quadrant = Quadrant.None)
        {
            Node node = new Node(bounds, level, quadrant);
            virtuals.Add(node.Id, node);
            return node;
        }

        public void Insert(Collider value, AABB2D bounds)
        {
            //if (value == null)
            //{
            //    return;
            //}

            Node node = CreateNode(value, bounds);

            // 如果根节点为空，则创建一个新的根节点
            if (root == null)
            {
                root = new Node(default, bounds);
                root.nodeIds.Add(node.Id);
                return;
            }

            // 如果节点的边界框与提供的边界框不相交,向上扩展根节点范围，当前根节点和待插入的新节点作为子节点
            if (!AABB2D.Intersects(bounds, root.bounds))
            {
                AABB2D maxBounds = AABB2D.Merge(root.bounds, bounds);
                ReBuildTree(maxBounds);
                return;
            }

            Insert(node);
        }

        public void Insert(Node nodeNew)
        {
            Collider value = nodeNew.collider;
            AABB2D bounds = nodeNew.bounds;

            // 使用队列进行广度优先遍历
            Queue<Node> queue = new Queue<Node>();
            queue.Enqueue(root);

            while (queue.Count > 0)
            {
                Node node = queue.Dequeue();

                // 如果节点的边界框与提供的边界框不相交，则跳过该节点
                if (!AABB2D.Intersects(node.bounds, bounds))
                    continue;


                // 如果节点有子节点，则根据提供点的边界将它们加入队列
                if (node.NorthWest != null)
                {
                    // 获取新值 相对于当前遍历的节点 应该插入的子节点索引
                    List<Quadrant> indices = GetIndex(node, bounds);
                    for (int i = 0; i < indices.Count; i++)
                    {
                        Quadrant index = indices[i];
                        queue.Enqueue(GetChildNodeByQuadrant(node, index));
                    }
                }
                else
                {
                    // 如果节点没有子节点，则设置其值，并在必要时进行分裂
                    node.nodeIds.Add(nodeNew.Id);

                    int count = node.nodeIds.Count;
                    if (count > capacity)
                    {
                        Split(node);

                        //将数据分给子节点
                        for (int i = 0; i < count; i++)
                        {
                            ulong id = node.nodeIds[i];
                            Node temp = nodes[id];

                            List<Quadrant> indices = GetIndex(node, temp.bounds);
                            for (int j = 0; j < indices.Count; j++)
                            {
                                Quadrant index = indices[j];
                                GetChildNodeByQuadrant(node, index)?.nodeIds.Add(temp.Id);
                            }
                        }
                        node.nodeIds.Clear();
                    }
                }
            }
        }
        private List<Quadrant> GetIndex(AABB2D bounds, AABB2D other)
        {
            List<Quadrant> indices = new List<Quadrant>();

            float midX = bounds.center.x;
            float midY = bounds.center.y;

            bool inNW = other.max.x <= midX && other.max.y <= midY;
            bool inNE = other.min.x >= midX && other.max.y <= midY;
            bool inSW = other.max.x <= midX && other.min.y >= midY;
            bool inSE = other.min.x >= midX && other.min.y >= midY;

            if (inNW)
                indices.Add(Quadrant.TopLeft);
            if (inNE)
                indices.Add(Quadrant.TopRight);
            if (inSW)
                indices.Add(Quadrant.BottomLeft);
            if (inSE)
                indices.Add(Quadrant.BottomRight);

            return indices;
        }

        private List<Quadrant> GetIndex(Node node, AABB2D other)
        {
            List<Quadrant> indices = new List<Quadrant>();

            bool inNW = AABB2D.Intersects(node.NorthWest.bounds, other);
            bool inNE = AABB2D.Intersects(node.NorthEast.bounds, other);
            bool inSW = AABB2D.Intersects(node.SouthWest.bounds, other);
            bool inSE = AABB2D.Intersects(node.SouthEast.bounds, other);

            if (inNW)
                indices.Add(Quadrant.TopLeft);
            if (inNE)
                indices.Add(Quadrant.TopRight);
            if (inSW)
                indices.Add(Quadrant.BottomLeft);
            if (inSE)
                indices.Add(Quadrant.BottomRight);

            return indices;
        }

        // 将节点分裂成四个子节点
        private void Split(Node node)
        {
            int level = node.level + 1;
            fix2 size = node.bounds.center - node.bounds.min;
            fix2 extents = size / 2;

            fix2 ne = new fix2(node.bounds.center.x + extents.x, node.bounds.center.y - extents.y);
            fix2 sw = new fix2(node.bounds.center.x - extents.x, node.bounds.center.y + extents.y);
            fix2 nw = node.bounds.center - extents;
            fix2 se = node.bounds.center + extents;

            node.NorthWest = CreateVirtualNode(new AABB2D(nw, size), level, Quadrant.TopLeft);
            node.NorthEast = CreateVirtualNode(new AABB2D(ne, size), level, Quadrant.TopRight);
            node.SouthWest = CreateVirtualNode(new AABB2D(sw, size), level, Quadrant.BottomLeft);
            node.SouthEast = CreateVirtualNode(new AABB2D(se, size), level, Quadrant.BottomRight);
        }

        // 查询指定范围内的值
        public List<Collider> Retrieve(AABB2D range)
        {
            List<Collider> results = new List<Collider>();
            if (root == null)
                return results;

            Stack<Node> stack = new Stack<Node>();
            stack.Push(root);

            while (stack.Count > 0)
            {
                Node node = stack.Pop();

                // 如果节点的边界框与查询范围不相交，则跳过该节点
                if (!AABB2D.Intersects(node.bounds, range))
                    continue;

                // 如果节点没有子节点，则将其值添加到结果中
                if (node.NorthWest == null)
                {
                    int count = node.nodeIds.Count;
                    for (int i = 0; i < count; i++)
                    {
                        ulong id = node.nodeIds[i];
                        Node leaf = nodes[id];
                        if (AABB2D.Intersects(leaf.bounds, range))
                            results.Add(leaf.collider);
                    }
                }
                else
                {
                    // 如果节点有子节点，如果它们的边界框与查询范围相交 ，则将子节点推入堆栈
                    if (AABB2D.Intersects(node.NorthWest.bounds, range))
                        stack.Push(node.NorthWest);
                    if (AABB2D.Intersects(node.NorthEast.bounds, range))
                        stack.Push(node.NorthEast);
                    if (AABB2D.Intersects(node.SouthWest.bounds, range))
                        stack.Push(node.SouthWest);
                    if (AABB2D.Intersects(node.SouthEast.bounds, range))
                        stack.Push(node.SouthEast);
                }
            }

            return results;
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="bounds">新的最大Bounds</param>
        private void ReBuildTree(AABB2D maxBounds)
        {
            foreach (var item in virtuals)
            {
                Node node = item.Value;
                node = null;
            }
            virtuals.Clear();

            root = CreateVirtualNode(maxBounds, 0);
            foreach (var item in nodes)
            {
                Node node = item.Value;
                Insert(node);
            }
        }

        //仅仅重建root，如果旧的root分布在新root的多个子节点内，旧root的子节点也会分布在多个子节点。不如重建整棵树
        private void ReBuildRoot(Node node)
        {
            Node temp = new Node(root);

            AABB2D aabb = AABB2D.Merge(root.bounds, node.bounds);
            root = CreateVirtualNode(aabb, 0);


            Split(root);

            InsertNodeAsChildren(root, temp);
            InsertNodeAsChildren(root, node);

        }


        private void InsertNodeAsChildren(Node node, Node checkedNode)
        {

            List<Quadrant> indices = GetIndex(node.bounds, checkedNode.bounds);
            for (int i = 0; i < indices.Count; i++)
            {
                Quadrant index = indices[i];
                GetChildNodeByQuadrant(node, index)?.nodeIds.Add(checkedNode.Id);
            }
        }

        private Node GetChildNodeByQuadrant(Node node, Quadrant index) => index switch
        {
            Quadrant.TopLeft => node.NorthWest,
            Quadrant.TopRight => node.NorthEast,
            Quadrant.BottomLeft => node.SouthWest,
            Quadrant.BottomRight => node.SouthEast,
            _ => node,
        };
    }
}