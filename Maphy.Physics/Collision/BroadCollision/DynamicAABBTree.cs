using System.Collections.Generic;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public sealed class DynamicAABBTree
    {
        private const int NullNode = -1;

        private readonly List<Node> nodes = new List<Node>();
        private readonly Dictionary<ulong, int> proxyNodes = new Dictionary<ulong, int>();
        private readonly List<int> queryStack = new List<int>();
        private readonly fix fatMargin;
        private int root = NullNode;
        private int freeList = NullNode;

        public DynamicAABBTree()
            : this(fix._0_1)
        {
        }

        public DynamicAABBTree(fix fatMargin)
        {
            this.fatMargin = math.max(fix.Zero, fatMargin);
        }

        public int ProxyCount => proxyNodes.Count;

        public void Clear()
        {
            nodes.Clear();
            proxyNodes.Clear();
            queryStack.Clear();
            root = NullNode;
            freeList = NullNode;
        }

        public bool Contains(ulong colliderId)
        {
            return proxyNodes.ContainsKey(colliderId);
        }

        public void CreateProxy(BroadphaseProxy proxy)
        {
            if (proxyNodes.ContainsKey(proxy.colliderId))
            {
                MoveProxy(proxy);
                return;
            }

            int nodeId = AllocateNode();
            Node node = nodes[nodeId];
            node.bounds = CreateFatBounds(proxy.bounds);
            node.proxy = proxy;
            node.height = 0;
            nodes[nodeId] = node;

            proxyNodes.Add(proxy.colliderId, nodeId);
            InsertLeaf(nodeId);
        }

        public bool RemoveProxy(ulong colliderId)
        {
            if (!proxyNodes.TryGetValue(colliderId, out int nodeId))
            {
                return false;
            }

            RemoveLeaf(nodeId);
            proxyNodes.Remove(colliderId);
            FreeNode(nodeId);
            return true;
        }

        public bool MoveProxy(BroadphaseProxy proxy)
        {
            if (!proxyNodes.TryGetValue(proxy.colliderId, out int nodeId))
            {
                CreateProxy(proxy);
                return true;
            }

            Node node = nodes[nodeId];
            if (Contains(node.bounds, proxy.bounds))
            {
                node.proxy = proxy;
                nodes[nodeId] = node;
                return false;
            }

            RemoveLeaf(nodeId);
            node = nodes[nodeId];
            node.bounds = CreateFatBounds(proxy.bounds);
            node.proxy = proxy;
            node.height = 0;
            nodes[nodeId] = node;
            InsertLeaf(nodeId);
            return true;
        }

        public void Query(AABB bounds, List<BroadphaseProxy> results)
        {
            if (results == null || root == NullNode)
            {
                return;
            }

            queryStack.Clear();
            queryStack.Add(root);

            while (queryStack.Count > 0)
            {
                int last = queryStack.Count - 1;
                int nodeId = queryStack[last];
                queryStack.RemoveAt(last);

                Node node = nodes[nodeId];
                if (!Physics.IsOverlap(node.bounds, bounds))
                {
                    continue;
                }

                if (node.IsLeaf)
                {
                    results.Add(node.proxy);
                }
                else
                {
                    queryStack.Add(node.left);
                    queryStack.Add(node.right);
                }
            }
        }

        internal void RemoveExcept(HashSet<ulong> activeColliderIds, List<ulong> removedBuffer)
        {
            removedBuffer.Clear();
            foreach (KeyValuePair<ulong, int> item in proxyNodes)
            {
                if (!activeColliderIds.Contains(item.Key))
                {
                    removedBuffer.Add(item.Key);
                }
            }

            for (int i = 0; i < removedBuffer.Count; i++)
            {
                RemoveProxy(removedBuffer[i]);
            }
        }

        private int AllocateNode()
        {
            if (freeList != NullNode)
            {
                int nodeId = freeList;
                Node node = nodes[nodeId];
                freeList = node.next;
                node.parent = NullNode;
                node.left = NullNode;
                node.right = NullNode;
                node.next = NullNode;
                node.height = 0;
                nodes[nodeId] = node;
                return nodeId;
            }

            nodes.Add(new Node
            {
                parent = NullNode,
                left = NullNode,
                right = NullNode,
                next = NullNode,
                height = 0,
            });
            return nodes.Count - 1;
        }

        private void FreeNode(int nodeId)
        {
            Node node = nodes[nodeId];
            node.parent = NullNode;
            node.left = NullNode;
            node.right = NullNode;
            node.next = freeList;
            node.height = -1;
            node.proxy = default;
            nodes[nodeId] = node;
            freeList = nodeId;
        }

        private void InsertLeaf(int leaf)
        {
            if (root == NullNode)
            {
                root = leaf;
                Node rootNode = nodes[root];
                rootNode.parent = NullNode;
                nodes[root] = rootNode;
                return;
            }

            AABB leafBounds = nodes[leaf].bounds;
            int sibling = FindBestSibling(leafBounds);
            int oldParent = nodes[sibling].parent;
            int newParent = AllocateNode();

            Node parentNode = nodes[newParent];
            parentNode.parent = oldParent;
            parentNode.left = sibling;
            parentNode.right = leaf;
            parentNode.height = nodes[sibling].height + 1;
            parentNode.bounds = Combine(leafBounds, nodes[sibling].bounds);
            nodes[newParent] = parentNode;

            Node siblingNode = nodes[sibling];
            siblingNode.parent = newParent;
            nodes[sibling] = siblingNode;

            Node leafNode = nodes[leaf];
            leafNode.parent = newParent;
            nodes[leaf] = leafNode;

            if (oldParent == NullNode)
            {
                root = newParent;
            }
            else
            {
                Node oldParentNode = nodes[oldParent];
                if (oldParentNode.left == sibling)
                {
                    oldParentNode.left = newParent;
                }
                else
                {
                    oldParentNode.right = newParent;
                }

                nodes[oldParent] = oldParentNode;
            }

            RefitAncestors(newParent);
        }

        private void RemoveLeaf(int leaf)
        {
            if (leaf == root)
            {
                root = NullNode;
                return;
            }

            int parent = nodes[leaf].parent;
            int grandParent = nodes[parent].parent;
            int sibling = nodes[parent].left == leaf ? nodes[parent].right : nodes[parent].left;

            if (grandParent != NullNode)
            {
                Node grandParentNode = nodes[grandParent];
                if (grandParentNode.left == parent)
                {
                    grandParentNode.left = sibling;
                }
                else
                {
                    grandParentNode.right = sibling;
                }

                nodes[grandParent] = grandParentNode;

                Node siblingNode = nodes[sibling];
                siblingNode.parent = grandParent;
                nodes[sibling] = siblingNode;

                FreeNode(parent);
                RefitAncestors(grandParent);
            }
            else
            {
                root = sibling;
                Node siblingNode = nodes[sibling];
                siblingNode.parent = NullNode;
                nodes[sibling] = siblingNode;
                FreeNode(parent);
            }

            Node leafNode = nodes[leaf];
            leafNode.parent = NullNode;
            nodes[leaf] = leafNode;
        }

        private int FindBestSibling(AABB leafBounds)
        {
            int index = root;
            while (!nodes[index].IsLeaf)
            {
                Node node = nodes[index];
                Node left = nodes[node.left];
                Node right = nodes[node.right];
                fix area = SurfaceArea(node.bounds);
                fix combinedArea = SurfaceArea(Combine(node.bounds, leafBounds));
                fix branchCost = fix._2 * combinedArea;
                fix inheritanceCost = fix._2 * (combinedArea - area);

                fix leftCost = GetDescendCost(left, leafBounds, inheritanceCost);
                fix rightCost = GetDescendCost(right, leafBounds, inheritanceCost);
                if (branchCost < leftCost && branchCost < rightCost)
                {
                    break;
                }

                index = leftCost < rightCost ? node.left : node.right;
            }

            return index;
        }

        private static fix GetDescendCost(Node child, AABB leafBounds, fix inheritanceCost)
        {
            AABB combined = Combine(child.bounds, leafBounds);
            if (child.IsLeaf)
            {
                return SurfaceArea(combined) + inheritanceCost;
            }

            return SurfaceArea(combined) - SurfaceArea(child.bounds) + inheritanceCost;
        }

        private void RefitAncestors(int nodeId)
        {
            int index = nodeId;
            while (index != NullNode)
            {
                Node node = nodes[index];
                if (!node.IsLeaf)
                {
                    Node left = nodes[node.left];
                    Node right = nodes[node.right];
                    node.bounds = Combine(left.bounds, right.bounds);
                    node.height = 1 + System.Math.Max(left.height, right.height);
                    nodes[index] = node;
                }

                index = nodes[index].parent;
            }
        }

        private AABB CreateFatBounds(AABB bounds)
        {
            fix3 expansion = new fix3(fatMargin);
            return AABB.FromMinMax(bounds.min - expansion, bounds.max + expansion);
        }

        private static AABB Combine(AABB a, AABB b)
        {
            return AABB.FromMinMax(math.min(a.min, b.min), math.max(a.max, b.max));
        }

        private static bool Contains(AABB outer, AABB inner)
        {
            fix3 outerMin = outer.min;
            fix3 outerMax = outer.max;
            fix3 innerMin = inner.min;
            fix3 innerMax = inner.max;
            return outerMin.x <= innerMin.x
                && outerMin.y <= innerMin.y
                && outerMin.z <= innerMin.z
                && outerMax.x >= innerMax.x
                && outerMax.y >= innerMax.y
                && outerMax.z >= innerMax.z;
        }

        private static fix SurfaceArea(AABB bounds)
        {
            fix3 size = math.abs(bounds.size);
            return fix._2 * (size.x * size.y + size.x * size.z + size.y * size.z);
        }

        private struct Node
        {
            public AABB bounds;
            public BroadphaseProxy proxy;
            public int parent;
            public int left;
            public int right;
            public int next;
            public int height;

            public bool IsLeaf => left == NullNode;
        }
    }
}
