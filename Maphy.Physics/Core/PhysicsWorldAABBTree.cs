using Maphy.Mathematics;

namespace Maphy.Physics
{
    /// <summary>
    /// No-GC AABB tree used by PhysicsWorld. Collider handles are mapped directly to
    /// leaf nodes, so normal world syncs move existing proxies instead of rebuilding.
    /// </summary>
    internal sealed class PhysicsWorldAABBTree
    {
        private const int NullNode = -1;

        private readonly fix fatMargin;
        private Node[] nodes;
        private int[] queryStack;
        private int[] colliderNodes;
        private int root = NullNode;
        private int nodeHighWaterMark;
        private int freeList = NullNode;
        private int proxyCount;
        private int queryStackCount;
        private AABB queryBounds;
        private Ray queryRay;
        private fix queryMaxDistance;
        private bool overflowed;

        public PhysicsWorldAABBTree()
            : this(fix._0_1)
        {
        }

        public PhysicsWorldAABBTree(fix fatMargin)
        {
            this.fatMargin = math.max(fix.Zero, fatMargin);
            nodes = System.Array.Empty<Node>();
            queryStack = System.Array.Empty<int>();
            colliderNodes = System.Array.Empty<int>();
        }

        public int ProxyCount => proxyCount;
        public int Height => root == NullNode ? 0 : nodes[root].height;
        public bool Overflowed => overflowed;

        public int MaxBalance
        {
            get
            {
                int maxBalance = 0;
                for (int i = 0; i < nodeHighWaterMark; i++)
                {
                    Node node = nodes[i];
                    if (!node.inUse || node.height <= 1 || node.IsLeaf)
                    {
                        continue;
                    }

                    int balance = System.Math.Abs(nodes[node.left].height - nodes[node.right].height);
                    if (balance > maxBalance)
                    {
                        maxBalance = balance;
                    }
                }

                return maxBalance;
            }
        }

        public void Reserve(int proxyCapacity)
        {
            int nodeCapacity = proxyCapacity > 0 ? proxyCapacity * 2 : 0;
            EnsureCapacity(ref nodes, nodeCapacity);
            EnsureCapacity(ref queryStack, nodeCapacity);
            EnsureIntCapacity(ref colliderNodes, proxyCapacity, NullNode);
        }

        public void Clear()
        {
            root = NullNode;
            nodeHighWaterMark = 0;
            freeList = NullNode;
            proxyCount = 0;
            queryStackCount = 0;
            overflowed = false;

            for (int i = 0; i < colliderNodes.Length; i++)
            {
                colliderNodes[i] = NullNode;
            }
        }

        public bool CreateProxy(ColliderHandle collider, AABB bounds)
        {
            if (!CanStoreCollider(collider))
            {
                overflowed = true;
                return false;
            }

            int existingNode = colliderNodes[collider.index];
            if (IsNodeInUse(existingNode))
            {
                if (nodes[existingNode].collider == collider)
                {
                    return MoveProxy(collider, bounds);
                }

                RemoveProxyAtIndex(collider.index);
            }

            int nodeId = AllocateNode();
            if (nodeId == NullNode)
            {
                return false;
            }

            nodes[nodeId] = new Node
            {
                bounds = CreateFatBounds(bounds),
                tightBounds = bounds,
                collider = collider,
                parent = NullNode,
                left = NullNode,
                right = NullNode,
                height = 0,
                inUse = true,
            };

            colliderNodes[collider.index] = nodeId;
            proxyCount++;
            if (InsertLeaf(nodeId))
            {
                return true;
            }

            colliderNodes[collider.index] = NullNode;
            proxyCount--;
            FreeNode(nodeId);
            return false;
        }

        public bool MoveProxy(ColliderHandle collider, AABB bounds)
        {
            if (!CanStoreCollider(collider))
            {
                overflowed = true;
                return false;
            }

            int nodeId = colliderNodes[collider.index];
            if (!IsNodeInUse(nodeId) || nodes[nodeId].collider != collider)
            {
                if (IsNodeInUse(nodeId))
                {
                    RemoveProxyAtIndex(collider.index);
                }

                return CreateProxy(collider, bounds);
            }

            Node node = nodes[nodeId];
            node.tightBounds = bounds;
            node.collider = collider;
            if (Contains(node.bounds, bounds))
            {
                nodes[nodeId] = node;
                return true;
            }

            RemoveLeaf(nodeId);
            node.bounds = CreateFatBounds(bounds);
            node.parent = NullNode;
            node.left = NullNode;
            node.right = NullNode;
            node.height = 0;
            node.inUse = true;
            nodes[nodeId] = node;
            return InsertLeaf(nodeId);
        }

        public bool RemoveProxy(ColliderHandle collider)
        {
            if (!CanStoreCollider(collider))
            {
                return false;
            }

            int nodeId = colliderNodes[collider.index];
            if (!IsNodeInUse(nodeId) || nodes[nodeId].collider != collider)
            {
                return false;
            }

            RemoveProxyAtIndex(collider.index);
            return true;
        }

        public void RemoveUnmarked(int[] syncMarks, int syncStamp)
        {
            int count = colliderNodes.Length;
            for (int i = 0; i < count; i++)
            {
                int nodeId = colliderNodes[i];
                if (!IsNodeInUse(nodeId))
                {
                    continue;
                }

                if (i >= syncMarks.Length || syncMarks[i] != syncStamp)
                {
                    RemoveProxyAtIndex(i);
                }
            }
        }

        public void ResetOverflow()
        {
            overflowed = false;
        }

        public void BeginQuery(AABB bounds)
        {
            overflowed = false;
            queryBounds = bounds;
            queryStackCount = 0;
            if (root != NullNode)
            {
                PushQueryNode(root);
            }
        }

        public void BeginRaycast(Ray ray, fix maxDistance)
        {
            overflowed = false;
            queryRay = ray;
            queryMaxDistance = maxDistance;
            queryStackCount = 0;
            if (root != NullNode && maxDistance >= fix.Zero)
            {
                PushQueryNode(root);
            }
        }

        public bool TryGetNext(out ColliderHandle collider, out AABB bounds)
        {
            while (queryStackCount > 0)
            {
                int nodeId = queryStack[--queryStackCount];
                Node node = nodes[nodeId];
                if (!Physics.IsOverlap(node.bounds, queryBounds))
                {
                    continue;
                }

                if (node.IsLeaf)
                {
                    collider = node.collider;
                    bounds = node.tightBounds;
                    return true;
                }

                PushQueryNode(node.left);
                PushQueryNode(node.right);
            }

            collider = ColliderHandle.Invalid;
            bounds = default;
            return false;
        }

        public bool TryGetNextRaycast(out ColliderHandle collider, out AABB bounds)
        {
            while (queryStackCount > 0)
            {
                int nodeId = queryStack[--queryStackCount];
                Node node = nodes[nodeId];
                if (!Physics.RaycastAABB(node.bounds, queryRay, queryMaxDistance, out fix _))
                {
                    continue;
                }

                if (node.IsLeaf)
                {
                    collider = node.collider;
                    bounds = node.tightBounds;
                    return true;
                }

                PushQueryNode(node.left);
                PushQueryNode(node.right);
            }

            collider = ColliderHandle.Invalid;
            bounds = default;
            return false;
        }

        private int AllocateNode()
        {
            if (freeList != NullNode)
            {
                int freeNodeId = freeList;
                freeList = nodes[freeNodeId].parent;
                nodes[freeNodeId] = new Node
                {
                    parent = NullNode,
                    left = NullNode,
                    right = NullNode,
                    height = 0,
                    inUse = true,
                };
                return freeNodeId;
            }

            if (nodeHighWaterMark >= nodes.Length)
            {
                overflowed = true;
                return NullNode;
            }

            int nodeId = nodeHighWaterMark++;
            nodes[nodeId] = new Node
            {
                parent = NullNode,
                left = NullNode,
                right = NullNode,
                height = 0,
                inUse = true,
            };
            return nodeId;
        }

        private void FreeNode(int nodeId)
        {
            if (!IsNodeInUse(nodeId))
            {
                return;
            }

            nodes[nodeId] = new Node
            {
                parent = freeList,
                left = NullNode,
                right = NullNode,
                height = -1,
                inUse = false,
            };
            freeList = nodeId;
        }

        private void PushQueryNode(int nodeId)
        {
            if (!IsNodeInUse(nodeId))
            {
                return;
            }

            if (queryStackCount >= queryStack.Length)
            {
                overflowed = true;
                return;
            }

            queryStack[queryStackCount++] = nodeId;
        }

        private bool InsertLeaf(int leaf)
        {
            if (root == NullNode)
            {
                root = leaf;
                Node rootNode = nodes[root];
                rootNode.parent = NullNode;
                nodes[root] = rootNode;
                return true;
            }

            AABB leafBounds = nodes[leaf].bounds;
            int sibling = FindBestSibling(leafBounds);
            int oldParent = nodes[sibling].parent;
            int newParent = AllocateNode();
            if (newParent == NullNode)
            {
                return false;
            }

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
            return true;
        }

        private void RemoveLeaf(int leaf)
        {
            if (leaf == root)
            {
                root = NullNode;
                Node rootLeafNode = nodes[leaf];
                rootLeafNode.parent = NullNode;
                nodes[leaf] = rootLeafNode;
                return;
            }

            Node leafNodeForParent = nodes[leaf];
            int parent = leafNodeForParent.parent;
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
                index = Balance(index);
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

        private int Balance(int nodeId)
        {
            Node node = nodes[nodeId];
            if (!node.inUse || node.IsLeaf || node.height < 2)
            {
                return nodeId;
            }

            int leftId = node.left;
            int rightId = node.right;
            Node left = nodes[leftId];
            Node right = nodes[rightId];
            int balance = right.height - left.height;

            if (balance > 1)
            {
                return RotateLeft(nodeId, leftId, rightId);
            }

            if (balance < -1)
            {
                return RotateRight(nodeId, leftId, rightId);
            }

            return nodeId;
        }

        private int RotateLeft(int nodeId, int leftId, int rightId)
        {
            Node node = nodes[nodeId];
            Node left = nodes[leftId];
            Node right = nodes[rightId];
            int rightLeftId = right.left;
            int rightRightId = right.right;
            Node rightLeft = nodes[rightLeftId];
            Node rightRight = nodes[rightRightId];

            right.left = nodeId;
            right.parent = node.parent;
            node.parent = rightId;

            if (right.parent != NullNode)
            {
                Node parent = nodes[right.parent];
                if (parent.left == nodeId)
                {
                    parent.left = rightId;
                }
                else
                {
                    parent.right = rightId;
                }

                nodes[right.parent] = parent;
            }
            else
            {
                root = rightId;
            }

            if (rightLeft.height > rightRight.height)
            {
                right.right = rightLeftId;
                node.right = rightRightId;
                rightRight.parent = nodeId;
                node.bounds = Combine(left.bounds, rightRight.bounds);
                right.bounds = Combine(node.bounds, rightLeft.bounds);
                node.height = 1 + System.Math.Max(left.height, rightRight.height);
                right.height = 1 + System.Math.Max(node.height, rightLeft.height);
                nodes[rightRightId] = rightRight;
            }
            else
            {
                right.right = rightRightId;
                node.right = rightLeftId;
                rightLeft.parent = nodeId;
                node.bounds = Combine(left.bounds, rightLeft.bounds);
                right.bounds = Combine(node.bounds, rightRight.bounds);
                node.height = 1 + System.Math.Max(left.height, rightLeft.height);
                right.height = 1 + System.Math.Max(node.height, rightRight.height);
                nodes[rightLeftId] = rightLeft;
            }

            nodes[nodeId] = node;
            nodes[rightId] = right;
            return rightId;
        }

        private int RotateRight(int nodeId, int leftId, int rightId)
        {
            Node node = nodes[nodeId];
            Node left = nodes[leftId];
            Node right = nodes[rightId];
            int leftLeftId = left.left;
            int leftRightId = left.right;
            Node leftLeft = nodes[leftLeftId];
            Node leftRight = nodes[leftRightId];

            left.left = nodeId;
            left.parent = node.parent;
            node.parent = leftId;

            if (left.parent != NullNode)
            {
                Node parent = nodes[left.parent];
                if (parent.left == nodeId)
                {
                    parent.left = leftId;
                }
                else
                {
                    parent.right = leftId;
                }

                nodes[left.parent] = parent;
            }
            else
            {
                root = leftId;
            }

            if (leftLeft.height > leftRight.height)
            {
                left.right = leftLeftId;
                node.left = leftRightId;
                leftRight.parent = nodeId;
                node.bounds = Combine(right.bounds, leftRight.bounds);
                left.bounds = Combine(node.bounds, leftLeft.bounds);
                node.height = 1 + System.Math.Max(right.height, leftRight.height);
                left.height = 1 + System.Math.Max(node.height, leftLeft.height);
                nodes[leftRightId] = leftRight;
            }
            else
            {
                left.right = leftRightId;
                node.left = leftLeftId;
                leftLeft.parent = nodeId;
                node.bounds = Combine(right.bounds, leftLeft.bounds);
                left.bounds = Combine(node.bounds, leftRight.bounds);
                node.height = 1 + System.Math.Max(right.height, leftLeft.height);
                left.height = 1 + System.Math.Max(node.height, leftRight.height);
                nodes[leftLeftId] = leftLeft;
            }

            nodes[nodeId] = node;
            nodes[leftId] = left;
            return leftId;
        }

        private AABB CreateFatBounds(AABB bounds)
        {
            fix3 expansion = new fix3(fatMargin);
            return AABB.FromMinMax(bounds.min - expansion, bounds.max + expansion);
        }

        private void RemoveProxyAtIndex(int colliderIndex)
        {
            int nodeId = colliderNodes[colliderIndex];
            if (!IsNodeInUse(nodeId))
            {
                colliderNodes[colliderIndex] = NullNode;
                return;
            }

            RemoveLeaf(nodeId);
            FreeNode(nodeId);
            colliderNodes[colliderIndex] = NullNode;
            proxyCount--;
        }

        private bool CanStoreCollider(ColliderHandle collider)
        {
            return collider.index >= 0 && collider.index < colliderNodes.Length;
        }

        private bool IsNodeInUse(int nodeId)
        {
            return nodeId >= 0 && nodeId < nodeHighWaterMark && nodes[nodeId].inUse;
        }

        private static bool Contains(AABB outer, AABB inner)
        {
            return outer.min.x <= inner.min.x
                && outer.min.y <= inner.min.y
                && outer.min.z <= inner.min.z
                && outer.max.x >= inner.max.x
                && outer.max.y >= inner.max.y
                && outer.max.z >= inner.max.z;
        }

        private static AABB Combine(AABB a, AABB b)
        {
            return AABB.FromMinMax(math.min(a.min, b.min), math.max(a.max, b.max));
        }

        private static fix SurfaceArea(AABB bounds)
        {
            fix3 size = math.abs(bounds.size);
            return fix._2 * (size.x * size.y + size.x * size.z + size.y * size.z);
        }

        private static void EnsureCapacity<T>(ref T[] array, int capacity)
        {
            if (capacity > array.Length)
            {
                System.Array.Resize(ref array, capacity);
            }
        }

        private static void EnsureIntCapacity(ref int[] array, int capacity, int defaultValue)
        {
            int previousLength = array.Length;
            if (capacity > previousLength)
            {
                System.Array.Resize(ref array, capacity);
                for (int i = previousLength; i < array.Length; i++)
                {
                    array[i] = defaultValue;
                }
            }
        }

        private struct Node
        {
            public AABB bounds;
            public AABB tightBounds;
            public ColliderHandle collider;
            public int parent;
            public int left;
            public int right;
            public int height;
            public bool inUse;

            public bool IsLeaf => left == NullNode;
        }
    }
}
