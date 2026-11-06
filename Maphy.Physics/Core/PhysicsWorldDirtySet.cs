using System;
using System.Runtime.CompilerServices;

namespace Maphy.Physics
{
    /// <summary>
    /// Reserved hierarchical bitset. Draining visits set indices in ascending order
    /// without scanning clean body/collider slots. Only Reserve may allocate.
    /// </summary>
    internal sealed class PhysicsWorldDirtySet
    {
        private ulong[][] levels = Array.Empty<ulong[]>();

        public void Reserve(int capacity)
        {
            int words = WordCount(capacity);
            if (words == 0 || (levels.Length != 0 && words <= levels[0].Length)) return;
            int height = 1;
            for (int n = words; n > 1; n = WordCount(n)) height++;
            var grown = new ulong[height][];
            grown[0] = new ulong[words];
            if (levels.Length != 0) Array.Copy(levels[0], grown[0], levels[0].Length);
            // Reconstruct summaries, including dirty bits pending across growth.
            for (int level = 1; level < height; level++)
            {
                ulong[] children = grown[level - 1];
                grown[level] = new ulong[WordCount(children.Length)];
                for (int i = 0; i < children.Length; i++)
                    if (children[i] != 0) grown[level][i >> 6] |= 1UL << (i & 63);
            }
            levels = grown;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(int index)
        {
            for (int level = 0; level < levels.Length; level++)
            {
                int word = index >> 6;
                ulong previous = levels[level][word];
                levels[level][word] = previous | (1UL << (index & 63));
                if (previous != 0) break;
                index = word;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Remove(int index)
        {
            for (int level = 0; level < levels.Length; level++)
            {
                int word = index >> 6;
                ulong previous = levels[level][word];
                if (previous == 0) break;
                ulong remaining = previous & ~(1UL << (index & 63));
                levels[level][word] = remaining;
                if (remaining != 0) break;
                index = word;
            }
        }

        public bool TryTakeFirstWord(out int start, out ulong bits)
        {
            start = 0;
            bits = 0;
            int level = levels.Length - 1;
            if (level < 0 || levels[level][0] == 0) return false;
            int word = 0;
            for (; level > 0; level--)
                word = (word << 6) + FirstBit(levels[level][word]);
            start = word << 6;
            bits = levels[0][word];
            levels[0][word] = 0;
            // Clear summaries once per nonempty leaf, not once per dirty object.
            for (level = 1; level < levels.Length; level++)
            {
                int parent = word >> 6;
                ulong remaining = levels[level][parent] & ~(1UL << (word & 63));
                levels[level][parent] = remaining;
                if (remaining != 0) break;
                word = parent;
            }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int TakeFirstBit(ref ulong bits)
        {
            int index = FirstBit(bits);
            bits &= bits - 1;
            return index;
        }

        private static int WordCount(int count) => (int)(((long)count + 63) >> 6);

        // Portable to Unity/.NET Standard 2.1 without BitOperations or a cold LUT.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int FirstBit(ulong bits)
        {
            int index = 0;
            if ((bits & 0xFFFFFFFFUL) == 0) { index += 32; bits >>= 32; }
            if ((bits & 0xFFFFUL) == 0) { index += 16; bits >>= 16; }
            if ((bits & 0xFFUL) == 0) { index += 8; bits >>= 8; }
            if ((bits & 0xFUL) == 0) { index += 4; bits >>= 4; }
            if ((bits & 3UL) == 0) { index += 2; bits >>= 2; }
            return index + ((bits & 1UL) == 0 ? 1 : 0);
        }

        internal static long EstimateBufferBytes(int capacity)
        {
            long bytes = 0;
            for (int words = WordCount(capacity); words > 0; words = words == 1 ? 0 : WordCount(words))
                bytes += (long)words * sizeof(ulong);
            return bytes;
        }
    }
}
