using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 project(fix2 a, fix2 b)
        {
            return (dot(a, b) / dot(b, b)) * b;
        }

        /// <summary>
        /// 向量a在向量b上的投影
        /// 投影=投影的模与b的模 的比例，乘以b
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 project(fix3 a, fix3 b)
        {
            return (dot(a, b) / dot(b, b)) * b;
        }

        /// <summary>
        /// 投影的模
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 projectLength(fix3 a, fix3 b)
        {
            return dot(a, b) / sqrt(dot(b, b));
        }

        /// <summary>
        /// a在b上的投影prj，投影的模与b的模的比值
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 projectRatio(fix3 a, fix3 b)
        {
            return dot(a, b) / dot(b, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 project(fix4 a, fix4 b)
        {
            return (dot(a, b) / dot(b, b)) * b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 projectonplane(fix3 a, fix3 planeNormal)
        {
            return a - dot(a, planeNormal) / dot(planeNormal, planeNormal) * planeNormal;
        }

        /// 入射向量i和法向量n，返回反射向量 r = i - 2.0f * dot(i, n) * n
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 reflect(fix2 i, fix2 n) { return i - 2f * n * dot(i, n); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 reflect(fix3 i, fix3 n) { return i - 2f * n * dot(i, n); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 reflect(fix4 i, fix4 n) { return i - 2f * n * dot(i, n); }


        /// 入射向量i、法向量n和折射率eta的折射向量.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 refract(fix2 i, fix2 n, fix eta)
        {
            fix ni = dot(n, i);
            fix k = 1.0f - eta * eta * (1.0f - ni * ni);
            return select(new fix2(0), eta * i - (eta * ni + sqrt(k)) * n, k >= 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 refract(fix3 i, fix3 n, fix eta)
        {
            fix ni = dot(n, i);
            fix k = 1.0f - eta * eta * (1.0f - ni * ni);
            return select(new fix3(0), eta * i - (eta * ni + sqrt(k)) * n, k >= 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 refract(fix4 i, fix4 n, fix eta)
        {
            fix ni = dot(n, i);
            fix k = 1 - eta * eta * (1 - ni * ni);
            return select(new fix4(0), eta * i - (eta * ni + sqrt(k)) * n, k >= 0);
        }

        /// Returns the result of clamping the value x into the interval [a, b], where x, a and b are fix values
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix clamp(fix x, fix a, fix b) { return max(a, min(b, x)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 clamp(fix2 x, fix2 a, fix2 b) { return max(a, min(b, x)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 clamp(fix3 x, fix3 a, fix3 b) { return max(a, min(b, x)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 clamp(fix4 x, fix4 a, fix4 b) { return max(a, min(b, x)); }


        /// Returns the result of clamping the fix value x into the interval [0, 1]
        /// 如果x取值小于0，则返回值为0。如果x取值大于1，则返回值为1。若x在0到1之间，则直接返回x的值
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix saturate(fix x) { return clamp(x, 0, 1); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 saturate(fix2 x) { return clamp(x, new fix2(0), new fix2(1)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 saturate(fix3 x) { return clamp(x, new fix3(0), new fix3(1)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 saturate(fix4 x) { return clamp(x, new fix4(0), new fix4(1)); }


        /// Computes a step function. Returns 1.0f when x >= y, 0.0f otherwise
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix step(fix y, fix x) { return select(0.0f, 1.0f, x >= y); }

        /// <summary>Returns a smooth Hermite interpolation between 0.0f and 1.0f when x is in [a, b].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix smoothstep(fix a, fix b, fix x)
        {
            var t = saturate((x - a) / (b - a));
            return t * t * (3 - (2 * t));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 smoothstep(fix2 a, fix2 b, fix2 x)
        {
            var t = saturate((x - a) / (b - a));
            return t * t * (3 - (2 * t));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 smoothstep(fix3 a, fix3 b, fix3 x)
        {
            var t = saturate((x - a) / (b - a));
            return t * t * (3 - (2 * t));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 smoothstep(fix4 a, fix4 b, fix4 x)
        {
            var t = saturate((x - a) / (b - a));
            return t * t * (3 - (2 * t));
        }

        /// 任意一个非0
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(fix2 x) { return x.x != 0 || x.y != 0; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(fix3 x) { return x.x != 0 || x.y != 0 || x.z != 0; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(fix4 x) { return x.x != 0 || x.y != 0 || x.z != 0 || x.w != 0; }


        /// 所有非0
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(fix2 x) { return x.x != 0 && x.y != 0; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(fix3 x) { return x.x != 0 && x.y != 0 && x.z != 0; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(fix4 x) { return x.x != 0 && x.y != 0 && x.z != 0 && x.w != 0; }


        /// Returns b if c is true, a otherwise
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix select(fix a, fix b, bool c) { return c ? b : a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 select(fix2 a, fix2 b, bool c) { return c ? b : a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 select(fix3 a, fix3 b, bool c) { return c ? b : a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 select(fix4 a, fix4 b, bool c) { return c ? b : a; }


        ///// <summary>Returns the bit pattern of a fix as an int.</summary>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static int asint(fix x)
        //{
        //    IntFixUnion u;
        //    u.intValue = 0;
        //    u.fixValue = x;
        //    return u.intValue;
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static int[] asint(fix2 x) { return new int[2] { asint(x.x), asint(x.y) }; }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static int[] asint(fix3 x) { return new int[3] { asint(x.x), asint(x.y), asint(x.z) }; }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static int[] asint(fix4 x) { return new int[4] { asint(x.x), asint(x.y), asint(x.z), asint(x.w) }; }

        ///// <summary>Returns the bit pattern of an int as a fix.</summary>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static fix asfix(int x)
        //{
        //    IntFixUnion u;
        //    u.fixValue = 0;
        //    u.intValue = x;

        //    return u.fixValue;
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static fix[] asfix(int[] x)
        //{
        //    fix[] a = new fix[x.Length];
        //    for (int i = 0; i < x.Length; i++)
        //    {
        //        a[i] = asfix(x[i]);
        //    }
        //    return a;
        //}

        ///// <summary>Returns the bit pattern of a uint as a fix.</summary>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static fix asfix(uint x) { return asfix((int)x); }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static fix[] asfix(uint[] x)
        //{
        //    fix[] a = new fix[x.Length];
        //    for (int i = 0; i < x.Length; i++)
        //    {
        //        a[i] = asfix(x[i]);
        //    }
        //    return a;
        //}

        ///// <summary>Returns the bit pattern of a fix as a uint.</summary>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static uint asuint(fix x) { return (uint)asint(x); }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static uint[] asuint(fix2 x) { return new uint[2] { asuint(x.x), asuint(x.y) }; }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static uint[] asuint(fix3 x) { return new uint[3] { asuint(x.x), asuint(x.y), asuint(x.z) }; }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static uint[] asuint(fix4 x) { return new uint[4] { asuint(x.x), asuint(x.y), asuint(x.z), asuint(x.w) }; }


    }
}
