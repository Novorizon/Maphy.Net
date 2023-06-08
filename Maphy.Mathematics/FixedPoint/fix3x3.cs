using System;
using System.Runtime.CompilerServices;


namespace Maphy.Mathematics
{
    [System.Serializable]
    public struct fix3x3 : IEquatable<fix3x3>
    {
        public fix3 c0;
        public fix3 c1;
        public fix3 c2;

        public static readonly fix3x3 identity = new fix3x3
            (
            fix._1, fix._0, fix._0,
            fix._0, fix._1, fix._0,
            fix._0, fix._0, fix._1
            );

        public static readonly fix3x3 zero = new fix3x3(
            fix._0, fix._0, fix._0,
            fix._0, fix._0, fix._0,
            fix._0, fix._0, fix._0
            );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3x3(fix3 c0, fix3 c1, fix3 c2)
        {
            this.c0 = c0;
            this.c1 = c1;
            this.c2 = c2;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3x3(fix m00, fix m01, fix m02,
                             fix m10, fix m11, fix m12,
                             fix m20, fix m21, fix m22)
        {
            this.c0 = new fix3(m00, m10, m20);
            this.c1 = new fix3(m01, m11, m21);
            this.c2 = new fix3(m02, m12, m22);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3x3(fix v)
        {
            fix3 x = new fix3(v, v, v);
            this.c0 = x;
            this.c1 = x;
            this.c2 = x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3x3(int v)
        {
            fix3 x = new fix3(v, v, v);
            this.c0 = x;
            this.c1 = x;
            this.c2 = x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3x3(float v)
        {
            fix3 x = new fix3(v, v, v);
            this.c0 = x;
            this.c1 = x;
            this.c2 = x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix3x3(fix v) { return new fix3x3(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix3x3(int v) { return new fix3x3(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix3x3(float v) { return new fix3x3(v); }

        /// <summary>
        /// componentwise multiplication, not  matrix multiplication! use mul() if you want matrix multiplication. 
        /// 分量相乘而不是矩阵乘法。矩阵乘法使用mul()
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 operator *(fix3x3 lhs, fix3x3 rhs) { return new fix3x3(lhs.c0 * rhs.c0, lhs.c1 * rhs.c1, lhs.c2 * rhs.c2); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 operator *(fix3x3 lhs, fix rhs) { return new fix3x3(lhs.c0 * rhs, lhs.c1 * rhs, lhs.c2 * rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 operator *(fix lhs, fix3x3 rhs) { return new fix3x3(lhs * rhs.c0, lhs * rhs.c1, lhs * rhs.c2); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 operator +(fix3x3 lhs, fix3x3 rhs) { return new fix3x3(lhs.c0 + rhs.c0, lhs.c1 + rhs.c1, lhs.c2 + rhs.c2); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 operator -(fix3x3 lhs, fix3x3 rhs) { return new fix3x3(lhs.c0 - rhs.c0, lhs.c1 - rhs.c1, lhs.c2 - rhs.c2); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 operator /(fix3x3 lhs, fix3x3 rhs) { return new fix3x3(lhs.c0 / rhs.c0, lhs.c1 / rhs.c1, lhs.c2 / rhs.c2); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 operator -(fix3x3 val) { return new fix3x3(-val.c0, -val.c1, -val.c2); }

        unsafe public ref fix3 this[int index]
        {
            get { fixed (fix3x3* array = &this) { return ref ((fix3*)array)[index]; } }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(fix3x3 rhs) { return c0.Equals(rhs.c0) && c1.Equals(rhs.c1) && c2.Equals(rhs.c2); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object o) { return Equals((fix3x3)o); }

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = c0.GetHashCode();
                hashCode = (hashCode * 397) ^ c1.GetHashCode();
                hashCode = (hashCode * 397) ^ c2.GetHashCode();
                return hashCode;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("fix3x3({0}f, {1}f, {2}f,  {3}f, {4}f, {5}f,  {6}f, {7}f, {8}f)", c0.x, c1.x, c2.x, c0.y, c1.y, c2.y, c0.z, c1.z, c2.z);
        }
    }

}
