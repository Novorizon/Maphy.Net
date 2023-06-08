using System;
using System.Runtime.CompilerServices;



namespace Maphy.Mathematics
{
    [Serializable]
    public struct fix4x4 : IEquatable<fix4x4>
    {
        public fix4 c0;
        public fix4 c1;
        public fix4 c2;
        public fix4 c3;

        public static readonly fix4x4 identity = new fix4x4(fix._1, fix._0, fix._0, fix._0, fix._0, fix._1, fix._0, fix._0, fix._0, fix._0, fix._1, fix._0, fix._0, fix._0, fix._0, fix._1);
        public static readonly fix4x4 zero = new fix4x4(fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0, fix._0);

        public fix3 right => new fix3(c0.x, c0.y, c0.z);
        public fix3 up => new fix3(c1.x, c1.y, c1.z);
        public fix3 forward => new fix3(c2.x, c2.y, c2.z);
        public fix3 position => new fix3(c3.x, c3.y, c3.z);
        public quaternion rotation => new quaternion(this);


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4x4(fix4 c0, fix4 c1, fix4 c2, fix4 c3)
        {
            this.c0 = c0;
            this.c1 = c1;
            this.c2 = c2;
            this.c3 = c3;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4x4(fix m00, fix m01, fix m02, fix m03,
                        fix m10, fix m11, fix m12, fix m13,
                        fix m20, fix m21, fix m22, fix m23,
                        fix m30, fix m31, fix m32, fix m33)
        {
            this.c0 = new fix4(m00, m10, m20, m30);
            this.c1 = new fix4(m01, m11, m21, m31);
            this.c2 = new fix4(m02, m12, m22, m32);
            this.c3 = new fix4(m03, m13, m23, m33);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4x4(fix v)
        {
            this.c0 = new fix4(v, v, v, v);
            this.c1 = new fix4(v, v, v, v);
            this.c2 = new fix4(v, v, v, v);
            this.c3 = new fix4(v, v, v, v);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4x4(int v)
        {
            fix4 x = new fix4(v);
            this.c0 = x;
            this.c1 = x;
            this.c2 = x;
            this.c3 = x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4x4(float v)
        {
            fix4 x = new fix4(v);
            this.c0 = x;
            this.c1 = x;
            this.c2 = x;
            this.c3 = x;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix4x4(fix v) { return new fix4x4(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix4x4(int v) { return new fix4x4(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix4x4(float v) { return new fix4x4(v); }



        /// <summary>
        /// componentwise multiplication, not  matrix multiplication! use mul if you want matrix multiplication
        /// 分量相乘而不是矩阵乘法。矩阵乘法使用mul()
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator *(fix4x4 lhs, fix4x4 rhs) { return new fix4x4(lhs.c0 * rhs.c0, lhs.c1 * rhs.c1, lhs.c2 * rhs.c2, lhs.c3 * rhs.c3); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator *(fix4x4 lhs, fix rhs) { return new fix4x4(lhs.c0 * rhs, lhs.c1 * rhs, lhs.c2 * rhs, lhs.c3 * rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator *(fix lhs, fix4x4 rhs) { return new fix4x4(lhs * rhs.c0, lhs * rhs.c1, lhs * rhs.c2, lhs * rhs.c3); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator +(fix4x4 lhs, fix4x4 rhs) { return new fix4x4(lhs.c0 + rhs.c0, lhs.c1 + rhs.c1, lhs.c2 + rhs.c2, lhs.c3 + rhs.c3); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator +(fix4x4 lhs, fix rhs) { return new fix4x4(lhs.c0 + rhs, lhs.c1 + rhs, lhs.c2 + rhs, lhs.c3 + rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator +(fix lhs, fix4x4 rhs) { return new fix4x4(lhs + rhs.c0, lhs + rhs.c1, lhs + rhs.c2, lhs + rhs.c3); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator -(fix4x4 lhs, fix4x4 rhs) { return new fix4x4(lhs.c0 - rhs.c0, lhs.c1 - rhs.c1, lhs.c2 - rhs.c2, lhs.c3 - rhs.c3); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator -(fix4x4 lhs, fix rhs) { return new fix4x4(lhs.c0 - rhs, lhs.c1 - rhs, lhs.c2 - rhs, lhs.c3 - rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator -(fix lhs, fix4x4 rhs) { return new fix4x4(lhs - rhs.c0, lhs - rhs.c1, lhs - rhs.c2, lhs - rhs.c3); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator /(fix4x4 lhs, fix4x4 rhs) { return new fix4x4(lhs.c0 / rhs.c0, lhs.c1 / rhs.c1, lhs.c2 / rhs.c2, lhs.c3 / rhs.c3); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator /(fix4x4 lhs, fix rhs) { return new fix4x4(lhs.c0 / rhs, lhs.c1 / rhs, lhs.c2 / rhs, lhs.c3 / rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator /(fix lhs, fix4x4 rhs) { return new fix4x4(lhs / rhs.c0, lhs / rhs.c1, lhs / rhs.c2, lhs / rhs.c3); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator %(fix4x4 lhs, fix4x4 rhs) { return new fix4x4(lhs.c0 % rhs.c0, lhs.c1 % rhs.c1, lhs.c2 % rhs.c2, lhs.c3 % rhs.c3); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator %(fix4x4 lhs, fix rhs) { return new fix4x4(lhs.c0 % rhs, lhs.c1 % rhs, lhs.c2 % rhs, lhs.c3 % rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 operator %(fix lhs, fix4x4 rhs) { return new fix4x4(lhs % rhs.c0, lhs % rhs.c1, lhs % rhs.c2, lhs % rhs.c3); }


        unsafe public ref fix4 this[int index]
        {
            get { fixed (fix4x4* array = &this) { return ref ((fix4*)array)[index]; } }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(fix4x4 rhs) { return c0.Equals(rhs.c0) && c1.Equals(rhs.c1) && c2.Equals(rhs.c2) && c3.Equals(rhs.c3); }

        public override bool Equals(object o) { return Equals((fix4x4)o); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = c0.GetHashCode();
                hashCode = (hashCode * 397) ^ c1.GetHashCode();
                hashCode = (hashCode * 397) ^ c2.GetHashCode();
                hashCode = (hashCode * 397) ^ c3.GetHashCode();
                return hashCode;
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("fix4x4({0}f, {1}f, {2}f, {3}f,  {4}f, {5}f, {6}f, {7}f,  {8}f, {9}f, {10}f, {11}f,  {12}f, {13}f, {14}f, {15}f)", c0.x, c1.x, c2.x, c3.x, c0.y, c1.y, c2.y, c3.y, c0.z, c1.z, c2.z, c3.z, c0.w, c1.w, c2.w, c3.w);
        }
    }

}
