using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Maphy.Mathematics
{

    [Serializable]
    [StructLayout(LayoutKind.Explicit)]
    public struct fix3 : IEquatable<fix3>
    {
        public static readonly fix3 left = new fix3(-1, 0, 0);
        public static readonly fix3 right = new fix3(1, 0, 0);
        public static readonly fix3 up = new fix3(0, 1, 0);
        public static readonly fix3 down = new fix3(0, -1, 0);
        public static readonly fix3 forward = new fix3(0, 0, 1);
        public static readonly fix3 backward = new fix3(0, 0, -1);
        public static readonly fix3 one = new fix3(1, 1, 1);
        public static readonly fix3 one_inverse = new fix3(-1, -1, -1);
        public static readonly fix3 zero = new fix3(0, 0, 0);
        public static readonly fix3 MaxValue = new fix3(2147483648L);
        public static readonly fix3 MinValue = -MaxValue;
        public static readonly fix3 NaN = new fix3(float.NaN);

        [FieldOffset(0)]
        public fix x;

        [FieldOffset(8)]
        public fix y;

        [FieldOffset(16)]
        public fix z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3(fix x, fix y, fix z)
        {
            this.x.value = x.value;
            this.y.value = y.value;
            this.z.value = z.value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3(int x, int y, int z)
        {
            this.x.value = (long)x << fix.PRECISION;
            this.y.value = (long)y << fix.PRECISION;
            this.z.value = (long)z << fix.PRECISION;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3(long x, long y, long z)
        {
            this.x.value = x << fix.PRECISION;
            this.y.value = y << fix.PRECISION;
            this.z.value = z << fix.PRECISION;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3(int x)
        {
            this.x.value = (long)x << fix.PRECISION;
            this.y.value = (long)x << fix.PRECISION;
            this.z.value = (long)x << fix.PRECISION;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3(float x)
        {
            this.x.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
            this.y.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
            this.z.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix3(fix v) { return new fix3(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix3(int v) { return new fix3(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix3(float v) { return new fix3(v); }


        #region 重载运算符
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator +(fix3 a, fix3 b)
        {
            a.x.value = a.x.value + b.x.value;
            a.y.value = a.y.value + b.y.value;
            a.z.value = a.z.value + b.z.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator -(fix3 a, fix3 b)
        {
            a.x.value = a.x.value - b.x.value;
            a.y.value = a.y.value - b.y.value;
            a.z.value = a.z.value - b.z.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator -(fix3 a, fix b)
        {
            a.x.value = a.x.value - b.value;
            a.y.value = a.y.value - b.value;
            a.z.value = a.z.value - b.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator -(fix a, fix3 b)
        {
            b.x.value = a.value - b.x.value;
            b.y.value = a.value - b.y.value;
            b.z.value = a.value - b.z.value;

            return b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator -(fix3 a)
        {
            a.x.value = -a.x.value;
            a.y.value = -a.y.value;
            a.z.value = -a.z.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator *(fix3 a, fix3 b)
        {
            a.x.value = (a.x.value * b.x.value) >> fix.PRECISION;
            a.y.value = (a.y.value * b.y.value) >> fix.PRECISION;
            a.z.value = (a.z.value * b.z.value) >> fix.PRECISION;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator *(fix3 a, fix b)
        {
            a.x.value = (a.x.value * b.value) >> fix.PRECISION;
            a.y.value = (a.y.value * b.value) >> fix.PRECISION;
            a.z.value = (a.z.value * b.value) >> fix.PRECISION;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator *(fix b, fix3 a)
        {
            a.x.value = (a.x.value * b.value) >> fix.PRECISION;
            a.y.value = (a.y.value * b.value) >> fix.PRECISION;
            a.z.value = (a.z.value * b.value) >> fix.PRECISION;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator /(fix3 a, fix3 b)
        {
            a.x.value = (a.x.value << fix.PRECISION) / b.x.value;
            a.y.value = (a.y.value << fix.PRECISION) / b.y.value;
            a.z.value = (a.z.value << fix.PRECISION) / b.z.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator /(fix3 a, fix b)
        {
            a.x.value = (a.x.value << fix.PRECISION) / b.value;
            a.y.value = (a.y.value << fix.PRECISION) / b.value;
            a.z.value = (a.z.value << fix.PRECISION) / b.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator /(fix b, fix3 a)
        {
            a.x.value = (a.x.value << fix.PRECISION) / b.value;
            a.y.value = (a.y.value << fix.PRECISION) / b.value;
            a.z.value = (a.z.value << fix.PRECISION) / b.value;

            return a;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(fix3 a, fix3 b)
        {
            return a.x.value == b.x.value && a.y.value == b.y.value && a.z.value == b.z.value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(fix3 a, fix3 b)
        {
            return a.x.value != b.x.value || a.y.value != b.y.value || a.z.value != b.z.value;
        }

        #endregion

        public bool Equals(fix3 other)
        {
            return x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);
        }

        public override bool Equals(object obj)
        {
            return obj is fix3 other && this == other;
        }


        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = x.GetHashCode();
                hashCode = (hashCode * 397) ^ y.GetHashCode();
                hashCode = (hashCode * 397) ^ z.GetHashCode();
                return hashCode;
            }
        }

        public override string ToString()
        {
            return $"({x}, {y}, {z})";
        }

        public class EqualityComparer : IEqualityComparer<fix3>
        {
            public static readonly EqualityComparer instance = new EqualityComparer();

            private EqualityComparer() { }

            bool IEqualityComparer<fix3>.Equals(fix3 x, fix3 y) { return x == y; }

            int IEqualityComparer<fix3>.GetHashCode(fix3 obj) { return obj.GetHashCode(); }
        }

        public fix3 xyz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix3(x, y, z); } }
        public fix3 xzy { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix3(x, z, y); } }
        public fix3 yxz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix3(y, x, z); } }
        public fix3 yzx { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix3(y, z, x); } }
        public fix3 zxy { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix3(z, x, y); } }
        public fix3 zyx { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix3(z, y, x); } }

        public fix4 yxxy { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(y, x, x, y); } }
        public fix4 zzyz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(z, z, y, z); } }

        unsafe public fix this[int index]
        {
            get { fixed (fix3* array = &this) { return ((fix*)array)[index]; } }
            set { fixed (fix* array = &x) { array[index] = value; } }
        }
    }
}
