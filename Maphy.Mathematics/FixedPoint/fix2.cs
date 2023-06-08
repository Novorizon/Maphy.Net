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
    public struct fix2 : IEquatable<fix2>
    {
        public static readonly fix2 left = new fix2(-1, 0);
        public static readonly fix2 right = new fix2(1, 0);
        public static readonly fix2 up = new fix2(0, 1);
        public static readonly fix2 down = new fix2(0, -1);
        public static readonly fix2 one = new fix2(1, 1);
        public static readonly fix2 minus_one = new fix2(-1, -1);
        public static readonly fix2 zero = new fix2(0, 0);

        [FieldOffset(0)]
        public fix x;

        [FieldOffset(8)]
        public fix y;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix2(fix x, fix y)
        {
            this.x.value = x.value;
            this.y.value = y.value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix2(long x, long y)
        {
            this.x.value = x << fix.PRECISION;
            this.y.value = y << fix.PRECISION;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix2(int x, int y)
        {
            this.x.value = (long)x << fix.PRECISION;
            this.y.value = (long)y << fix.PRECISION;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix2(float x, float y)
        {
            this.x = x;
            this.y = y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix2(int x)
        {
            this.x.value = (long)x << fix.PRECISION;
            this.y.value = (long)x << fix.PRECISION;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix2(float x)
        {
            this.x.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
            this.y.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix2(fix v) { return new fix2(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix2(int v) { return new fix2(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix2(float v) { return new fix2(v); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator +(fix2 a, fix2 b)
        {
            a.x.value = a.x.value + b.x.value;
            a.y.value = a.y.value + b.y.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator -(fix2 a, fix2 b)
        {
            a.x.value = a.x.value - b.x.value;
            a.y.value = a.y.value - b.y.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator -(fix2 a, fix b)
        {
            a.x.value = a.x.value - b.value;
            a.y.value = a.y.value - b.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator -(fix a, fix2 b)
        {
            b.x.value = a.value - b.x.value;
            b.y.value = a.value - b.y.value;

            return b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator -(fix2 a)
        {
            a.x.value = -a.x.value;
            a.y.value = -a.y.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator *(fix2 a, fix2 b)
        {
            a.x.value = (a.x.value * b.x.value) >> fix.PRECISION;
            a.y.value = (a.y.value * b.y.value) >> fix.PRECISION;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator *(fix2 a, fix b)
        {
            a.x.value = (a.x.value * b.value) >> fix.PRECISION;
            a.y.value = (a.y.value * b.value) >> fix.PRECISION;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator *(fix b, fix2 a)
        {
            a.x.value = (a.x.value * b.value) >> fix.PRECISION;
            a.y.value = (a.y.value * b.value) >> fix.PRECISION;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator /(fix2 a, fix2 b)
        {
            a.x.value = (a.x.value << fix.PRECISION) / b.x.value;
            a.y.value = (a.y.value << fix.PRECISION) / b.y.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator /(fix2 a, fix b)
        {
            a.x.value = (a.x.value << fix.PRECISION) / b.value;
            a.y.value = (a.y.value << fix.PRECISION) / b.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 operator /(fix b, fix2 a)
        {
            a.x.value = (a.x.value << fix.PRECISION) / b.value;
            a.y.value = (a.y.value << fix.PRECISION) / b.value;

            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(fix2 a, fix2 b)
        {
            return a.x.value == b.x.value && a.y.value == b.y.value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(fix2 a, fix2 b)
        {
            return a.x.value != b.x.value || a.y.value != b.y.value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool[] operator >=(fix2 lhs, fix2 rhs) { return new bool[2] { lhs.x >= rhs.x, lhs.y >= rhs.y }; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool[] operator <=(fix2 lhs, fix2 rhs) { return new bool[2] { lhs.x <= rhs.x, lhs.y <= rhs.y }; }

        unsafe public fix this[int index]
        {
            get { fixed (fix2* array = &this) { return ((fix*)array)[index]; } }
            set { fixed (fix* array = &x) { array[index] = value; } }
        }

        public override bool Equals(object obj)
        {
            return obj is fix2 other && this == other;
        }

        public bool Equals(fix2 other)
        {
            return this == other;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return (x.GetHashCode() * 397) ^ y.GetHashCode();
            }
        }

        public override string ToString()
        {
            return $"({x}, {y})";
        }

        public class EqualityComparer : IEqualityComparer<fix2>
        {
            public static readonly EqualityComparer instance = new EqualityComparer();

            private EqualityComparer() { }

            bool IEqualityComparer<fix2>.Equals(fix2 x, fix2 y)
            {
                return x == y;
            }

            int IEqualityComparer<fix2>.GetHashCode(fix2 obj)
            {
                return obj.GetHashCode();
            }

        }
    }
}