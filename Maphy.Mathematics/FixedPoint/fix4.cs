using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.VisualScripting.YamlDotNet.Core.Tokens;

namespace Maphy.Mathematics
{
    [StructLayout(LayoutKind.Explicit)]
    public struct fix4 : IEquatable<fix4>
    {
        public const int SIZE = 32;

        [FieldOffset(0)]
        public fix x;

        [FieldOffset(8)]
        public fix y;

        [FieldOffset(16)]
        public fix z;

        [FieldOffset(24)]
        public fix w;

        public static readonly fix4 zero;
        public static readonly fix4 one = new fix4 { x = fix._1, y = fix._1, z = fix._1, w = fix._1 };
        public static readonly fix4 one_inverse = -one;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4(fix x, fix y, fix z, fix w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4(fix2 xy, fix2 zw)
        {
            x = xy.x;
            y = xy.y;
            z = zw.x;
            w = zw.y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4(fix3 v, fix w)
        {
            x = v.x;
            y = v.y;
            z = v.z;
            this.w = w;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4(int x, int y, int z, int w)
        {
            this.x.value = (long)x << fix.PRECISION;
            this.y.value = (long)y << fix.PRECISION;
            this.z.value = (long)z << fix.PRECISION;
            this.w.value = (long)w << fix.PRECISION;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal fix4(long x, long y, long z, long w)
        {
            this.x.value = x << fix.PRECISION;
            this.y.value = y << fix.PRECISION;
            this.z.value = z << fix.PRECISION;
            this.w.value = w << fix.PRECISION;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4(float x, float y, float z, float w)
        {
            this.x.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
            this.y.value = (long)(y * fix.ONE + 0.5f * (y < 0 ? -1 : 1));
            this.z.value = (long)(z * fix.ONE + 0.5f * (z < 0 ? -1 : 1));
            this.w.value = (long)(w * fix.ONE + 0.5f * (w < 0 ? -1 : 1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4(int x)
        {
            this.x = (long)x << fix.PRECISION;
            this.y = (long)x << fix.PRECISION;
            this.z = (long)x << fix.PRECISION;
            this.w = (long)x << fix.PRECISION;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4(float x)
        {
            this.x.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
            this.y.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
            this.z.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
            this.w.value = (long)(x * fix.ONE + 0.5f * (x < 0 ? -1 : 1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix4(fix v) { return new fix4(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix4(int v) { return new fix4(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix4(float v) { return new fix4(v); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator *(fix4 lhs, fix4 rhs)
        {
            lhs.x.value = (lhs.x.value * rhs.x.value) >> fix.PRECISION;
            lhs.y.value = (lhs.y.value * rhs.y.value) >> fix.PRECISION;
            lhs.z.value = (lhs.z.value * rhs.z.value) >> fix.PRECISION;
            lhs.w.value = (lhs.w.value * rhs.w.value) >> fix.PRECISION;
            return lhs;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator *(fix4 lhs, fix rhs) { return new fix4(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator *(fix lhs, fix4 rhs) { return new fix4(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator +(fix4 lhs, fix4 rhs) { return new fix4(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator +(fix4 lhs, fix rhs) { return new fix4(lhs.x + rhs, lhs.y + rhs, lhs.z + rhs, lhs.w + rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator +(fix lhs, fix4 rhs) { return new fix4(lhs + rhs.x, lhs + rhs.y, lhs + rhs.z, lhs + rhs.w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator -(fix4 v) { return new fix4(-v.x, -v.y, -v.z, -v.w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator -(fix4 lhs, fix4 rhs) { return new fix4(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator -(fix4 lhs, fix rhs) { return new fix4(lhs.x - rhs, lhs.y - rhs, lhs.z - rhs, lhs.w - rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator -(fix lhs, fix4 rhs) { return new fix4(lhs - rhs.x, lhs - rhs.y, lhs - rhs.z, lhs - rhs.w); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator /(fix4 lhs, fix4 rhs) { return new fix4(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator /(fix4 lhs, fix rhs) { return new fix4(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator /(fix lhs, fix4 rhs) { return new fix4(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z, lhs / rhs.w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator %(fix4 lhs, fix4 rhs) { return new fix4(lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z, lhs.w % rhs.w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator %(fix4 lhs, fix rhs) { return new fix4(lhs.x % rhs, lhs.y % rhs, lhs.z % rhs, lhs.w % rhs); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 operator %(fix lhs, fix4 rhs) { return new fix4(lhs % rhs.x, lhs % rhs.y, lhs % rhs.z, lhs % rhs.w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(fix4 a, fix4 b) { return a.x.value == b.x.value && a.y.value == b.y.value && a.z.value == b.z.value && a.w.value == b.w.value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(fix4 a, fix4 b) { return a.x.value != b.x.value || a.y.value != b.y.value || a.z.value != b.z.value || a.w.value != b.w.value; }


        /// <summary>Returns a string representation of the fix4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("fix4({0}f, {1}f, {2}f, {3}f)",
                ((decimal)x).ToString("0.##########"),
                ((decimal)y).ToString("0.##########"),
                ((decimal)z).ToString("0.##########"),
                ((decimal)w).ToString("0.##########"));

        }

        public bool Equals(fix4 other)
        {
            return x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z) && w.Equals(other.w);
        }

        public override bool Equals(object obj)
        {
            return obj is fix4 other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = x.GetHashCode();
                hashCode = (hashCode * 397) ^ y.GetHashCode();
                hashCode = (hashCode * 397) ^ z.GetHashCode();
                hashCode = (hashCode * 397) ^ w.GetHashCode();
                return hashCode;
            }
        }
        public fix4 xyzw { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(x, y, z, w); } }
        public fix4 xywz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(x, y, w, z); } }
        public fix4 xzyw { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(x, z, y, w); } }
        public fix4 xzwy { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(x, z, w, y); } }
        public fix4 xwyz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(x, w, y, z); } }
        public fix4 xwzy { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(x, w, z, y); } }

        public fix4 yxzw { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(y, x, z, w); } }
        public fix4 yxwz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(x, y, w, z); } }
        public fix4 yzxw { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(y, z, x, w); } }
        public fix4 yzwx { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(y, z, w, x); } }
        public fix4 ywxz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(y, w, x, z); } }
        public fix4 ywzx { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(y, w, z, x); } }

        public fix4 zxyw { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(z, x, y, w); } }
        public fix4 zxwy { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(z, x, w, y); } }
        public fix4 zyxw { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(z, y, x, w); } }
        public fix4 zywx { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(z, y, w, x); } }
        public fix4 zwxy { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(z, w, x, y); } }
        public fix4 zwyx { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(z, w, y, x); } }

        public fix4 wxyz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(w, x, y, z); } }
        public fix4 wxzy { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(w, x, z, y); } }
        public fix4 wyxz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(w, y, x, z); } }
        public fix4 wyzx { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(w, y, z, x); } }
        public fix4 wzxy { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(w, z, x, y); } }
        public fix4 wzyx { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix4(w, z, y, x); } }
        public fix3 xyz { [MethodImpl(MethodImplOptions.AggressiveInlining)]            get { return new fix3(x, y, z); } }

        public fix4 wwww { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return new fix4(w, w, w, w); } }
        public fix4 xyzx { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return new fix4(x, y, z, x); } }
        public fix4 wwwx { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return new fix4(w, w, w, x); } }
        public fix4 yzxy { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return new fix4(y, z, x, y); } }
        public fix4 zxyy { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return new fix4(z, x, y, y); } }
        public fix4 zxyz { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return new fix4(z, x, y, z); } }
        public fix4 yzxz { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return new fix4(y, z, x, z); } }
        unsafe public fix this[int index]
        {
            get { fixed (fix4* array = &this) { return ((fix*)array)[index]; } }
            set { fixed (fix* array = &x) { array[index] = value; } }
        }
    }
}