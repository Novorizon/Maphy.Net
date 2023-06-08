using System;
using System.Globalization;
using System.IO;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{

    /// Q16 fixed-point number.
    public partial struct fix : IEquatable<fix>, IComparable<fix>
    {
        public long value;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix(int v) { value = (long)v << PRECISION; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix(long v) { value = v << PRECISION; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix(float v) { value = (long)(v * ONE + 0.5f * (v < 0 ? -1 : 1)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix(double v) { value = (long)(v * ONE); }

        //with raw value
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Raw(long value) { fix v; v.value = value; return v; }


        //int=>  
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix(int value) { return new fix(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator int(fix value) { return (int)(value.value >> PRECISION); }

        //long=>  
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix(long value) { return new fix(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator long(fix value) { return value.value >> PRECISION; }

        //float=>  
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix(float value) { return new fix(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float(fix value) { return value.value / 65536f; }

        //double=>  
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix(double value) { return new fix(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator double(fix value) { return value.value / 65536d; }

        //decimal=>  
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fix(decimal value) { fix v; v.value = (long)(value * ONE); return v; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator decimal(fix value) { return (decimal)value.value / ONE; }


        public int CompareTo(fix other)
        {
            return value.CompareTo(other.value);
        }

        public bool Equals(fix other)
        {
            return value == other.value;
        }

        public override bool Equals(object obj)
        {
            return obj is fix other && this == other;
        }

        public override int GetHashCode()
        {
            return value.GetHashCode();
        }

        public override string ToString()
        {
            return ((decimal)this).ToString("0.##########");
        }



        #region 重载运算符

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(fix a, fix b) { a.value += b.value; return a; }

        //int +
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(fix a, int b) { a.value += (long)b << PRECISION; return a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(int a, fix b) { b.value = ((long)a << PRECISION) + b.value; return b; }

        //long +
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(fix a, long b) { a.value += b << PRECISION; return a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(long a, fix b) { b.value = (a << PRECISION) + b.value; return b; }

        //float +
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(fix a, float b) { a.value += (long)(b * ONE + 0.5f * (b < 0 ? -1 : 1)); return a; }

        //float +
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(float a, fix b) { b.value = (long)(a * ONE + 0.5f * (a < 0 ? -1 : 1)) + b.value; return b; }

        // 负号
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator -(fix a) { a.value = -a.value; return a; }

        //减
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator -(fix a, fix b) { a.value -= b.value; return a; }

        //int -
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator -(fix a, int b) { a.value -= (long)b << PRECISION; return a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator -(int a, fix b) { b.value = ((long)a << PRECISION) - b.value; return b; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator *(fix a, fix b) { a.value = (a.value * b.value) >> PRECISION; return a; }

        //int *
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator *(fix a, int b) { a.value *= b; return a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator *(int a, fix b) { b.value *= a; return b; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator /(fix a, fix b) { if (b == 0) return NaN; a.value = (a.value << PRECISION) / b.value; return a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator /(fix a, int b) { if (b == 0) return NaN; a.value /= b; return a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator /(int a, fix b) { if (b == 0) return NaN; b.value = ((long)a << PRECISION2) / b.value; return b; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator %(fix a, fix b) { a.value %= b.value; return a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator %(fix a, int b) { a.value %= (long)b << PRECISION; return a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator %(int a, fix b) { b.value = ((long)a << PRECISION) % b.value; return b; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(fix a, fix b) { return a.value < b.value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(fix a, int b) { return a.value < (long)b << PRECISION; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(int a, fix b) { return (long)a << PRECISION < b.value; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(fix a, fix b) { return a.value <= b.value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(fix a, int b) { return a.value <= (long)b << PRECISION; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(int a, fix b) { return (long)a << PRECISION <= b.value; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(fix a, fix b) { return a.value > b.value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(fix a, int b) { return a.value > (long)b << PRECISION; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(int a, fix b) { return (long)a << PRECISION > b.value; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(fix a, fix b) { return a.value >= b.value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(fix a, int b) { return a.value >= (long)b << PRECISION; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(int a, fix b) { return (long)a << PRECISION >= b.value; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(fix a, fix b) { return a.value == b.value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(fix a, int b) { return a.value == (long)b << PRECISION; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(int a, fix b) { return (long)a << PRECISION == b.value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(fix a, fix b) { return a.value != b.value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(fix a, int b) { return a.value != (long)b << PRECISION; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(int a, fix b) { return (long)a << PRECISION != b.value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator >>(fix x, int amount) { return new fix(x.value >> amount); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator <<(fix x, int amount) { return new fix(x.value << amount); }
        #endregion



    }
}
