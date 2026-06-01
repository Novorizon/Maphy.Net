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
        public fix(long v) { value = SafeFromInteger(v).value; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix(float v) { value = (long)(v * ONE + 0.5f * (v < 0 ? -1 : 1)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix(double v) { value = (long)(v * ONE); }

        //with raw value
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Raw(long value) { fix v; v.value = value; return v; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsNaN(fix value) { return value.value == long.MinValue; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix SafeAdd(fix a, fix b)
        {
            return fixWide.Add(a, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix SafeSub(fix a, fix b)
        {
            return fixWide.Sub(a, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix SafeMul(fix a, fix b)
        {
            return fixWide.Mul(a, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix SafeDiv(fix a, fix b)
        {
            return fixWide.Div(a, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static fix SafeFromInteger(long v)
        {
            return fixWide.FromInteger(v);
        }

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
        public static fix operator +(fix a, fix b) { return SafeAdd(a, b); }

        //int +
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(fix a, int b) { return SafeAdd(a, SafeFromInteger(b)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(int a, fix b) { return SafeAdd(SafeFromInteger(a), b); }

        //long +
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(fix a, long b) { return SafeAdd(a, SafeFromInteger(b)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(long a, fix b) { return SafeAdd(SafeFromInteger(a), b); }

        //float +
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(fix a, float b) { return SafeAdd(a, new fix(b)); }

        //float +
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator +(float a, fix b) { return SafeAdd(new fix(a), b); }

        // 负号
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator -(fix a) { return IsNaN(a) ? NaN : SafeSub(Zero, a); }

        //减
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator -(fix a, fix b) { return SafeSub(a, b); }

        //int -
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator -(fix a, int b) { return SafeSub(a, SafeFromInteger(b)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator -(int a, fix b) { return SafeSub(SafeFromInteger(a), b); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator *(fix a, fix b) { return SafeMul(a, b); }

        //int *
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator *(fix a, int b) { return SafeMul(a, SafeFromInteger(b)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator *(int a, fix b) { return SafeMul(SafeFromInteger(a), b); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator /(fix a, fix b) { return SafeDiv(a, b); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator /(fix a, int b) { return SafeDiv(a, SafeFromInteger(b)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator /(int a, fix b) { return SafeDiv(SafeFromInteger(a), b); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator %(fix a, fix b) { if (IsNaN(a) || IsNaN(b) || b.value == 0) return NaN; a.value %= b.value; return a; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator %(fix a, int b) { return a % SafeFromInteger(b); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator %(int a, fix b) { return SafeFromInteger(a) % b; }


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
        public static fix operator >>(fix x, int amount) { return IsNaN(x) ? NaN : Raw(x.value >> amount); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix operator <<(fix x, int amount)
        {
            return fixWide.ShiftLeft(x, amount);
        }
        #endregion



    }
}
