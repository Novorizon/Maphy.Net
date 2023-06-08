using System;
using System.IO;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{

    public partial struct fix : IEquatable<fix>, IComparable<fix>
    {
        //定点数定标Q16
        public const int BITS = 64;
        public const int PRECISION = 16;
        public const int PRECISION2 = PRECISION * 2;
        public const long ONE = 1L << PRECISION;


        public static readonly fix Max = new fix((1L << (BITS - PRECISION - 1)) - 1);//可表示的最大值
        public static readonly fix Min = new fix(-(1L << (BITS - PRECISION - 1)));//可表示的最小值
        public static readonly fix Max32 = new fix(2147483648L);//32位最大值
        public static readonly fix Min32 = -Max32;//32位最小值
        public static readonly fix One = new fix(ONE);
        public static readonly fix Zero = new fix(0);
        public static readonly fix NaN = float.NaN;


        public static readonly fix _0 = 0;
        public static readonly fix _1 = 1;
        public static readonly fix _2 = 2;
        public static readonly fix _3 = 3;
        public static readonly fix _4 = 4;
        public static readonly fix _5 = 5;
        public static readonly fix _6 = 6;
        public static readonly fix _7 = 7;
        public static readonly fix _8 = 8;
        public static readonly fix _9 = 9;
        public static readonly fix _10 = 10;
        public static readonly fix _100 = 100;
        public static readonly fix _1000 = 1000;
        public static readonly fix _10000 = 10000;
        public static readonly fix _100000 = 100000;

        public static readonly fix _0_00001 = _1 / _100000;
        public static readonly fix _0_0001 = _1 / _10000;
        public static readonly fix _0_0005 = _0_0001 * 5;
        public static readonly fix _0_9995 = _1 - _0_0005;

        public static readonly fix _0_001 = _1 / _1000;
        public static readonly fix _0_002 = _0_001 * 2;
        public static readonly fix _0_003 = _0_001 * 3;
        public static readonly fix _0_004 = _0_001 * 4;
        public static readonly fix _0_005 = _0_001 * 5;

        public static readonly fix _0_01 = _1 / _100;
        public static readonly fix _0_02 = _0_01 * 2;
        public static readonly fix _0_03 = _0_01 * 3;
        public static readonly fix _0_04 = _0_01 * 4;
        public static readonly fix _0_05 = _0_01 * 5;

        public static readonly fix _0_1 = _1 / 10;
        public static readonly fix _0_2 = _0_1 * 2;
        public static readonly fix _0_25 = _1 / 4;
        public static readonly fix _0_5 = _1 / 2;
        public static readonly fix _0_75 = _1 - _0_25;
        public static readonly fix _0_95 = _1 - _0_05;
        public static readonly fix _0_99 = _1 - _0_01;

        public static readonly fix _1_01 = _1 + _0_01;
        public static readonly fix _1_1 = _1 + _0_1;
        public static readonly fix _1_5 = _1 + _0_5;

    }
}
