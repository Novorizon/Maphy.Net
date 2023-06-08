using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class math
    {
        public static readonly fix PI = fix.Raw(205887L);
        public static readonly fix PI2 = PI * 2;
        public static readonly fix PI2rcp = 1 / PI2;
        public static readonly fix Deg2Rad = fix.Raw(1144L);
        public static readonly fix Rad2Deg = fix.Raw(3754936L);


        public static readonly fix Ln2 = fix.Raw(45426);
        public static readonly fix Ln10 = 2.3025850929940456840179914547m;
        public static readonly fix Lnr = 0.2002433314278771112016301167m;
        public static readonly fix Epsilon = fix.Raw(1);
        public static readonly fix E = fix.Raw(178145L);

        /// Extrinsic rotation order. Specifies in which order rotations around the principal axes (x, y and z) are to be applied
        public enum RotationOrder : byte
        {
            XYZ,
            XZY,
            YXZ,
            YZX,
            ZXY,
            ZYX,
            Default = ZXY
        };
    }
}
