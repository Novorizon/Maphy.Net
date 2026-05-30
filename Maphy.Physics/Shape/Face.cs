using System;

namespace Maphy.Physics
{
    [Flags]
    public enum Face
    {
        None = 0,
        Left = 1,
        Right = 1 << 1,
        Top = 1 << 2,
        Bottom = 1 << 3,
        Front = 1 << 4,
        Back = 1 << 5,
    }
}