
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    [Serializable]
    public class Box
    {
        public static readonly int EDGE = 12;
        public static readonly int FACE = 6;
        public static readonly int VERTEX = 8;
        public static readonly int NORMAL = 3;

        public fix3 Center { get; private set; }//ÖÐÐÄµã
        public fix3 Size { get; private set; }
        public fix3 BevelRadius { get; private set; }//Ð±°ë¾¶
        public fix3 Min { get; private set; }
        public fix3 Max { get; private set; }

        public ulong Id { get => throw new System.NotImplementedException(); set => throw new System.NotImplementedException(); }
        public Material material { get => throw new System.NotImplementedException(); set => throw new System.NotImplementedException(); }


        public Box(fix3 center, fix3 size)
        {
            Center = center;
            Size= size;
            BevelRadius = size / 2;
            Min = center - size / 2;
            Max = Min + size;
        }


    }
}

