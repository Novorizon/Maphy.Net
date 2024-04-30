
using Maphy.Mathematics;
using UnityEngine.UIElements;

namespace Maphy.Physics
{
    public struct Entity
    {
        public ulong id;
        //ÊÀ½ç×ø±ê
        public fix3 translation;
        public quaternion orientation;

        //public Rigid rigid;
        public bool isStatic;

        public Entity(ulong id)
        {
            this.id = id;
            translation = fix3.zero;
            orientation = quaternion.identity;
            this.isStatic = false;
        }

        public Entity(ulong id, fix3 position, quaternion rotation,bool isStatic=false)
        {
            this.id = id;
            translation = position;
            orientation =rotation;
            this.isStatic = isStatic;
        }
    }
}
