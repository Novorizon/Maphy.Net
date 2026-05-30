using Maphy.Mathematics;

namespace Maphy.Physics
{
    //可能会用来存储多个刚体
    public struct Entity
    {
        public static readonly Entity Default = new Entity(0);

        public ulong id;
        public fix3 translation;
        public quaternion orientation;
        public bool isStatic;

        public Entity(ulong id)
        {
            this.id = id;
            translation = fix3.zero;
            orientation = quaternion.identity;
            isStatic = false;
        }

        public Entity(ulong id, fix3 position, quaternion rotation, bool isStatic = false)
        {
            this.id = id;
            translation = position;
            orientation = rotation;
            this.isStatic = isStatic;
        }

        public void SetTransform(fix3 translation, quaternion orientation)
        {
            this.translation = translation;
            this.orientation = orientation;
        }
    }
}
