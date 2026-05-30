using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct WorldSettings
    {
        public bool enableGravity;
        public fix gravity;

        public static WorldSettings Default => new WorldSettings(true, -9.8f);

        public WorldSettings(bool enableGravity)
            : this(enableGravity, -9.8f)
        {
        }

        public WorldSettings(bool enableGravity, fix gravity)
        {
            this.enableGravity = enableGravity;
            this.gravity = gravity;
        }
    }
}