
using Maphy.Mathematics;
namespace Maphy.Physics
{
    public struct WorldSettings
    {
        public bool enableGravity;
        public fix gravity;

        public WorldSettings(bool enableGravity)
        {
            this.enableGravity = enableGravity;
            gravity = -9.8;
        }
    }
}
