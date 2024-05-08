
using Maphy.Mathematics;
namespace Maphy.Physics
{
    public struct WorldSettings
    {
        public bool enableGravity;
        public fix gravity;

        /// Velocity threshold for contact velocity restitution
        public fix restitutionVelocityThreshold;

        public WorldSettings(bool enableGravity)
        {
            this.enableGravity = enableGravity;
            gravity = -9.8;
            restitutionVelocityThreshold = fix._0_03;
        }
    }
}
