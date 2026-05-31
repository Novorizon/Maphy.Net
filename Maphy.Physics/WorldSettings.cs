using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct WorldSettings
    {
        public bool enableGravity;
        public fix gravity;
        public fix timeStep;
        public fix restitution;
        public fix friction;
        public fix penetrationSlop;
        public fix positionCorrectionPercent;
        public int solverIterations;

        public static WorldSettings Default => new WorldSettings(true, -9.8f);

        public WorldSettings(bool enableGravity)
            : this(enableGravity, -9.8f)
        {
        }

        public WorldSettings(bool enableGravity, fix gravity)
        {
            this.enableGravity = enableGravity;
            this.gravity = gravity;
            timeStep = fix.One / 60;
            restitution = fix.Zero;
            friction = fix._0_5;
            penetrationSlop = fix._0_01;
            positionCorrectionPercent = fix._0_2;
            solverIterations = 1;
        }
    }
}
