using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct WorldSettings
    {
        public bool enableGravity;
        public fix gravity;
        public fix timeStep;
        public fix restitution;
        public fix restitutionVelocityThreshold;
        public fix friction;
        public fix warmStartScale;
        public fix penetrationSlop;
        public fix positionCorrectionPercent;
        public fix maxPositionCorrection;
        public fix maxLinearVelocity;
        public fix maxAngularVelocity;
        public fix maxTranslationPerStep;
        public fix maxRotationPerStep;
        public fix maxContactImpulse;
        public fix maxFrictionImpulse;
        public int solverIterations;
        public int positionIterations;
        public bool enableSleeping;
        public fix linearSleepThreshold;
        public fix angularSleepThreshold;
        public fix sleepTime;
        public bool enableCCD;
        public bool enableDynamicCCD;
        public fix ccdMinVelocity;
        public fix ccdSkin;
        public int ccdMaxIterations;
        public NarrowPhaseAlgorithm narrowPhaseAlgorithm;
        public ContactManifoldSettings contactManifoldSettings;
        public int maxSubSteps;
        public bool deferLifecycleChangesDuringCallbacks;
        public bool catchCallbackExceptions;

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
            restitutionVelocityThreshold = fix._0_5;
            friction = fix._0_5;
            warmStartScale = fix.One;
            penetrationSlop = fix._0_01;
            positionCorrectionPercent = fix._0_2;
            maxPositionCorrection = fix.Max;
            maxLinearVelocity = fix.Zero;
            maxAngularVelocity = fix.Zero;
            maxTranslationPerStep = fix.Zero;
            maxRotationPerStep = fix.Zero;
            maxContactImpulse = fix.Zero;
            maxFrictionImpulse = fix.Zero;
            solverIterations = 1;
            positionIterations = 1;
            enableSleeping = true;
            linearSleepThreshold = fix._0_01;
            angularSleepThreshold = fix._0_01;
            sleepTime = fix._0_5;
            enableCCD = false;
            enableDynamicCCD = false;
            ccdMinVelocity = fix._1;
            ccdSkin = fix.Zero;
            ccdMaxIterations = 1;
            narrowPhaseAlgorithm = NarrowPhaseAlgorithm.Auto;
            contactManifoldSettings = ContactManifoldSettings.Default;
            maxSubSteps = 8;
            deferLifecycleChangesDuringCallbacks = true;
            catchCallbackExceptions = false;
        }
    }
}
