using Maphy.Mathematics;

namespace Maphy.Physics
{
    /// <summary>
    /// Small settings block for the no-GC core. It intentionally does not expose
    /// callback or collection behavior; those belong in the object-friendly World layer.
    /// </summary>
    public struct PhysicsWorldSettings
    {
        public bool enableGravity;
        public fix3 gravity;
        public fix timeStep;
        public int maxSubSteps;
        public fix restitution;
        public fix restitutionVelocityThreshold;
        public fix friction;
        public fix warmStartScale;
        public fix penetrationSlop;
        public fix positionCorrectionPercent;
        public fix maxPositionCorrection;
        public fix maxContactImpulse;
        public fix maxFrictionImpulse;
        public fix contactVelocityBiasFactor;
        public fix maxContactBiasVelocity;
        public fix maxLinearVelocity;
        public fix maxAngularVelocity;
        public fix maxTranslationPerStep;
        public fix maxRotationPerStep;
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

        public PhysicsWorldSettings(bool enableGravity)
        {
            this.enableGravity = enableGravity;
            gravity = new fix3(fix.Zero, -new fix(10), fix.Zero);
            timeStep = fix.One / 60;
            maxSubSteps = 4;
            restitution = fix.Zero;
            restitutionVelocityThreshold = fix._0_5;
            friction = fix._0_5;
            warmStartScale = fix.One;
            penetrationSlop = fix._0_01;
            positionCorrectionPercent = fix._0_2;
            maxPositionCorrection = fix.Max;
            maxContactImpulse = fix.Zero;
            maxFrictionImpulse = fix.Zero;
            contactVelocityBiasFactor = fix.Zero;
            maxContactBiasVelocity = fix.Zero;
            maxLinearVelocity = fix.Zero;
            maxAngularVelocity = fix.Zero;
            maxTranslationPerStep = fix.Zero;
            maxRotationPerStep = fix.Zero;
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
        }

        public static PhysicsWorldSettings Default => new PhysicsWorldSettings(true);
    }
}
