using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public sealed partial class PhysicsWorld
    {
        private fix[] solverInverseMasses = Array.Empty<fix>();
        private fix3[] solverInverseInertias = Array.Empty<fix3>();
        private bool solverMassesPrepared;

        /// <summary>Diagnostic reference switch; excluded from simulation state hashes.</summary>
        public bool EnableSolverMassCache { get; set; } = true;

        private void PrepareSolverMasses()
        {
            solverMassesPrepared = EnableSolverMassCache && contactManifoldCount > 0;
            if (!solverMassesPrepared) return;
            // Wake propagation is complete. Type, mass, inertia and sleeping state
            // remain constant through warm start, velocity and position solving.
            for (int i = 0; i < bodyHighWaterMark; i++)
            {
                solverInverseMasses[i] = bodyAlive[i] ? bodies[i].inverseMass : fix.Zero;
                solverInverseInertias[i] = bodyAlive[i] ? bodies[i].inverseInertia : fix3.zero;
            }
        }

        private fix3 GetSolverInverseInertia(in PhysicsWorldBody body)
            => solverMassesPrepared ? solverInverseInertias[body.handle.index] : body.inverseInertia;
    }
}
