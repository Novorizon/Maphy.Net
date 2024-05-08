
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct Material
    {
        private fix density;
        private fix frictionCoefficient;
        private fix bounciness;
        public fix GetDensity() { return density; }
        public void SetDensity(fix density) { this.density= density; }
        public fix GetFrictionCoefficient() { return frictionCoefficient; }
        public void SetFrictionCoefficient(fix frictionCoefficient) { this.frictionCoefficient = frictionCoefficient; }
        public fix GetBounciness() { return bounciness; }
        public void SetBounciness(fix bounciness) { this.bounciness = bounciness; }
    }
}