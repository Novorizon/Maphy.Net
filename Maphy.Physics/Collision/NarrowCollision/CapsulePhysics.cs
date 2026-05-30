using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap(Capsule capsule, fix3 point)
        {
            fix3 closestPoint = GetClosestPointOnSegment(capsule.Center1, capsule.Center2, point);
            return math.distancesq(closestPoint, point) <= capsule.Radius2;
        }

        internal static bool IsOverlap(Capsule a, Capsule b)
        {
            GetClosestPointsBetweenSegments(
                a.Center1,
                a.Center2,
                b.Center1,
                b.Center2,
                out fix3 closestA,
                out fix3 closestB);

            fix radius = a.Radius + b.Radius;
            return math.distancesq(closestA, closestB) <= radius * radius;
        }
    }
}
