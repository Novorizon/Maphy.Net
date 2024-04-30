
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        //每个点 在法线axis上的投影点
        internal static fix2 ExtremeProjectPoint(fix3 axis, fix3[] points)
        {
            fix min = fix.Max;
            fix max = fix.Min;
            for (int i = 0; i < points.Length; i++)
            {
                fix p = math.dot(axis, points[i]);
                min = math.min(p, min);
                max = math.max(p, max);
            }
            return new fix2(min, max);
        }

        //端点 在自己法线axis上的投影点
        internal static fix2 ExtremeProjectPoint(fix3 axis, fix3 min, fix3 max)
        {
            fix a = math.dot(axis, min);
            fix b = math.dot(axis, max);
            fix MIN = math.min(a, b);
            fix MAX = math.max(a, b);

            return new fix2(MIN, MAX);
        }

        internal static bool IsOverlap(fix2 point0, fix2 point1)
        {
            if (point0.x > point1.y || point0.y < point1.x)
                return false;
            return true;
        }


        internal static bool intersectSegmentSphere(fix3 p, fix3 d, fix3 s_c, fix r, fix t)
        {
            fix tmax = math.length(d * d);
            fix3 m = p - s_c;
            fix b = math.length(m * d);
            fix c = math.length(m * m - r * r);
            if (tmax > 0.0f)
            {
                tmax = math.sqrt(tmax);
                d = d / tmax;
            }
            else
            {
                if (c > 0.0f)
                    return false;
                else
                {
                    t = 0;
                    //q = p;
                    return true;
                }
            }
            //Exit if r's origin outside s (c > 0) and r pointing away from s (b > 0)
            if (c > 0.0f && b > 0.0f) return false;
            fix discr = b * b - c;
            // A negative discriminant corresponds to ray missing sphere.
            if (discr < 0.0f) return false;
            // Ray now found to intersect sphere, compute smallest t value of
            // intersection.
            t = -b - math.sqrt(discr);
            // If t > tmax then sphere is missed.
            if (t > tmax)
                return false;
            // If t is negative then segment started inside sphere, so clamp t to
            // zero.
            if (t < 0.0f) t = 0.0f;
            // Calculate intersection point.
            //q = p + d * t;
            // Set t to the interval 0 <= t <= tmax.
            t = t / tmax;
            return true;
        }


        internal static bool intersectSegmentCapsule(fix3 sa, fix3 sb, fix3 p, fix3 q, fix r, fix t)
        {
            fix3 d = q - p, m = sa - p, n = sb - sa;
            fix md = math.length(m * d);
            fix nd = math.length(n * d);
            fix dd = math.length(d * d);
            // Test if segment fully outside either endcap of capsule.
            if (md < 0.0f && md + nd < 0.0f)
            {
                // Segment outside 'p'.
                return intersectSegmentSphere(sa, n, p, r, t);
            }
            if (md > dd && md + nd > dd)
            {
                // Segment outside 'q'
                return intersectSegmentSphere(sa, n, q, r, t);
            }
            fix nn = math.length(n * n);
            fix mn = math.length(m * n);
            fix a = dd * nn - nd * nd;
            fix k = math.length(m * m - r * r);
            fix c = dd * k - md * md;
            if (math.abs(a) < math.Epsilon)
            {
                // Segment runs parallel to cylinder axis.
                if (c > 0) return false; // 'a' and thus the segment lies outside cyl.
                                         // Now known that segment intersects cylinder. Figure out how.
                if (md < 0)
                    // Intersect against 'p' endcap.
                    intersectSegmentSphere(sa, n, p, r, t);
                else if (md > dd)
                    // Intersect against 'q' encap.
                    intersectSegmentSphere(sa, n, q, r, t);
                else t = 0.0f; // 'a' lies inside cylinder.
                return true;
            }
            fix b = dd * mn - nd * md;
            fix discr = b * b - a * c;
            if (discr < 0.0f) return false; // No real roots; no intersection.

            t = (-b - math.sqrt(discr)) / a;
            fix t0 = t;
            if (md + t * nd < 0.0f)
            {
                // Intersection outside cylinder on 'p' side;
                return intersectSegmentSphere(sa, n, p, r, t);
            }
            else if (md + t * nd > dd)
            {
                // Intsection outside cylinder on 'q' side.
                return intersectSegmentSphere(sa, n, q, r, t);
            }
            t = t0;
            // Intersection if segment intersects cylinder between end-caps.
            return t >= 0.0f && t <= 1.0f;
        }

    }
}
