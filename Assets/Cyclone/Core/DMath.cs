using System;
using System.Runtime.CompilerServices;

namespace Cyclone.Core
{
    public class DMath
    {

        public const double EPS = 1e-18;

        public const double PI = Math.PI;

        public const double SQRT2 = 1.414213562373095;

        public const double Rad2Deg = 180.0 / PI;

        public const double Deg2Rad = PI / 180.0;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SafeAcos(double r)
        {
            return Math.Acos(Math.Min(1.0, Math.Max(-1.0, r)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SafeAsin(double r)
        {
            return Math.Asin(Math.Min(1.0, Math.Max(-1.0, r)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SafeSqrt(double v)
        {
            if (v <= 0.0) return 0.0;
            return Math.Sqrt(v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SafeLog(double v)
        {
            if (v <= 0.0) return 0.0;
            return Math.Log(v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SafeLog10(double v)
        {
            if (v <= 0.0) return 0.0;
            return Math.Log10(v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SafeInvSqrt(double n, double d, double eps = EPS)
        {
            if (d <= 0.0) return 0.0;
            d = Math.Sqrt(d);
            if (d < eps) return 0.0;
            return n / d;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SafeInv(double v, double eps = EPS)
        {
            if (Math.Abs(v) < eps) return 0.0;
            return 1.0 / v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SafeDiv(double n, double d, double eps = EPS)
        {
            if (Math.Abs(d) < eps) return 0.0;
            return n / d;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsZero(double v, double eps = EPS)
        {
            return Math.Abs(v) < eps;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsFinite(double f)
        {
            return !(double.IsInfinity(f) || double.IsNaN(f));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Sqr(double v)
        {
            return v * v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Pow3(double v)
        {
            return v * v * v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Pow4(double v)
        {
            return v * v * v * v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Clamp(double v, double min, double max)
        {
            if (v < min) v = min;
            if (v > max) v = max;
            return v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Clamp01(double v)
        {
            if (v < 0.0) v = 0.0;
            if (v > 1.0) v = 1.0;
            return v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SmoothStep(double edge0, double edge1, double x)
        {
            double t = Clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
            return t * t * (3.0 - 2.0 * t);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Frac(double x)
        {
            return x - Math.Floor(x);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Lerp(double v0, double v1, double a)
        {
            return v0 * (1.0 - a) + v1 * a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SignOrZero(double v)
        {
            if (v == 0) return 0;
            return Math.Sign(v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Min(double a, double b, double c)
        {
            return Math.Min(a, Math.Min(b, c));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Max(double a, double b, double c)
        {
            return Math.Max(a, Math.Max(b, c));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Normalize(double a, double min, double max)
        {
            double len = max - min;
            if (len <= 0) return 0;
            return (a - min) / len;
        }

    }
}




















