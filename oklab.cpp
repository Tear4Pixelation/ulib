#include "oklab.h"

#include <cmath>

// --- sRGB transfer function ------------------------------------------------------------------------

static real srgbToLinear(real c)
{
  return c <= real(0.04045) ? c/real(12.92) : std::pow((c + real(0.055))/real(1.055), real(2.4));
}

static real linearToSrgb(real c)
{
  if(c < 0) c = 0;
  return c <= real(0.0031308) ? real(12.92)*c : real(1.055)*std::pow(c, real(1)/real(2.4)) - real(0.055);
}

// --- OKLab -----------------------------------------------------------------------------------------

// linear-light RGB, which may fall outside [0,1] when the color is out of gamut - that is exactly what
//  oklchInGamut() tests for, so this must not clamp
static void oklchToLinearRgb(const ColorOkLch& lch, real* out)
{
  real rad = lch.h * real(M_PI)/real(180);
  real a = lch.C*std::cos(rad), b = lch.C*std::sin(rad);
  real l_ = lch.L + real(0.3963377774)*a + real(0.2158037573)*b;
  real m_ = lch.L - real(0.1055613458)*a - real(0.0638541728)*b;
  real s_ = lch.L - real(0.0894841775)*a - real(1.2914855480)*b;
  real l = l_*l_*l_, m = m_*m_*m_, s = s_*s_*s_;
  out[0] =  real(4.0767416621)*l - real(3.3077115913)*m + real(0.2309699292)*s;
  out[1] = -real(1.2684380046)*l + real(2.6097574011)*m - real(0.3413193965)*s;
  out[2] = -real(0.0041960863)*l - real(0.7034186147)*m + real(1.7076147010)*s;
}

static void colorToOklab(Color c, real* Lab)
{
  real R = srgbToLinear(c.red()/real(255));
  real G = srgbToLinear(c.green()/real(255));
  real B = srgbToLinear(c.blue()/real(255));
  real l = std::cbrt(real(0.4122214708)*R + real(0.5363325363)*G + real(0.0514459929)*B);
  real m = std::cbrt(real(0.2119034982)*R + real(0.6806995451)*G + real(0.1073969566)*B);
  real s = std::cbrt(real(0.0883024619)*R + real(0.2817188376)*G + real(0.6299787005)*B);
  Lab[0] = real(0.2104542553)*l + real(0.7936177850)*m - real(0.0040720468)*s;
  Lab[1] = real(1.9779984951)*l - real(2.4285922050)*m + real(0.4505937099)*s;
  Lab[2] = real(0.0259040371)*l + real(0.7827717662)*m - real(0.8086757660)*s;
}

real oklchWrapHue(real hue)
{
  hue = std::fmod(hue, real(360));
  return hue < 0 ? hue + real(360) : hue;
}

ColorOkLch oklchFromColor(Color c)
{
  real Lab[3];
  colorToOklab(c, Lab);
  real h = std::atan2(Lab[2], Lab[1]) * real(180)/real(M_PI);
  return ColorOkLch(Lab[0], std::sqrt(Lab[1]*Lab[1] + Lab[2]*Lab[2]), oklchWrapHue(h));
}

Color oklchToColor(const ColorOkLch& lch, int alpha)
{
  real rgb[3];
  oklchToLinearRgb(lch, rgb);
  int out[3];
  for(int ii = 0; ii < 3; ++ii) {
    int v = int(real(255)*linearToSrgb(rgb[ii]) + real(0.5));
    out[ii] = v < 0 ? 0 : (v > 255 ? 255 : v);
  }
  return Color(out[0], out[1], out[2], alpha);
}

bool oklchInGamut(const ColorOkLch& lch)
{
  real rgb[3];
  oklchToLinearRgb(lch, rgb);
  for(int ii = 0; ii < 3; ++ii) {
    if(rgb[ii] < real(-1e-4) || rgb[ii] > real(1) + real(1e-4))
      return false;
  }
  return true;
}

real oklchMaxChroma(real L, real hue)
{
  real lo = 0, hi = real(0.45);
  // 28 bisections puts the answer well below the 1/255 that can be displayed
  for(int ii = 0; ii < 28; ++ii) {
    real mid = (lo + hi)/2;
    if(oklchInGamut(ColorOkLch(L, mid, hue))) lo = mid; else hi = mid;
  }
  return lo;
}

// --- the cusp table --------------------------------------------------------------------------------

// The cusp moves a long way around the wheel - yellow peaks near L=0.96, blue near L=0.49 - which is
//  precisely why a fixed lightness ramp cannot serve every hue.  It depends only on the sRGB primaries,
//  so it is the same table on every machine and can be built once.
//
// One entry per degree, linearly interpolated in between: the cusp varies smoothly with hue, so 1 degree
//  is accurate to ~1e-4, far below a displayable difference.  Interpolating (rather than rounding to the
//  nearest degree) matters because jitter puts family hues at fractional angles, and rounding would make
//  two hues a fraction of a degree apart share a cusp and so land on visibly different chroma.
struct CuspTable
{
  real L[360];
  real C[360];

  CuspTable()
  {
    for(int deg = 0; deg < 360; ++deg) {
      real bestL = real(0.5), bestC = -1;
      for(real l = real(0.05); l <= real(0.98); l += real(0.01)) {
        real c = oklchMaxChroma(l, real(deg));
        if(c > bestC) { bestC = c; bestL = l; }
      }
      // refine around the coarse peak
      real lo = std::max(real(0.02), bestL - real(0.012));
      real hi = std::min(real(0.995), bestL + real(0.012));
      for(real l = lo; l <= hi; l += real(0.0015)) {
        real c = oklchMaxChroma(l, real(deg));
        if(c > bestC) { bestC = c; bestL = l; }
      }
      L[deg] = bestL;
      C[deg] = bestC;
    }
  }
};

ColorOkLch oklchCusp(real hue)
{
  // function-local static: C++11 guarantees this is initialized exactly once and is thread safe, which
  //  matters because palettes can be generated from the document-load thread
  static const CuspTable table;

  hue = oklchWrapHue(hue);
  int lo = int(hue);
  if(lo < 0) lo = 0;
  if(lo > 359) lo = 359;
  int hi = (lo + 1) % 360;
  real f = hue - real(lo);
  return ColorOkLch(table.L[lo] + f*(table.L[hi] - table.L[lo]),
                    table.C[lo] + f*(table.C[hi] - table.C[lo]), hue);
}

// --- distance and contrast -------------------------------------------------------------------------

real oklabDeltaE(Color a, Color b)
{
  real A[3], B[3];
  colorToOklab(a, A);
  colorToOklab(b, B);
  real dL = A[0] - B[0], da = A[1] - B[1], db = A[2] - B[2];
  return std::sqrt(dL*dL + da*da + db*db);
}

real srgbRelLuminance(Color c)
{
  return real(0.2126)*srgbToLinear(c.red()/real(255))
       + real(0.7152)*srgbToLinear(c.green()/real(255))
       + real(0.0722)*srgbToLinear(c.blue()/real(255));
}

real srgbContrast(Color a, Color b)
{
  real la = srgbRelLuminance(a), lb = srgbRelLuminance(b);
  if(la < lb) std::swap(la, lb);
  return (la + real(0.05))/(lb + real(0.05));
}
