#pragma once

// OKLab / OKLCh color math and the sRGB gamut boundary (COLORS_SPEC.md §2).
//
// OKLCh is the only common space where "same lightness" and "same hue" actually look like it, which is
//  the premise the palette generator rests on.  None of this belongs on Color: it needs floats, it has to
//  represent out-of-gamut values during intermediate steps, and Color is used in the render hot path
//  where neither is wanted.
//
// Matrices are Bjorn Ottosson's (https://bottosson.github.io/posts/oklab/).

#include "color.h"

struct ColorOkLch
{
  real L = 0;   // 0 - 1
  real C = 0;   // 0 - ~0.33 in sRGB
  real h = 0;   // degrees, 0 - 360

  ColorOkLch(real _L = 0, real _C = 0, real _h = 0) : L(_L), C(_C), h(_h) {}
};

// conversions; alpha is not part of OKLCh and is passed through separately by callers
ColorOkLch oklchFromColor(Color c);
// out-of-gamut components are clamped, so this always returns a displayable color
Color oklchToColor(const ColorOkLch& lch, int alpha = 255);

// is (L,C,h) inside sRGB?
bool oklchInGamut(const ColorOkLch& lch);
// the gamut boundary: largest chroma this hue can hold at this lightness
real oklchMaxChroma(real L, real hue);
// the cusp: the lightness at which this hue is most saturated, and that chroma.  Table-backed (see
//  oklab.cpp) because a naive search is far too slow to call per family per keystroke.
ColorOkLch oklchCusp(real hue);

// perceptual distance, for snapping a color to the nearest palette entry
real oklabDeltaE(Color a, Color b);

// WCAG relative luminance and contrast ratio.  Used as the legibility floor - at its cusp most of the
//  hue wheel is unreadable as ink on white paper (yellow is 1.10:1), so this is what drives the walk.
real srgbRelLuminance(Color c);
real srgbContrast(Color a, Color b);

// wrap to [0,360)
real oklchWrapHue(real hue);
