#include "palettegen.h"

#include <cmath>
#include <cstring>
#include <stdint.h>

bool operator==(const PaletteRecipe& a, const PaletteRecipe& b)
{
  return a.gen == b.gen && a.seedHue == b.seedHue && a.vividness == b.vividness
      && a.depth == b.depth && a.minContrast == b.minContrast && a.jitter == b.jitter
      && a.paperL == b.paperL && a.paperWarm == b.paperWarm && a.families == b.families;
}

// --- cusp-walk-1 -----------------------------------------------------------------------------------
//
// FROZEN.  Do not change anything in this section - not a constant, not a loop bound, not the order of
//  two floating point operations.  If the output changes, it is a new generator with a new id, however
//  harmless the change looks.  colortest.cpp's golden check exists because "this fix is surely harmless"
//  is exactly how version 1 stops being version 1.

namespace cuspwalk1 {

// Deterministic per-(seed, family) scatter.  Must not be a RNG: a random wobble would repaint the theme
//  on every launch, make the same document look different on two machines, and make the golden check
//  impossible.  Integer ops are all on uint32_t so this is identical on every platform.
static real hash01(real seed, int i)
{
  uint32_t x = uint32_t(int32_t(std::lround(double(seed)*1000.0))) ^ (uint32_t(i + 1) * 0x9e3779b9u);
  x *= 0x85ebca6bu;
  x ^= x >> 13;
  x *= 0xc2b2ae35u;
  x ^= x >> 16;
  return real(x) / real(4294967296.0);
}

static real clampReal(real v, real lo, real hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Paper is a low-chroma warm tint.  It is generated first because every ink color is walked *against*
//  it - the legibility floor is meaningless without knowing what the ink sits on.
static ColorOkLch paperLch(const PaletteRecipe& r)
{
  real C = real(0.022) * r.paperWarm * (r.paperL > real(0.5) ? real(1) : real(0.6));
  return ColorOkLch(r.paperL, C, 80);
}

// The rule: start at the hue's cusp, step `depth` away from the paper, then keep stepping until the ink
//  is legible on that paper.  Lightness is never chosen - it is whatever legibility forces - and chroma
//  is then whatever the gamut still allows at that lightness.
static real walkLightness(const PaletteRecipe& r, real hue, Color paper, bool darkInk)
{
  ColorOkLch cusp = oklchCusp(hue);
  real L = darkInk ? cusp.L - r.depth : cusp.L + r.depth;
  L = clampReal(L, real(0.12), real(0.95));

  real step = darkInk ? real(-0.004) : real(0.004);
  for(int guard = 0; guard < 250; ++guard) {
    Color ink = oklchToColor(ColorOkLch(L, r.vividness*oklchMaxChroma(L, hue), hue));
    if(srgbContrast(ink, paper) >= r.minContrast)
      break;
    real next = L + step;
    // refusing to go further is correct: past this the color is no longer ink.  The palette is allowed
    //  to fall short of minContrast rather than turn every hue into the same near-black.
    if(next < real(0.10) || next > real(0.97))
      break;
    L = next;
  }
  return L;
}

static void generate(const PaletteRecipe& r, Palette* out)
{
  int n = r.families < 1 ? 1 : r.families;
  ColorOkLch paperLc = paperLch(r);
  bool darkInk = r.paperL > real(0.5);

  out->recipe = r;
  out->paper = oklchToColor(paperLc);

  out->families.clear();
  out->families.reserve(n);
  for(int ii = 0; ii < n; ++ii) {
    // Family 0 is not jittered: the seed hue must actually appear in its own palette, or "this is the
    //  color you chose" is a lie and the seed control has nothing to point at.
    real jitter = ii == 0 ? real(0) : (hash01(r.seedHue, ii)*real(2) - real(1)) * r.jitter;
    real hue = oklchWrapHue(r.seedHue + real(ii)*real(360)/real(n) + jitter);

    PaletteFamily fam;
    fam.hue = hue;

    real L = walkLightness(r, hue, out->paper, darkInk);
    fam.base = oklchToColor(ColorOkLch(L, r.vividness*oklchMaxChroma(L, hue), hue));

    // The dark variant takes its chroma as the same *fraction of what is available* at its own
    //  lightness, not as a fraction of the base's chroma.  Scaling the base's chroma down a second time
    //  is what made this collapse to near-black at low vividness in the prototype.
    real dL = darkInk ? std::max(real(0.20), L - real(0.13)) : std::min(real(0.92), L + real(0.13));
    fam.dark = oklchToColor(ColorOkLch(dL, r.vividness*oklchMaxChroma(dL, hue), hue));

    // The highlighter is tuned to sit *under* text rather than over bare paper, so it stays near the
    //  cusp (bright) with its chroma capped, and carries alpha.
    ColorOkLch cusp = oklchCusp(hue);
    real hL = darkInk ? std::min(real(0.93), cusp.L + real(0.10))
                      : std::max(real(0.25), cusp.L - real(0.18));
    real hC = std::min(oklchMaxChroma(hL, hue), real(0.13)) * r.vividness;
    fam.hl = oklchToColor(ColorOkLch(hL, hC, hue), 97);

    out->families.push_back(fam);
  }

  // Ruling is derived from the paper, not from the ink: it is part of the page, and a rule line that
  //  tracked an ink color would fight whatever was written on it.
  real ruleL = darkInk ? std::max(real(0.55), r.paperL - real(0.30))
                       : std::min(real(0.80), r.paperL + real(0.22));
  out->rule = oklchToColor(ColorOkLch(ruleL, real(0.045), 250), 0x9F);

  // Accents are families, not new colors - a bookmark that did not belong to the palette would be the
  //  one thing on the page that ignored the theme.
  out->bookmark = out->families[(n/4) % n].base;
  out->link     = out->families[(n/2) % n].base;
  out->selection= out->families[0].base;
  out->neutral  = darkInk ? Color(Color::BLACK) : Color(Color::WHITE);
}

}  // namespace cuspwalk1

// --- registry --------------------------------------------------------------------------------------

static const PaletteGenDef s_paletteGens[] = {
  { "cusp-walk-1", &cuspwalk1::generate },
};

static const int s_numPaletteGens = int(sizeof(s_paletteGens)/sizeof(s_paletteGens[0]));

const PaletteGenDef* paletteGenById(const char* id)
{
  if(!id || !id[0])
    return NULL;
  for(int ii = 0; ii < s_numPaletteGens; ++ii) {
    if(strcmp(s_paletteGens[ii].id, id) == 0)
      return &s_paletteGens[ii];
  }
  return NULL;
}

// the newest generator is the last entry, so adding one is appending a line
const PaletteGenDef* defaultPaletteGen() { return &s_paletteGens[s_numPaletteGens - 1]; }
int paletteGenCount() { return s_numPaletteGens; }
const PaletteGenDef* paletteGenByIndex(int i)
  { return i >= 0 && i < s_numPaletteGens ? &s_paletteGens[i] : NULL; }

bool generatePalette(const PaletteRecipe& recipe, Palette* out)
{
  const PaletteGenDef* def = paletteGenById(recipe.gen.c_str());
  bool known = def != NULL;
  if(!known)
    def = defaultPaletteGen();
  def->generate(recipe, out);
  // report the recipe as it was asked for, so an unknown id round-trips back to the file unchanged
  //  rather than being silently rewritten to whatever this build happens to have
  out->recipe = recipe;
  return known || recipe.gen.empty();
}

// --- reference palette -----------------------------------------------------------------------------

// Frozen along with cusp-walk-1: __inkbase values in every document on disk are quoted against this.
//
// EVERY field is set explicitly, deliberately.  Taking the struct's defaults would tie the reference
//  palette to whatever the *current* build considers a default recipe, so changing a default - the
//  family count, say - would silently redefine what every stored __inkbase means.  This is a frozen
//  constant that happens to be written as a recipe, not a recipe that happens to be constant.
const Palette& referencePalette()
{
  static Palette ref;
  static bool built = false;
  if(!built) {
    PaletteRecipe r;
    r.gen = "cusp-walk-1";
    r.seedHue = 0;
    r.vividness = 1.0;
    r.depth = 0.10;
    r.minContrast = 3.0;
    r.jitter = 11;
    r.paperL = 0.99;
    r.paperWarm = 0.35;
    r.families = 12;
    generatePalette(r, &ref);
    built = true;
  }
  return ref;
}

// --- snapping and matching -------------------------------------------------------------------------

Color Palette::nearest(Color c, int* familyOut, int* variantOut) const
{
  Color best = neutral;
  real bestDist = oklabDeltaE(c, neutral);
  if(familyOut) *familyOut = -1;
  if(variantOut) *variantOut = PALETTE_BASE;

  for(size_t ii = 0; ii < families.size(); ++ii) {
    for(int vv = 0; vv < PALETTE_NUM_VARIANTS; ++vv) {
      Color cand = families[ii].variant(vv);
      real dist = oklabDeltaE(c, cand);
      // strict <, so ties go to the first candidate and the result is stable
      if(dist < bestDist) {
        bestDist = dist;
        best = cand;
        if(familyOut) *familyOut = int(ii);
        if(variantOut) *variantOut = vv;
      }
    }
  }
  // an exact palette member has distance 0 to itself, so nearest(nearest(c)) == nearest(c)
  return best;
}

bool Palette::indexOf(Color c, int* familyOut, int* variantOut) const
{
  // alpha is the tool's business, not the palette's - a marker stroke is its family at another alpha
  Color opaque = c.opaque();
  if(opaque == neutral.opaque()) {
    if(familyOut) *familyOut = -1;
    if(variantOut) *variantOut = PALETTE_BASE;
    return true;
  }
  for(size_t ii = 0; ii < families.size(); ++ii) {
    for(int vv = 0; vv < PALETTE_NUM_VARIANTS; ++vv) {
      if(families[ii].variant(vv).opaque() == opaque) {
        if(familyOut) *familyOut = int(ii);
        if(variantOut) *variantOut = vv;
        return true;
      }
    }
  }
  return false;
}

Color Palette::entry(int family, int variant) const
{
  if(family < 0 || families.empty())
    return neutral;
  if(family >= int(families.size()))
    family = int(families.size()) - 1;
  return families[family].variant(variant);
}

bool Palette::mapFrom(const Palette& from, Color c, Color* out) const
{
  int family = 0, variant = PALETTE_BASE;
  if(!from.indexOf(c, &family, &variant))
    return false;   // not one of `from`'s colors: leave it exactly as it is
  if(family < 0) {
    // the neutral maps to the neutral, which is how black-on-white survives a theme change - it is
    //  exempt from the walk in both palettes, so "map by ordinal" would be wrong here
    *out = neutral;
  }
  else {
    // Ordinal, proportional when the counts differ - the same rule as matchReference(), and for the
    //  same reason: hue proximity collapses two families onto one once both palettes are jittered.
    size_t nfrom = from.families.size();
    size_t idx = nfrom ? (size_t(family)*families.size() + nfrom/2) / nfrom : 0;
    if(idx >= families.size())
      idx = families.empty() ? 0 : families.size() - 1;
    *out = entry(int(idx), variant);
  }
  out->setAlpha(c.alpha());
  return true;
}

bool Palette::matchReference(Color inkbase, int variant, Color* out) const
{
  if(families.empty())
    return false;
  // Identify which reference family this __inkbase came from, then take the family in the same ORDINAL
  //  position here.  Both steps matter:
  //
  //  - Matching on hue proximity instead looks natural and is wrong.  Both palettes carry up to
  //    +/-jitter of scatter, so two offsets can differ by 2*jitter (22 degrees at the default) against a
  //    spacing of only 30 - more than half a slot.  Two reference families then land on one target
  //    family and a twelve-color document quietly becomes an eleven-color one.  Measured: 5 collisions
  //    on absolute hue, still 2 on seed-relative hue.  Ordinal position is injective by construction.
  //
  //  - The ordinal is *derived from the color*, never stored.  A stored index is the failure shapeId was
  //    bitten by twice: "family 5" stops meaning the same thing the moment the count or the spacing rule
  //    changes.  Deriving it means a future generator can space hues however it likes.
  //
  // Within the reference palette the hue lookup is exact, since a genuine __inkbase *is* one of these
  //  colors; a garbage value degrades to its nearest neighbour rather than to family 0.
  const Palette& ref = referencePalette();
  if(ref.families.empty())
    return false;
  real hue = oklchFromColor(inkbase).h;
  size_t refIdx = 0;
  real bestDelta = real(1e9);
  for(size_t ii = 0; ii < ref.families.size(); ++ii) {
    real d = std::fabs(oklchWrapHue(ref.families[ii].hue - hue + real(180)) - real(180));
    if(d < bestDelta) { bestDelta = d; refIdx = ii; }
  }
  // proportional when the counts differ, the identity when they match
  size_t best = (refIdx*families.size() + ref.families.size()/2) / ref.families.size();
  if(best >= families.size())
    best = families.size() - 1;
  if(variant < 0 || variant >= PALETTE_NUM_VARIANTS)
    variant = PALETTE_BASE;
  *out = families[best].variant(variant);
  return true;
}
