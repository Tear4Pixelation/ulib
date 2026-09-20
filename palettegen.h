#pragma once

// Themed palette generation (COLORS_SPEC.md).
//
// A theme is a PaletteRecipe - about eight numbers and a generator id - from which a Palette is
//  *generated*.  A document stores the recipe, never the resulting colors; strokes store their literal
//  color, so the palette governs what the picker offers and never what is already drawn.
//
// The generator id is a STRING, not an index (COLORS_SPEC.md §3.1).  Shapes learned this twice: an index
//  silently reassigns meaning the moment the table changes, and here that would repaint a document.
//
// Shipped generators are frozen - their output for a given recipe must never change - and are never
//  deleted.  Each is ~80 lines of dependency-free math; keeping several forever costs nothing next to a
//  document opening in different colors than it was written in.  colortest.cpp's golden check is what
//  actually enforces this.

#include <string>
#include <vector>

#include "oklab.h"

enum PaletteVariant { PALETTE_BASE = 0, PALETTE_DARK = 1, PALETTE_HL = 2, PALETTE_NUM_VARIANTS = 3 };

struct PaletteRecipe
{
  // generator id; empty means "use the current default"
  std::string gen;
  real seedHue    = 218;    // degrees - the ONLY thing taken from a seed color
  real vividness  = 1.0;    // fraction of each hue's own max chroma, never an absolute chroma
  real depth      = 0.10;   // how far below the cusp to start; the neon-to-ink knob
  real minContrast= 3.0;    // legibility floor against this theme's own paper
  real jitter     = 11;     // deterministic hue scatter, degrees
  real paperL     = 0.99;
  real paperWarm  = 0.35;
  // 12, and a reduction to 8 was tried and reverted.  Fewer families do separate better on LIGHT
  //  paper (min pairwise dE 0.055 vs 0.041), and the plan was to restore the picker's richness from
  //  each family's dark variant.  On DARK paper that inverts: 12 bases measure 0.066 while 8 families
  //  with their darks measure 0.014, because the walk has already pushed ink up toward light and
  //  `dL = min(0.92, L + 0.13)` then clamps a dark on top of its own base.  Hue variety is also what
  //  a note-taking palette is actually asked for - red AND blue AND green - which shades do not give.
  int  families   = 12;

  friend bool operator==(const PaletteRecipe& a, const PaletteRecipe& b);
  friend bool operator!=(const PaletteRecipe& a, const PaletteRecipe& b) { return !(a == b); }
};

struct PaletteFamily
{
  Color base;
  Color dark;
  Color hl;     // highlighter - translucent, so it carries alpha
  real hue = 0;

  Color variant(int which) const
    { return which == PALETTE_DARK ? dark : (which == PALETTE_HL ? hl : base); }
};

struct Palette
{
  PaletteRecipe recipe;
  Color paper;
  Color rule;
  Color bookmark;
  Color link;
  Color selection;
  // Black on white is not a theme decision (COLORS_SPEC.md §10.8).  This family is exempt from the walk
  //  and is plain black on light paper, plain white on dark.
  Color neutral;
  std::vector<PaletteFamily> families;

  bool isDarkPaper() const { return recipe.paperL <= 0.5; }

  // nearest entry by OKLab distance - the snap the pickers apply.  Idempotent by construction: the
  //  candidate set is fixed and an exact member has distance 0, with ties broken by first index.
  Color nearest(Color c, int* familyOut = NULL, int* variantOut = NULL) const;

  // Restyling: find the entry in *this* palette corresponding to a stroke's recorded __inkbase, by
  //  matching hue against the reference palette's families.  Returns false if nothing matches, in which
  //  case the caller must leave the stroke's literal color alone.
  bool matchReference(Color inkbase, int variant, Color* out) const;

  // Exact lookup: is this color one of ours, and if so which slot?  familyOut is -1 for the neutral.
  //  Alpha is ignored, so a translucent marker stroke still matches its family.
  bool indexOf(Color c, int* familyOut, int* variantOut) const;
  // The color at a slot; family < 0 means the neutral.
  Color entry(int family, int variant) const;

  // Restyle mapping (COLORS_SPEC.md §7): given a color drawn under `from`, return the corresponding
  //  color in this palette.  Returns false when the color is not one of `from`'s - an imported PDF, a
  //  deliberately off-palette stroke, or ink from a theme older than `from` - in which case the caller
  //  must leave it alone.  Alpha is carried across, never remapped.
  bool mapFrom(const Palette& from, Color c, Color* out) const;
};

struct PaletteGenDef
{
  const char* id;
  void (*generate)(const PaletteRecipe& recipe, Palette* out);
};

// NULL if the id is unknown - which is not an error: a document written by a newer version still renders
//  correctly from its literal stroke colors, it just cannot be restyled until the app is upgraded
const PaletteGenDef* paletteGenById(const char* id);
const PaletteGenDef* defaultPaletteGen();
int paletteGenCount();
const PaletteGenDef* paletteGenByIndex(int i);

// Resolves recipe.gen, falling back to the default generator when it is empty or unknown.  Returns false
//  if a fallback was used, so callers can tell the user their document is from a newer version.
bool generatePalette(const PaletteRecipe& recipe, Palette* out);

// The fixed palette that __inkbase values are quoted against (COLORS_SPEC.md §5.3).  Its recipe is frozen
//  along with the generator that builds it.
const Palette& referencePalette();

// --- shipped themes --------------------------------------------------------------------------------
//
// The themes the picker offers.  A theme is INK CHARACTER plus PAPER TINT; it deliberately does NOT
//  own light-vs-dark, which is the dialog's toggle.  paperL does two jobs - which side of the
//  generator's mirror, and how light exactly - and only the second belongs to a theme.  Letting a
//  theme set paperL outright makes "vivid ink on white" and "vivid ink on black" two separate entries
//  when they are one theme seen in two modes.
//
// Hue is NOT a theme axis, and that is measured rather than assumed: with families spread over the
//  full wheel, two palettes 180 degrees apart differ by a mean of 10 degrees per color - less than the
//  jitter.  Restricting the hue span would make seeds matter, but a 120-degree theme has no blue in
//  it at all, which is not a palette anyone can take notes with.  So every theme covers the whole
//  wheel and they differ by how the ink sits on the page instead.
struct PaletteTheme
{
  const char* id;       // stable, but never stored - the document stores the resulting recipe
  const char* name;
  real vividness;
  real depth;
  real minContrast;
  real paperWarm;
  real paperOffset;     // how far this theme's paper sits from pure white / pure black
};

int paletteThemeCount();
const PaletteTheme* paletteThemeByIndex(int i);
const PaletteTheme* paletteThemeById(const char* id);

// The recipe for a theme.  `dark` is the only thing that decides which side of the mirror the paper
//  sits on, so the same theme has a light and a dark rendering rather than a light and a dark entry.
PaletteRecipe paletteThemeRecipe(const PaletteTheme& theme, bool dark);

// Which shipped theme a recipe came from, or -1.  Matches on the character fields only, so a recipe
//  still resolves to its theme after the dark toggle has moved paperL.  Used to ring the active tile.
int paletteThemeIndexOf(const PaletteRecipe& recipe);
