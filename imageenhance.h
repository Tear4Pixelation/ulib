#pragma once

#include "geom.h"

// The "make it look scanned rather than photographed" half of document scanning.  What actually separates
//  the two is not resolution, it is even lighting: a photo of a page carries the shadow of the phone and
//  whatever lamp is in the room, and that is what makes it read as a snapshot.  Dividing the image by a
//  heavily blurred copy of itself removes exactly that, because the blurred copy *is* the illumination.
// As with imagewarp, these take raw RGBA buffers rather than Image so that this translation unit stays
//  free of the Painter/nanovg dependency and can be unit tested on its own.  Buffers are 4 bytes per
//  pixel with alpha at byte offset 3; alpha is passed through untouched.
// All of these run once, when the user picks a filter in the scan dialog, not per frame.

enum ScanFilter {
  SCAN_ORIGINAL = 0,  // leave the warped photo alone
  SCAN_COLOR,         // flatten illumination per channel (also neutralizes color casts), then stretch
  SCAN_GREYSCALE,     // as above but on luma only
  SCAN_MONO           // greyscale plus a local threshold - tiny as PNG, which keeps .svgz small
};

// everything below works in place

void enhanceDocument(unsigned int* pixels, int width, int height, ScanFilter filter);

// building blocks, exposed separately so they can be tested and retuned independently
// lumaOnly preserves hue by scaling all three channels together; otherwise each channel is flattened on
//  its own, which also white balances the page
void flattenIllumination(unsigned int* pixels, int width, int height, int blurRadius, bool lumaOnly);
void stretchLevels(unsigned int* pixels, int width, int height, real lowPercent, real highPercent);
void toGreyscale(unsigned int* pixels, int width, int height);
void adaptiveThreshold(unsigned int* pixels, int width, int height, int blurRadius, int offset);

// the blur has to be much wider than the text strokes, or the text becomes its own background and gets
//  erased along with the shadows
int defaultBlurRadius(int width, int height);

// dstPix must have room for the rotated result: quarterTurns odd swaps width and height
void rotateQuarterTurns(const unsigned int* srcPix, int srcw, int srch, unsigned int* dstPix, int quarterTurns);
