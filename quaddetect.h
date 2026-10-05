#pragma once

#include "geom.h"

// Finding the page outline in a photo, so the scan dialog's corner handles start somewhere sensible.
// This only has to produce a good *initial guess* - the user can drag any corner afterwards - which is
//  what makes it reasonable to do without OpenCV.  A wrong guess costs one drag; refusing to guess costs
//  four.  So the bar is "usually right and never absurd", not "always right".
// Pipeline: downscale to a few hundred pixels, Sobel with thinning, gradient-directed Hough that keeps each
//  line's polarity (which side is brighter), then score every combination of a left, top, right and bottom
//  candidate by how much of each side - between its corners - looks like a page edge, and refit the winner.
// Buffers are the usual Image layout: 4 bytes per pixel, alpha at byte offset 3.

// quad is filled with the corners in source pixel coords, ordered TL, TR, BR, BL.
// Returns false if nothing plausible was found, in which case quad is left alone - callers should fall
//  back to defaultDocumentQuad().
bool detectDocumentQuad(const unsigned int* pixels, int width, int height, Point quad[4]);

// a plain rectangle inset from the image edges, for when detection fails or is turned off
void defaultDocumentQuad(int width, int height, real insetFraction, Point quad[4]);
