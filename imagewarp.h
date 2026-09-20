#pragma once

#include "homography.h"

class ThreadPool;

// Perspective ("keystone") correction for document scanning: given the four corners of a page in a photo,
//  produce a head-on view of it.  Painter/nanovg can't do this - its paint matrices are affine - and a GL
//  path would need a custom shader plus an FBO readback, so this is a plain CPU implementation.  It runs
//  once per scan, not per frame, so that is the right tradeoff.
// These take raw RGBA buffers rather than Image deliberately: it keeps this translation unit free of the
//  Painter/nanovg dependency that image.cpp pulls in, so it can be unit tested on its own.  Buffers are
//  the usual Image layout - 4 bytes per pixel with alpha at byte offset 3 (accessed as bytes, so this does
//  not depend on endianness).  RGB order is irrelevant, since channels 0-2 are only ever averaged.

// quad corners are ordered TL, TR, BR, BL to match Transform3D

// dstToSrc maps destination pixel coords to source pixel coords (i.e. the inverse of the visual mapping);
//  supersample is the width of the box filter grid per destination pixel, 0 to choose it automatically
void warpPerspectiveRGBA(const unsigned int* srcPix, int srcw, int srch,
    unsigned int* dstPix, int dstw, int dsth, const Transform3D& dstToSrc,
    int supersample = 0, ThreadPool* pool = NULL);

// convenience wrapper: flatten the quad in srcPix onto the whole of dstPix
void warpQuadRGBA(const unsigned int* srcPix, int srcw, int srch,
    unsigned int* dstPix, int dstw, int dsth, const Point quad[4], ThreadPool* pool = NULL);

// output size for a document quad, preserving its approximate aspect ratio, long edge capped at maxdim
void quadOutputSize(const Point quad[4], int maxdim, int* dstw, int* dsth);

// true if the corners form a non-self-intersecting quad wound in the expected direction - the corner drag
//  UI needs this to reject a drag that would turn the quad inside out
bool isConvexQuad(const Point quad[4]);
