#include "imagewarp.h"
#include "threadutil.h"

#include <string.h>
#include <vector>

static const int MAX_SUPERSAMPLE = 4;

// accumulate one bilinear tap set, premultiplied by alpha so that partially out-of-bounds destination
//  pixels blend toward transparent instead of toward black
static void sampleBilinear(const unsigned char* srcBytes, int srcw, int srch, real u, real v, real* accum)
{
  // pixel (i,j) has its center at (i + 0.5, j + 0.5)
  real fracU = u - real(0.5), fracV = v - real(0.5);
  real floorU = std::floor(fracU), floorV = std::floor(fracV);
  int baseX = int(floorU), baseY = int(floorV);
  real tu = fracU - floorU, tv = fracV - floorV;
  const real weights[4] = { (1 - tu)*(1 - tv), tu*(1 - tv), (1 - tu)*tv, tu*tv };
  static const int tapOffsets[4][2] = { {0,0}, {1,0}, {0,1}, {1,1} };
  for(int tap = 0; tap < 4; ++tap) {
    int sx = baseX + tapOffsets[tap][0], sy = baseY + tapOffsets[tap][1];
    if(sx < 0 || sx >= srcw || sy < 0 || sy >= srch)
      continue;  // outside the source image contributes nothing, i.e. fully transparent
    const unsigned char* pixel = srcBytes + (size_t(sy)*srcw + sx)*4;
    real weight = weights[tap];
    real premul = weight*pixel[3]*(real(1)/255);
    accum[0] += premul*pixel[0];
    accum[1] += premul*pixel[1];
    accum[2] += premul*pixel[2];
    accum[3] += weight*pixel[3];
  }
}

static unsigned char clampToByte(real value)
{
  return (unsigned char)(value < 0 ? 0 : (value > 255 ? 255 : value + real(0.5)));
}

static void warpBand(const unsigned char* srcBytes, int srcw, int srch, unsigned char* dstBytes, int dstw,
    const Transform3D& dstToSrc, int firstRow, int lastRow, int nsub)
{
  const real invSub = real(1)/nsub;
  const int nsamples = nsub*nsub;
  const real* mat = dstToSrc.asArray();
  std::vector<real> accum(size_t(dstw)*4);
  for(int y = firstRow; y < lastRow; ++y) {
    std::fill(accum.begin(), accum.end(), real(0));
    for(int subY = 0; subY < nsub; ++subY) {
      real sampleY = y + (subY + real(0.5))*invSub;
      for(int subX = 0; subX < nsub; ++subX) {
        real sampleX = (subX + real(0.5))*invSub;
        // numerator and denominator are both linear in x, so step them along the scanline rather than
        //  re-evaluating the matrix product per pixel
        real numX = mat[0]*sampleX + mat[3]*sampleY + mat[6];
        real numY = mat[1]*sampleX + mat[4]*sampleY + mat[7];
        real denom = mat[2]*sampleX + mat[5]*sampleY + mat[8];
        for(int x = 0; x < dstw; ++x, numX += mat[0], numY += mat[1], denom += mat[2]) {
          if(denom == 0)
            continue;  // point at infinity - leave it transparent
          real invDenom = 1/denom;
          sampleBilinear(srcBytes, srcw, srch, numX*invDenom, numY*invDenom, &accum[size_t(x)*4]);
        }
      }
    }
    unsigned char* dstRow = dstBytes + size_t(y)*dstw*4;
    for(int x = 0; x < dstw; ++x) {
      const real* pixelAccum = &accum[size_t(x)*4];
      real alphaSum = pixelAccum[3];
      unsigned char* out = dstRow + size_t(x)*4;
      if(alphaSum <= 0) {
        out[0] = 0;  out[1] = 0;  out[2] = 0;  out[3] = 0;
      }
      else {
        // undo the premultiply; the per-sample weights already sum to 1, so alphaSum/nsamples is the
        //  average alpha and alphaSum/255 is the total premultiply factor that was applied
        real unpremul = 255/alphaSum;
        out[0] = clampToByte(pixelAccum[0]*unpremul);
        out[1] = clampToByte(pixelAccum[1]*unpremul);
        out[2] = clampToByte(pixelAccum[2]*unpremul);
        out[3] = clampToByte(alphaSum/nsamples);
      }
    }
  }
}

// how many source pixels does one destination pixel cover?  If we are downscaling appreciably, plain
//  bilinear aliases badly on small text, so fall back to a box filter of that many taps per axis.
static int autoSupersample(const Transform3D& dstToSrc, int dstw, int dsth)
{
  Point corners[4] = { dstToSrc.map(Point(0, 0)), dstToSrc.map(Point(dstw, 0)),
      dstToSrc.map(Point(dstw, dsth)), dstToSrc.map(Point(0, dsth)) };
  for(int ii = 0; ii < 4; ++ii) {
    if(corners[ii].isNaN())
      return 1;
  }
  // shoelace formula
  real area = 0;
  for(int ii = 0; ii < 4; ++ii)
    area += cross(corners[ii], corners[(ii + 1) % 4]);
  area = std::abs(area)/2;
  real linearScale = std::sqrt(area/(real(dstw)*dsth));
  // the scale varies across a perspective warp, but not enough for a document photo to be worth varying
  //  the filter per pixel; one conservative value for the whole image is plenty
  int nsub = int(linearScale + real(0.5));
  return nsub < 1 ? 1 : (nsub > MAX_SUPERSAMPLE ? MAX_SUPERSAMPLE : nsub);
}

void warpPerspectiveRGBA(const unsigned int* srcPix, int srcw, int srch,
    unsigned int* dstPix, int dstw, int dsth, const Transform3D& dstToSrc, int supersample, ThreadPool* pool)
{
  if(!dstPix || dstw <= 0 || dsth <= 0)
    return;
  if(!srcPix || srcw <= 0 || srch <= 0 || !dstToSrc.isValid()) {
    memset(dstPix, 0, size_t(dstw)*dsth*4);
    return;
  }
  int nsub = supersample > 0 ? std::min(supersample, MAX_SUPERSAMPLE) : autoSupersample(dstToSrc, dstw, dsth);
  const unsigned char* srcBytes = (const unsigned char*)srcPix;
  unsigned char* dstBytes = (unsigned char*)dstPix;

  const int MIN_THREADED_ROWS = 64;
  if(!pool || dsth < MIN_THREADED_ROWS) {
    warpBand(srcBytes, srcw, srch, dstBytes, dstw, dstToSrc, 0, dsth, nsub);
    return;
  }
  // more bands than threads so that the pool can even out the load; the bands are independent, each
  //  writing only its own rows, so no locking is needed
  const int NUM_BANDS = 16;
  int rowsPerBand = (dsth + NUM_BANDS - 1)/NUM_BANDS;
  std::vector< std::future<void> > pending;
  for(int firstRow = 0; firstRow < dsth; firstRow += rowsPerBand) {
    int lastRow = std::min(firstRow + rowsPerBand, dsth);
    pending.push_back(pool->enqueue(warpBand, srcBytes, srcw, srch, dstBytes, dstw, dstToSrc,
        firstRow, lastRow, nsub));
  }
  for(std::future<void>& task : pending)
    task.wait();
}

void warpQuadRGBA(const unsigned int* srcPix, int srcw, int srch,
    unsigned int* dstPix, int dstw, int dsth, const Point quad[4], ThreadPool* pool)
{
  Point dstQuad[4] = { Point(0, 0), Point(dstw, 0), Point(dstw, dsth), Point(0, dsth) };
  warpPerspectiveRGBA(srcPix, srcw, srch, dstPix, dstw, dsth,
      Transform3D::quadToQuad(dstQuad, quad), 0, pool);
}

void quadOutputSize(const Point quad[4], int maxdim, int* dstw, int* dsth)
{
  // averaging the two opposite edges of each pair is the honest estimate available without knowing the
  //  camera's focal length; recovering the true aspect ratio needs EXIF data that phones often omit
  real width = (quad[0].dist(quad[1]) + quad[3].dist(quad[2]))/2;
  real height = (quad[0].dist(quad[3]) + quad[1].dist(quad[2]))/2;
  if(!(width > 0) || !(height > 0)) {
    *dstw = 0;
    *dsth = 0;
    return;
  }
  // never upscale - the warp cannot add detail the photo does not have
  real scale = maxdim > 0 ? std::min(real(1), maxdim/std::max(width, height)) : 1;
  *dstw = std::max(1, int(width*scale + real(0.5)));
  *dsth = std::max(1, int(height*scale + real(0.5)));
}

bool isConvexQuad(const Point quad[4])
{
  int sign = 0;
  for(int ii = 0; ii < 4; ++ii) {
    Point edge = quad[(ii + 1) % 4] - quad[ii];
    Point next = quad[(ii + 2) % 4] - quad[(ii + 1) % 4];
    real turn = cross(edge, next);
    if(turn == 0)
      return false;  // collinear corners would make the homography degenerate
    int turnSign = turn > 0 ? 1 : -1;
    if(sign == 0)
      sign = turnSign;
    else if(sign != turnSign)
      return false;
  }
  return true;
}
