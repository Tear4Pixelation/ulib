#include "imageenhance.h"

#include <stdint.h>
#include <string.h>
#include <vector>

static unsigned char clampToByte(float value)
{
  return (unsigned char)(value < 0 ? 0 : (value > 255 ? 255 : value + 0.5f));
}

static unsigned char lumaOf(const unsigned char* pixel)
{
  // Rec. 601 luma, the usual weighting for perceived brightness
  return (unsigned char)((pixel[0]*77 + pixel[1]*150 + pixel[2]*29) >> 8);
}

// Summed-area table with a zero first row and column, so the sum over any box is four lookups no matter
//  how large the box is - which is the whole point here, since the blur radius is a large fraction of the
//  image and a separable blur would still be O(radius) per pixel.
// 64-bit because a 32-bit accumulator overflows at 255*width*height, i.e. around 16 megapixels; scans are
//  usually smaller than that, but "usually" is not a good enough reason to leave a silent wraparound in.
static void buildIntegral(const unsigned char* channelSrc, int width, int height, int srcStride,
    std::vector<uint64_t>& integral)
{
  integral.assign(size_t(width + 1)*(height + 1), 0);
  for(int y = 0; y < height; ++y) {
    uint64_t rowSum = 0;
    const unsigned char* srcRow = channelSrc + size_t(y)*width*srcStride;
    uint64_t* prevRow = &integral[size_t(y)*(width + 1)];
    uint64_t* curRow = &integral[size_t(y + 1)*(width + 1)];
    for(int x = 0; x < width; ++x) {
      rowSum += srcRow[size_t(x)*srcStride];
      curRow[x + 1] = prevRow[x + 1] + rowSum;
    }
  }
}

// mean of the box of the given radius centered on (x,y), clipped to the image
static float boxAverage(const std::vector<uint64_t>& integral, int width, int height,
    int x, int y, int radius)
{
  int left = std::max(0, x - radius), right = std::min(width, x + radius + 1);
  int top = std::max(0, y - radius), bottom = std::min(height, y + radius + 1);
  const uint64_t* topRow = &integral[size_t(top)*(width + 1)];
  const uint64_t* bottomRow = &integral[size_t(bottom)*(width + 1)];
  uint64_t sum = bottomRow[right] - bottomRow[left] - topRow[right] + topRow[left];
  int area = (right - left)*(bottom - top);
  return area > 0 ? float(sum)/area : 0;
}

int defaultBlurRadius(int width, int height)
{
  // roughly a sixth of the page across: wide enough that a line of text averages out into its background,
  //  narrow enough to still follow a shadow gradient
  return std::max(8, std::max(width, height)/12);
}

void flattenIllumination(unsigned int* pixels, int width, int height, int blurRadius, bool lumaOnly)
{
  if(!pixels || width <= 0 || height <= 0 || blurRadius < 1)
    return;
  unsigned char* bytes = (unsigned char*)pixels;
  std::vector<uint64_t> integral;
  if(lumaOnly) {
    std::vector<unsigned char> luma(size_t(width)*height);
    for(size_t ii = 0; ii < luma.size(); ++ii)
      luma[ii] = lumaOf(bytes + ii*4);
    buildIntegral(luma.data(), width, height, 1, integral);
    for(int y = 0; y < height; ++y) {
      for(int x = 0; x < width; ++x) {
        size_t index = size_t(y)*width + x;
        float background = boxAverage(integral, width, height, x, y, blurRadius);
        // where the local background is already black there is no illumination to divide out, and doing
        //  it anyway would amplify sensor noise into a blown-out mess
        float gain = background < 1 ? 1.0f : 255.0f/background;
        unsigned char* pixel = bytes + index*4;
        pixel[0] = clampToByte(pixel[0]*gain);
        pixel[1] = clampToByte(pixel[1]*gain);
        pixel[2] = clampToByte(pixel[2]*gain);
      }
    }
  }
  else {
    // one channel at a time: each channel's own blurred copy is its own illumination estimate, so this
    //  corrects a color cast (warm lamp, blue window light) as a side effect
    for(int channel = 0; channel < 3; ++channel) {
      buildIntegral(bytes + channel, width, height, 4, integral);
      for(int y = 0; y < height; ++y) {
        for(int x = 0; x < width; ++x) {
          unsigned char* pixel = bytes + (size_t(y)*width + x)*4 + channel;
          float background = boxAverage(integral, width, height, x, y, blurRadius);
          *pixel = background < 1 ? *pixel : clampToByte(*pixel*(255.0f/background));
        }
      }
    }
  }
}

void stretchLevels(unsigned int* pixels, int width, int height, real lowPercent, real highPercent)
{
  if(!pixels || width <= 0 || height <= 0 || !(highPercent > lowPercent))
    return;
  unsigned char* bytes = (unsigned char*)pixels;
  size_t nPixels = size_t(width)*height;
  int histogram[256] = {0};
  for(size_t ii = 0; ii < nPixels; ++ii)
    ++histogram[lumaOf(bytes + ii*4)];

  // percentiles rather than the outright min and max, so that a single dust speck or specular highlight
  //  cannot decide the whole mapping
  size_t lowTarget = size_t(nPixels*lowPercent/100);
  size_t highTarget = size_t(nPixels*highPercent/100);
  int lowLevel = 0, highLevel = 255;
  size_t cumulative = 0;
  for(int level = 0; level < 256; ++level) {
    cumulative += histogram[level];
    if(cumulative > lowTarget) { lowLevel = level; break; }
  }
  cumulative = 0;
  for(int level = 255; level >= 0; --level) {
    cumulative += histogram[level];
    if(nPixels - cumulative < highTarget) { highLevel = level; break; }
  }
  if(highLevel <= lowLevel)
    return;

  unsigned char lut[256];
  float scale = 255.0f/(highLevel - lowLevel);
  for(int level = 0; level < 256; ++level)
    lut[level] = clampToByte((level - lowLevel)*scale);
  // the same mapping for all three channels; a per channel stretch would shift hues
  for(size_t ii = 0; ii < nPixels; ++ii) {
    unsigned char* pixel = bytes + ii*4;
    pixel[0] = lut[pixel[0]];
    pixel[1] = lut[pixel[1]];
    pixel[2] = lut[pixel[2]];
  }
}

void toGreyscale(unsigned int* pixels, int width, int height)
{
  if(!pixels || width <= 0 || height <= 0)
    return;
  unsigned char* bytes = (unsigned char*)pixels;
  size_t nPixels = size_t(width)*height;
  for(size_t ii = 0; ii < nPixels; ++ii) {
    unsigned char* pixel = bytes + ii*4;
    pixel[0] = pixel[1] = pixel[2] = lumaOf(pixel);
  }
}

void adaptiveThreshold(unsigned int* pixels, int width, int height, int blurRadius, int offset)
{
  if(!pixels || width <= 0 || height <= 0 || blurRadius < 1)
    return;
  unsigned char* bytes = (unsigned char*)pixels;
  size_t nPixels = size_t(width)*height;
  std::vector<unsigned char> luma(nPixels);
  for(size_t ii = 0; ii < nPixels; ++ii)
    luma[ii] = lumaOf(bytes + ii*4);
  std::vector<uint64_t> integral;
  buildIntegral(luma.data(), width, height, 1, integral);
  // a global threshold would lose either the faint pencil or the dark ink; comparing each pixel against
  //  its own neighbourhood, with a margin so that blank paper does not dissolve into noise, keeps both
  for(int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      size_t index = size_t(y)*width + x;
      float localMean = boxAverage(integral, width, height, x, y, blurRadius);
      unsigned char value = luma[index] < localMean - offset ? 0 : 255;
      unsigned char* pixel = bytes + index*4;
      pixel[0] = pixel[1] = pixel[2] = value;
    }
  }
}

void enhanceDocument(unsigned int* pixels, int width, int height, ScanFilter filter)
{
  if(!pixels || width <= 0 || height <= 0)
    return;
  int blurRadius = defaultBlurRadius(width, height);
  switch(filter) {
  case SCAN_ORIGINAL:
    break;
  case SCAN_COLOR:
    flattenIllumination(pixels, width, height, blurRadius, false);
    stretchLevels(pixels, width, height, 0.5, 99.5);
    break;
  case SCAN_GREYSCALE:
    toGreyscale(pixels, width, height);
    flattenIllumination(pixels, width, height, blurRadius, true);
    stretchLevels(pixels, width, height, 0.5, 99.5);
    break;
  case SCAN_MONO:
    toGreyscale(pixels, width, height);
    flattenIllumination(pixels, width, height, blurRadius, true);
    // a much tighter radius than the illumination pass - this one has to track the text, not the lighting
    adaptiveThreshold(pixels, width, height, std::max(4, std::max(width, height)/40), 10);
    break;
  }
}

void rotateQuarterTurns(const unsigned int* srcPix, int srcw, int srch, unsigned int* dstPix, int quarterTurns)
{
  if(!srcPix || !dstPix || srcw <= 0 || srch <= 0)
    return;
  int turns = ((quarterTurns % 4) + 4) % 4;  // C's % keeps the sign of the dividend
  if(turns == 0) {
    memcpy(dstPix, srcPix, size_t(srcw)*srch*4);
    return;
  }
  if(turns == 2) {
    for(int y = 0; y < srch; ++y) {
      for(int x = 0; x < srcw; ++x)
        dstPix[size_t(y)*srcw + x] = srcPix[size_t(srch - 1 - y)*srcw + (srcw - 1 - x)];
    }
    return;
  }
  int dstw = srch;  // a quarter turn swaps the axes
  int dsth = srcw;
  for(int y = 0; y < dsth; ++y) {
    for(int x = 0; x < dstw; ++x) {
      dstPix[size_t(y)*dstw + x] = turns == 1 ? srcPix[size_t(srch - 1 - x)*srcw + y]
                                              : srcPix[size_t(x)*srcw + (srcw - 1 - y)];
    }
  }
}
