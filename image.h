#pragma once

#include <stddef.h>
#include <stdint.h>
#include <vector>
#include "geom.h"

#define USE_STB_IMAGE

class Image {
public:
  typedef std::vector<unsigned char> EncodeBuff;

  int width;
  int height;
  unsigned char* data;  //std::vector<char> data;
  mutable EncodeBuff encData;
  enum Encoding {UNKNOWN=0, PNG=1, JPEG=2} encoding;  // preferred encoding
  mutable int painterHandle;
  // texture of this image as rewritten by a ColorMap (night mode), for the map whose imageKey() is
  //  mappedKey; mappedHandle == UNMAPPED records that the map chose to leave this image alone (a photo)
  mutable int mappedHandle = -1;
  mutable int mappedKey = 0;
  static constexpr int UNMAPPED = -2;
  // identifies the pixels: every constructor and invalidate() take a new, never reused id and a move carries
  //  it along, so something derived from the image (a scaled save, a thumbnail proxy) can be cached against
  //  it - pixels changed in place must be followed by invalidate(), as the texture cache already requires
  uint64_t contentId;

  Image(int w, int h, Encoding imgfmt = UNKNOWN);
  Image(Image&& other);
  Image& operator=(Image&& obj);
  ~Image();
  Image copy() const { return Image(*this); }
  void invalidate();

  unsigned char* bytes() { if(!data && !encData.empty()) data = bytesOnce(); return data; }
  const unsigned char* constBytes() const { return const_cast<Image*>(this)->bytes(); }
  unsigned int* pixels() { return (unsigned int*)bytes(); }
  const unsigned int* constPixels() const { return (const unsigned int*)constBytes(); }
  unsigned char* bytesOnce() const;
  int dataLen() const { return width*height*4; }
  int getWidth() const { return width; }
  int getHeight() const { return height; }
  bool hasTransparency() const;
  Image& subtract(const Image& other, int scale=1, int offset=0);

  EncodeBuff encode(Encoding dflt) const;  // dflt=PNG
  EncodeBuff encodePNG() const;
  EncodeBuff encodeJPEG(int quality = 75) const;
  // PNG of tightly packed 8-bit samples (`channels` 1-4 a pixel, rows `stride` bytes apart), for a big
  //  image made once - an imported PDF page.  Rows are filtered as encodePNG() filters them, then deflated
  //  in independent horizontal stripes on up to `threads` threads (one IDAT chunk each) with a shallower
  //  match search than encodePNG()'s level 8, which spent seconds a page on scans.  Lossless: it decodes to
  //  exactly the pixels given; the bytes differ from encodePNG()'s.  Empty on failure.
  static EncodeBuff encodePNGStriped(const unsigned char* samples, int w, int h, int channels,
      ptrdiff_t stride, int threads);
  // Start decoding this image's encoded bytes on a worker thread, for an image about to be drawn (the page
  //  after the visible ones): bytesOnce() then takes the pixels from the worker instead of decoding a
  //  300 DPI page on the UI thread, waiting for it if it is still running.  Only for an image that has
  //  encoded bytes and no pixels; at most a couple are held at once, the oldest dropped first.  Returns
  //  true if a decode is pending or was started.  A no-op without threads (wasm).
  bool prefetchDecode() const;

  void fill(unsigned int color);
  void fillRect(Rect rect, unsigned int color);
  Image scaled(int w, int h) const;  // return a scaled version of the image
  Image transformed(const Transform2D& tf) const;
  Image cropped(const Rect& src) const;
  bool isNull() const { return !data && encData.empty(); }
  bool operator==(const Image& other) const;
  bool operator!=(const Image& other) const { return !operator==(other); }

  //enum PixelFormat {ARGB32, ARGB32_Premul, RGB32};
  //PixelFormat format() const { return ARGB32; }
  //Image convertToFormat(PixelFormat format) const { return Image(*this); }
  //unsigned int pixel(int x, int y) const { return constPixels()[y * width + x]; }
  //unsigned int* scanLine(int y) { return pixels() + y * width; }

#ifndef USE_STB_IMAGE
  static Image* decodePNG(const char* buff, size_t len);
  static Image* decodeJPEG(const char* buff, size_t len);
#endif
  static Image decodeBuffer(const void* buff, size_t len, Encoding formatHint = UNKNOWN);
  static Image fromPixels(int w, int h, unsigned char* d, Encoding imgfmt = UNKNOWN);
  static Image fromPixelsNoCopy(int w, int h, unsigned char* d, Encoding imgfmt = UNKNOWN);
protected:
  void dropPrefetch() const;
  Image(int w, int h, unsigned char* d, Encoding imgfmt, EncodeBuff encdata = EncodeBuff())
      : width(w), height(h), data(d), encData(encdata), encoding(imgfmt), painterHandle(-1),
        contentId(newContentId()) {}
  static uint64_t newContentId();
  Image(const Image& other);
};
