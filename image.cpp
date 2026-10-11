// Image decoding/encoding:
// * uses https://github.com/nothings/stb - PNG/JPG enc (stb_image_write.h), PNG/JPG dec (stb_image.h)
// * optionally uses miniz to provide deflate for PNG (impl in stb_image_write uses fixed dictionary)
// * code using libjpeg(-turbo) and libpng hasn't been tested recently
// Refs:
// - https://blog.gibson.sh/2015/07/18/comparing-png-compression-ratios-of-stb_image_write-lodepng-miniz-and-libpng/
// - https://blog.gibson.sh/2015/03/23/comparing-performance-stb_image-vs-libjpeg-turbo-libpng-and-lodepng/
// Other options:
// - https://github.com/serge-rgb/TinyJPEG/ for JPEG compression
// - https://github.com/lvandeve/lodepng - PNG enc/dec (2 files)
// Image processing libraries:
// - CImg - can only read/write to files; could load with separate code
// - CxImage - seems to do everything; last update 2011

// JPEG images are degraded slightly every time they're encoded, with severe degredation after a few hundred
//  cycles, so we cache JPEG data when decoding to avoid reencoding unless modified; we don't bother for PNG
//  since reencoding is lossless and encoded data is closer in size to decoded data (so more memory usage)

// I/O options: char*, size_t vs. istream, ostream vs. vector/string
// libpng: uses callbacks passed source/dest and length, so buffer or streams work equally well
// libjpeg: uses buffers natively ... but since we need to convert RGB888 <-> RGBA8888, we have to go through
//   temp buffer both directions, so we can use streams IF we can combine the conversion with stream I/O
// STB_image: works with mem buffers natively; supports callbacks

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <utility>
#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <thread>
#include <chrono>
#include <algorithm>
#include "image.h"
#include "painter.h"

Image::Image(int w, int h, Encoding imgfmt) : Image(w, h, NULL, imgfmt)
{
  if(w > 0 && h > 0)
    data = (unsigned char*)calloc(w*h, 4);
}

Image::Image(Image&& other) : width(std::exchange(other.width, 0)), height(std::exchange(other.height, 0)),
    data(std::exchange(other.data, nullptr)), encData(std::move(other.encData)),
    encoding(other.encoding), painterHandle(std::exchange(other.painterHandle, -1)),
    mappedHandle(std::exchange(other.mappedHandle, -1)), mappedKey(std::exchange(other.mappedKey, 0)) {}

Image& Image::operator=(Image&& other)
{
  std::swap(width, other.width);
  std::swap(height, other.height);
  std::swap(data, other.data);
  std::swap(encData, other.encData);
  std::swap(encoding, other.encoding);
  std::swap(painterHandle, other.painterHandle);
  std::swap(mappedHandle, other.mappedHandle);
  std::swap(mappedKey, other.mappedKey);
  return *this;
}

// we've switched from vector to plain pointer for data since that's what stb_image's load fns return
// we can't copy painterHandle ... TODO: could use something like clone_ptr here instead
Image::Image(const Image& other) : width(other.width), height(other.height), data(NULL),
   encData(other.encData), encoding(other.encoding), painterHandle(-1)
{
  int n = width*height*4;
  if(other.data) {
    data = (unsigned char*)malloc(n);
    memcpy(data, other.data, n);
  }
}

Image Image::fromPixels(int w, int h, unsigned char* d, Encoding imgfmt)
{
  size_t n = w*h*4;
  unsigned char* ourpx = (unsigned char*)malloc(n);
  memcpy(ourpx, d, n);
  return Image(w, h, ourpx, imgfmt);
}

Image Image::fromPixelsNoCopy(int w, int h, unsigned char* d, Encoding imgfmt)
{
  return Image(w, h, d, imgfmt);
}

Image::~Image()
{
  invalidate();
  if(data)
    free(data);
}

void Image::invalidate()
{
  dropPrefetch();
  encData.clear();
  Painter::invalidateImage(painterHandle, dataLen());
  painterHandle = -1;
  if(mappedHandle >= 0)
    Painter::invalidateImage(mappedHandle, dataLen());
  mappedHandle = -1;
  mappedKey = 0;
}

// use Painter for image transformations
Image Image::transformed(const Transform2D& tf) const
{
  // apply transform to bounding rect to determine size of output image
  Rect b = tf.mapRect(Rect::wh(width, height));
  int wout = std::ceil(b.width());
  int hout = std::ceil(b.height());
  Image out(wout, hout, encoding);
  int usegpu = Painter::cachingPainter && Painter::cachingPainter->usesGPU() && (width * height > 1<<20);
  Painter painter(usegpu ? Painter::PAINT_GL : Painter::PAINT_SW, &out);
  painter.setBackgroundColor(Color::TRANSPARENT_COLOR);
  painter.beginFrame();
  painter.transform(Transform2D().translate(-b.left, -b.top) * tf);
  // all scaling done by transform, so pass dest = src so drawImage doesn't scale
  painter.drawImage(Rect::wh(width, height), *this, Rect(), data ? Painter::ImageNoCopy : 0);
  painter.endFrame();
  return out;
}

Image Image::scaled(int w, int h) const
{
  return transformed(Transform2D().scale(w/(float)width, h/(float)height));
}

Image Image::cropped(const Rect& src) const
{
  int outw = std::min(int(src.right), width) - int(src.left);
  int outh = std::min(int(src.bottom), height) - int(src.top);
  if(outw <= 0 || outh <= 0)
    return Image(0, 0);
  Image out(outw, outh, encoding);
  const unsigned int* srcpix = constPixels() + int(src.top)*width + int(src.left);
  unsigned int* dstpix = out.pixels();
  for(int y = 0; y < out.height; ++y) {
    for(int x = 0; x < out.width; ++x)
      dstpix[y*out.width + x] = srcpix[y*width + x];
  }
  return out;
}

void Image::fill(unsigned int color)
{
  invalidate();
  unsigned int* px = pixels();
  for(int ii = 0; ii < width*height; ++ii)
    px[ii] = color;
}

void Image::fillRect(Rect rect, unsigned int color)
{
  invalidate();
  unsigned int* px = pixels();
  int x0 = std::max(0, int(rect.left));
  int y0 = std::max(0, int(rect.top));
  int x1 = std::min(int(std::ceil(rect.right)), width);
  int y1 = std::min(int(std::ceil(rect.bottom)), height);

  if(color == 0 || color == 0xFFFFFFFF) {
    if(x0 == 0 && x1 == width)
      memset(&px[y0*width], color, 4*width*(y1 - y0));
    else {
      for(int iy = y0; iy < y1; ++iy)
        memset(&px[iy*width + x0], color, 4*(x1 - x0));
    }
  }
  else {
    for(int iy = y0; iy < y1; ++iy) {
      for(int ix = x0; ix < x1; ++ix)
        px[iy*width + ix] = color;
    }
  }
}

// only used for comparing test images, so we don't care about performance
Image& Image::subtract(const Image& other, int scale, int offset)
{
  invalidate();
  unsigned char* a = bytes();
  const unsigned char* b = other.constBytes();
  for(int ii = 0; ii < std::min(height, other.height); ++ii) {
    for(int jj = 0; jj < std::min(width, other.width)*4; ++jj) {
      if(jj % 4 != 3) // skip alpha
        a[ii*width*4 + jj] = scale*((int)a[ii*width*4 + jj] - b[ii*other.width*4 + jj]) + offset;
        //a[ii*width*4 + jj] = std::min(std::max(scale*((int)a[ii*width*4 + jj] - b[ii*other.width*4 + jj]) + offset, 0), 255);
    }
  }
  return *this;
}

bool Image::operator==(const Image& other) const
{
  if(width != other.width || height != other.height)
    return false;
  return memcmp(constBytes(), other.constBytes(), dataLen()) == 0;
}

bool Image::hasTransparency() const
{
  if(encoding == JPEG && !data) return false;  // unmodified from JPEG source implies no transparency
  const unsigned int* pixels = constPixels();
  for(int ii = 0; ii < width*height; ++ii) {
    if((pixels[ii] & 0xFF000000) != 0xFF000000)
      return true;
  }
  return false;
}

// decoding

// JPEG and PNG dominate code, so excluding other types gains little
//#define STBI_ONLY_JPEG
//#define STBI_ONLY_PNG
//#define STBI_ONLY_BMP
// don't need file I/O
//#define STBI_NO_STDIO
//#define STB_IMAGE_IMPLEMENTATION -- we expect this to be done somewhere else
#include "stb_image.h"

// Decoding ahead (prefetchDecode): one worker thread decodes a copy of an image's encoded bytes - a copy,
//  because the Image may be destroyed or moved meanwhile - and bytesOnce() collects the pixels.  Jobs are
//  keyed by the address of the Image's encoded buffer, which a move keeps; invalidate() (so the destructor
//  too) drops the job before the buffer can be freed or reused, so a key never outlives its bytes.
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
#define IMAGE_NO_THREADS 1
#endif

namespace {

struct DecodeJob {
  enum State { QUEUED, RUNNING, DONE, CANCELLED };
  const unsigned char* key;
  size_t keyLen;
  Image::EncodeBuff input;
  unsigned char* pixels = NULL;
  State state = QUEUED;
  std::chrono::steady_clock::time_point started;
  bool orphaned = false;  // dropped while running: the worker frees what it made
};

struct DecodeAhead {
  static constexpr size_t MAX_JOBS = 2;  // the pages either side of the view; ~35 MB each at 300 DPI
  std::mutex mutex;
  std::condition_variable wake;  // worker: a job was queued
  std::condition_variable done;  // bytesOnce(): a running job finished
  std::vector< std::shared_ptr<DecodeJob> > jobs;  // registered, oldest first
  std::deque< std::shared_ptr<DecodeJob> > queue;
  bool started = false;

  std::vector< std::shared_ptr<DecodeJob> >::iterator find(const unsigned char* key, size_t len)
  {
    return std::find_if(jobs.begin(), jobs.end(),
        [&](const std::shared_ptr<DecodeJob>& job) { return job->key == key && job->keyLen == len; });
  }

  // with the lock held: forget a registered job, freeing or cancelling what it holds
  void release(std::vector< std::shared_ptr<DecodeJob> >::iterator it)
  {
    DecodeJob* job = it->get();
    if(job->state == DecodeJob::DONE)
      free(job->pixels);
    else if(job->state == DecodeJob::RUNNING)
      job->orphaned = true;
    else
      job->state = DecodeJob::CANCELLED;  // still queued: the worker skips it
    job->pixels = NULL;
    jobs.erase(it);
    --registered;
  }

  void run()
  {
    for(;;) {
      std::shared_ptr<DecodeJob> job;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [this](){ return !queue.empty(); });
        job = std::move(queue.front());
        queue.pop_front();
        if(job->state == DecodeJob::CANCELLED)
          continue;
        job->state = DecodeJob::RUNNING;
        job->started = std::chrono::steady_clock::now();
      }
      int w = 0, h = 0;
      unsigned char* pixels = stbi_load_from_memory(job->input.data(), int(job->input.size()), &w, &h, NULL, 4);
      Image::EncodeBuff().swap(job->input);
      {
        std::lock_guard<std::mutex> lock(mutex);
        job->state = DecodeJob::DONE;
        addSample(workerMsPerMpx, std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - job->started).count(), double(w)*h/1E6);
        if(job->orphaned)
          free(pixels);
        else
          job->pixels = pixels;
      }
      done.notify_all();
    }
  }

  std::atomic<int> registered{0};  // jobs.size(), readable without the lock

  // Smoothed decode cost, ms per megapixel, on the worker and on the drawing thread; 0 until measured.
  //  When the page is needed while its decode is still running, waiting can be slower than decoding it
  //  right there - the worker started a frame ago and shares the CPU with the frames being drawn (a fast
  //  fling): bytesOnce() then decodes it itself.
  double workerMsPerMpx = 0;
  double localMsPerMpx = 0;
  static void addSample(double& average, double ms, double mpx)
  {
    if(mpx > 0) average = average > 0 ? 0.7*average + 0.3*ms/mpx : ms/mpx;
  }
};

// never destroyed: the worker thread runs until the process ends
DecodeAhead* decodeAhead() { static DecodeAhead* instance = new DecodeAhead; return instance; }

}  // namespace

bool Image::prefetchDecode() const
{
#ifdef IMAGE_NO_THREADS
  return false;
#else
  if(data || encData.empty()) return false;
  DecodeAhead* ahead = decodeAhead();
  std::lock_guard<std::mutex> lock(ahead->mutex);
  if(ahead->find(encData.data(), encData.size()) != ahead->jobs.end())
    return true;
  if(ahead->jobs.size() >= DecodeAhead::MAX_JOBS)
    ahead->release(ahead->jobs.begin());
  auto job = std::make_shared<DecodeJob>();
  job->key = encData.data();
  job->keyLen = encData.size();
  job->input = encData;
  ahead->jobs.push_back(job);
  ++ahead->registered;
  ahead->queue.push_back(std::move(job));
  if(!ahead->started) {
    ahead->started = true;
    std::thread([ahead](){ ahead->run(); }).detach();
  }
  ahead->wake.notify_one();
  return true;
#endif
}

void Image::dropPrefetch() const
{
#ifndef IMAGE_NO_THREADS
  if(encData.empty()) return;
  DecodeAhead* ahead = decodeAhead();
  if(ahead->registered.load(std::memory_order_acquire) == 0) return;
  std::lock_guard<std::mutex> lock(ahead->mutex);
  auto it = ahead->find(encData.data(), encData.size());
  if(it != ahead->jobs.end())
    ahead->release(it);
#endif
}

unsigned char* Image::bytesOnce() const
{
  if(data || encData.empty()) return data;
#ifndef IMAGE_NO_THREADS
  DecodeAhead* ahead = decodeAhead();
  if(ahead->registered.load(std::memory_order_acquire) > 0) {
    std::unique_lock<std::mutex> lock(ahead->mutex);
    auto it = ahead->find(encData.data(), encData.size());
    if(it != ahead->jobs.end()) {
      std::shared_ptr<DecodeJob> job = *it;
      if(job->state == DecodeJob::RUNNING) {
        double mpx = double(width)*height/1E6;
        double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - job->started).count();
        double remaining = ahead->workerMsPerMpx*mpx - elapsed;
        bool decodeHere = ahead->workerMsPerMpx > 0 && ahead->localMsPerMpx > 0
            && remaining > ahead->localMsPerMpx*mpx;
        if(!decodeHere)
          ahead->done.wait(lock, [&](){ return job->state == DecodeJob::DONE; });
      }
      unsigned char* pixels = job->state == DecodeJob::DONE ? std::exchange(job->pixels, nullptr) : NULL;
      auto still = ahead->find(encData.data(), encData.size());  // the wait let go of the lock
      if(still != ahead->jobs.end())
        ahead->release(still);
      if(pixels)
        return pixels;  // else not started yet (now cancelled), dropped as too far behind, or failed
    }
  }
#endif
  int w = 0, h = 0;
#ifndef IMAGE_NO_THREADS
  if(width*height >= 512*512) {  // timed, for the choice above
    auto start = std::chrono::steady_clock::now();
    unsigned char* pixels = stbi_load_from_memory(encData.data(), encData.size(), &w, &h, NULL, 4);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    std::lock_guard<std::mutex> lock(ahead->mutex);
    DecodeAhead::addSample(ahead->localMsPerMpx, ms, double(w)*h/1E6);
    return pixels;
  }
#endif
  return stbi_load_from_memory(encData.data(), encData.size(), &w, &h, NULL, 4);  // request 4 channels (RGBA)
}

Image Image::decodeBuffer(const void* buff, size_t len, Encoding formatHint)
{
  if(!buff || len < 16)
    return Image(0, 0);

  const unsigned char* cbuff = (const unsigned char*)buff;
  if(cbuff[0] == 0xFF && cbuff[1] == 0xD8)
    formatHint = JPEG;
  else if(memcmp(cbuff, "\x89PNG", 4) == 0)
    formatHint = PNG;

#ifdef USE_STB_IMAGE
  int w = 0, h = 0;
  stbi_info_from_memory(cbuff, len, &w, &h, NULL);
  //unsigned char* data = stbi_load_from_memory(buff, len, &w, &h, NULL, 4);  // request 4 channels (RGBA)
  return Image(w, h, NULL, formatHint, EncodeBuff(cbuff, cbuff+len));  //formatHint == JPEG ?
#else
  if(formatHint == PNG)
    return decodePNG(cbuff, len);
  else //if(formatHint == JPEG)
    return decodeJPEG(cbuff, len);
#endif
}

// encoding

Image::EncodeBuff Image::encode(Encoding fmt) const
{
  return fmt == JPEG ? encodeJPEG() : encodePNG();
}

#ifndef NO_MINIZ
#include "miniz/miniz.h"

// use miniz instead of zlib impl built into stb_image_write for better compression
static unsigned char* mz_stbiw_zlib_compress(unsigned char *data, int data_len, int *out_len, int quality)
{
  mz_ulong buflen = mz_compressBound(data_len);
  // Note that the returned buffer will be free'd by stbi_write_png*()
  // with STBIW_FREE(), so if you have overridden that (+ STBIW_MALLOC()),
  // adjust the next malloc() call accordingly:
  unsigned char* buf = (unsigned char*)malloc(buflen);
  if(buf == NULL || mz_compress2(buf, &buflen, data, data_len, quality) != 0) {
    free(buf); // .. yes, this would have to be adjusted as well.
    return NULL;
  }
  *out_len = buflen;
  return buf;
}

#define STBIW_ZLIB_COMPRESS  mz_stbiw_zlib_compress
#endif // NO_MINIZ

//#define STBI_WRITE_NO_STDIO
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

static void stbi_write_vec(void* context, void* data, int size)
{
  auto v = static_cast<Image::EncodeBuff*>(context);
  auto d = static_cast<unsigned char*>(data);
  v->insert(v->end(), d, d + size);
}

// use of encData: can be used for PNG or JPEG, but PNG never overwrites JPEG
Image::EncodeBuff Image::encodePNG() const
{
  //stbi_write_png_compression_level = quality;
  if(encData.size() > 4 && memcmp(encData.data(), "\x89PNG", 4) == 0)
    return encData;
  auto* b = constBytes();
  if(!b) return {};
  EncodeBuff vbuff;
  EncodeBuff& v = encData.empty() ? encData : vbuff;
  v.reserve(dataLen()/4);  // guess at compressed size
  // returns 0 on failure ...
  if(!stbi_write_png_to_func(&stbi_write_vec, &v, width, height, 4, b, width*4))
    v.clear();
  return v;
}

// encodePNGStriped: PNG for big images made once (imported PDF pages).  encodePNG() at stb's level 8 with
//  its try-every-filter search took 0.6 s for a rendered A4 text page at 300 DPI and 4.7 s for a scan, 80-90%
//  of an import.  Here rows get the same per-row filter choice as stb's (least sum of absolute values), in
//  loops that vectorize, and deflate searches 192 match candidates instead of 512 (lazy matching kept):
//  measured +0.3% bytes on text pages and +3.8% on scans for ~2x less CPU.  The image is cut into stripes
//  deflated independently on separate threads - a filter only looks one row up, into the source pixels, so
//  stripes need nothing from each other; all but the last end in a sync flush so the raw deflate streams
//  join into one zlib stream, and the stripes' Adler-32s combine.  Each stripe is its own IDAT chunk.
#ifndef NO_MINIZ
namespace {

constexpr mz_uint STRIPED_PNG_PROBES = 192;

// branchless, so the row loop vectorizes
inline unsigned char pngPaeth(int a, int b, int c)
{
  int pa = abs(b - c), pb = abs(a - c), pc = abs(a + b - 2*c);
  int notA = (pa > pb) | (pa > pc);
  int bOrC = pb <= pc ? b : c;
  return (unsigned char)(notA ? bOrC : a);
}

// one row with PNG filter `type` (0-4); `prev` NULL for the image's first row, which filters against zeros
void pngFilterRow(const unsigned char* __restrict row, const unsigned char* __restrict prev, int len, int n,
    int type, unsigned char* __restrict out)
{
  switch(type) {
  case 0:
    memcpy(out, row, len);
    break;
  case 1:
    for(int i = 0; i < n; ++i) out[i] = row[i];
    for(int i = n; i < len; ++i) out[i] = row[i] - row[i-n];
    break;
  case 2:
    if(!prev) memcpy(out, row, len);
    else for(int i = 0; i < len; ++i) out[i] = row[i] - prev[i];
    break;
  case 3:
    if(!prev) {
      for(int i = 0; i < n; ++i) out[i] = row[i];
      for(int i = n; i < len; ++i) out[i] = row[i] - (row[i-n] >> 1);
    }
    else {
      for(int i = 0; i < n; ++i) out[i] = row[i] - (prev[i] >> 1);
      for(int i = n; i < len; ++i) out[i] = row[i] - ((row[i-n] + prev[i]) >> 1);
    }
    break;
  case 4:
    if(!prev) {  // Paeth with the row above all zeros is Sub
      for(int i = 0; i < n; ++i) out[i] = row[i];
      for(int i = n; i < len; ++i) out[i] = row[i] - row[i-n];
    }
    else {
      for(int i = 0; i < n; ++i) out[i] = row[i] - prev[i];
      for(int i = n; i < len; ++i) out[i] = row[i] - pngPaeth(row[i-n], prev[i], prev[i-n]);
    }
    break;
  }
}

// stb's estimate of how well a filtered row compresses: the less, the better
int pngRowCost(const unsigned char* __restrict line, int len)
{
  int cost = 0;
  for(int i = 0; i < len; ++i) cost += abs((signed char)line[i]);
  return cost;
}

uint32_t adler32Combine(uint32_t adler1, uint32_t adler2, uint64_t len2)
{
  const uint64_t BASE = 65521;
  uint64_t rem = len2 % BASE;
  uint64_t sum1 = adler1 & 0xffff;
  uint64_t sum2 = (rem*sum1) % BASE;
  sum1 += (adler2 & 0xffff) + BASE - 1;
  sum2 += ((adler1 >> 16) & 0xffff) + ((adler2 >> 16) & 0xffff) + BASE - rem;
  if(sum1 >= BASE) sum1 -= BASE;
  if(sum1 >= BASE) sum1 -= BASE;
  if(sum2 >= (BASE << 1)) sum2 -= (BASE << 1);
  if(sum2 >= BASE) sum2 -= BASE;
  return uint32_t(sum1 | (sum2 << 16));
}

void putBE32(Image::EncodeBuff& out, uint32_t value)
{
  unsigned char bytes[4] = { (unsigned char)(value >> 24), (unsigned char)(value >> 16),
      (unsigned char)(value >> 8), (unsigned char)value };
  out.insert(out.end(), bytes, bytes + 4);
}

void putPngChunk(Image::EncodeBuff& out, const char* type, const unsigned char* bytes, uint32_t len)
{
  putBE32(out, len);
  size_t start = out.size();
  out.insert(out.end(), type, type + 4);
  if(len) out.insert(out.end(), bytes, bytes + len);
  putBE32(out, uint32_t(mz_crc32(MZ_CRC32_INIT, &out[start], len + 4)));
}

struct PngStripe {
  Image::EncodeBuff chunk;  // a complete IDAT chunk
  uint32_t adler = 1;
  size_t filteredLen = 0;
  bool ok = false;
};

mz_bool appendDeflated(const void* buff, int len, void* user)
{
  auto out = static_cast<Image::EncodeBuff*>(user);
  out->insert(out->end(), (const unsigned char*)buff, (const unsigned char*)buff + len);
  return MZ_TRUE;
}

// rows [y0, y1) filtered, raw deflated and wrapped as an IDAT chunk
void encodePngStripe(const unsigned char* samples, ptrdiff_t stride, int w, int n, int y0, int y1, bool last,
    PngStripe* stripe)
{
  const int len = w*n;
  Image::EncodeBuff filtered(size_t(len + 1)*(y1 - y0));
  Image::EncodeBuff trial(size_t(len)*5);
  for(int y = y0; y < y1; ++y) {
    const unsigned char* row = samples + y*stride;
    const unsigned char* prev = y > 0 ? row - stride : NULL;
    unsigned char* dest = &filtered[size_t(len + 1)*(y - y0)];
    int best = 0, bestCost = INT32_MAX;
    for(int type = 0; type < 5; ++type) {
      pngFilterRow(row, prev, len, n, type, &trial[size_t(len)*type]);
      int cost = pngRowCost(&trial[size_t(len)*type], len);
      if(cost < bestCost) { bestCost = cost; best = type; }
    }
    dest[0] = (unsigned char)best;
    memcpy(dest + 1, &trial[size_t(len)*best], len);
  }
  Image::EncodeBuff().swap(trial);
  stripe->filteredLen = filtered.size();
  stripe->adler = uint32_t(mz_adler32(MZ_ADLER32_INIT, filtered.data(), filtered.size()));
  Image::EncodeBuff& out = stripe->chunk;
  out.reserve(filtered.size()/4 + 64);
  out.resize(8);  // length and type, filled in below
  // probes as level 6 would set them (no zlib header: -15), then deeper
  mz_uint flags = tdefl_create_comp_flags_from_zip_params(6, -15, MZ_DEFAULT_STRATEGY);
  flags = (flags & ~mz_uint(TDEFL_MAX_PROBES_MASK)) | STRIPED_PNG_PROBES;
  tdefl_compressor* comp = (tdefl_compressor*)malloc(sizeof(tdefl_compressor));
  stripe->ok = comp && tdefl_init(comp, &appendDeflated, &out, int(flags)) == TDEFL_STATUS_OKAY
      && tdefl_compress_buffer(comp, filtered.data(), filtered.size(), last ? TDEFL_FINISH : TDEFL_SYNC_FLUSH)
          == (last ? TDEFL_STATUS_DONE : TDEFL_STATUS_OKAY);
  free(comp);
  uint32_t dataLen = uint32_t(out.size() - 8);
  for(int i = 0; i < 4; ++i)
    out[i] = (unsigned char)(dataLen >> (24 - 8*i));
  memcpy(&out[4], "IDAT", 4);
  putBE32(out, uint32_t(mz_crc32(MZ_CRC32_INIT, &out[4], dataLen + 4)));
}

}  // namespace

Image::EncodeBuff Image::encodePNGStriped(const unsigned char* samples, int w, int h, int channels,
    ptrdiff_t stride, int threads)
{
  static const unsigned char colorType[5] = {0, 0, 4, 2, 6};
  if(!samples || w <= 0 || h <= 0 || channels < 1 || channels > 4)
    return EncodeBuff();
#ifdef IMAGE_NO_THREADS
  threads = 1;
#endif
  // stripes of at least 64 rows: each restarts the match history, which costs a little compression
  int nstripes = std::max(1, std::min(threads, h/64));
  std::vector<PngStripe> stripes(nstripes);
  std::vector<std::thread> workers;
  for(int ii = 0; ii < nstripes; ++ii) {
    int y0 = int(int64_t(h)*ii/nstripes), y1 = int(int64_t(h)*(ii + 1)/nstripes);
    bool last = ii == nstripes - 1;
    if(last)  // on this thread
      encodePngStripe(samples, stride, w, channels, y0, y1, true, &stripes[ii]);
    else
      workers.emplace_back(encodePngStripe, samples, stride, w, channels, y0, y1, false, &stripes[ii]);
  }
  for(std::thread& worker : workers)
    worker.join();

  EncodeBuff out;
  size_t total = 0;
  for(const PngStripe& stripe : stripes) {
    if(!stripe.ok) return EncodeBuff();
    total += stripe.chunk.size();
  }
  out.reserve(total + 64);
  static const unsigned char signature[8] = {137, 80, 78, 71, 13, 10, 26, 10};
  out.insert(out.end(), signature, signature + 8);
  unsigned char ihdr[13] = { (unsigned char)(w >> 24), (unsigned char)(w >> 16), (unsigned char)(w >> 8),
      (unsigned char)w, (unsigned char)(h >> 24), (unsigned char)(h >> 16), (unsigned char)(h >> 8),
      (unsigned char)h, 8, colorType[channels], 0, 0, 0 };
  putPngChunk(out, "IHDR", ihdr, 13);
  const unsigned char zlibHeader[2] = {0x78, 0x9C};  // deflate, 32K window, default level
  putPngChunk(out, "IDAT", zlibHeader, 2);
  uint32_t adler = 1;
  for(const PngStripe& stripe : stripes) {
    out.insert(out.end(), stripe.chunk.begin(), stripe.chunk.end());
    adler = adler32Combine(adler, stripe.adler, stripe.filteredLen);
  }
  const unsigned char adlerBytes[4] = { (unsigned char)(adler >> 24), (unsigned char)(adler >> 16),
      (unsigned char)(adler >> 8), (unsigned char)adler };
  putPngChunk(out, "IDAT", adlerBytes, 4);
  putPngChunk(out, "IEND", NULL, 0);
  return out;
}
#else
Image::EncodeBuff Image::encodePNGStriped(const unsigned char* samples, int w, int h, int channels,
    ptrdiff_t stride, int threads)
{
  EncodeBuff out;
  if(!stbi_write_png_to_func(&stbi_write_vec, &out, w, h, channels, samples, int(stride)))
    out.clear();
  return out;
}
#endif  // NO_MINIZ

Image::EncodeBuff Image::encodeJPEG(int quality) const
{
  if(encData.size() && encData[0] == 0xFF)
    return encData;
  auto* b = constBytes();
  if(!b) return {};
  encData.clear();
  encData.reserve(dataLen()/4);
  // returns 0 on failure ...
  if(!stbi_write_jpg_to_func(&stbi_write_vec, &encData, width, height, 4, b, quality))
    encData.clear();
  return encData;  // makes a copy unavoidably
}

// libjpeg, libpng, and base64 code removed 19 Feb 2021

// test
#ifdef IMAGE_TEST
#include <fstream>

int main(int argc, char* argv[])
{
  if(argc < 2)
    return -1;
  std::vector<char> buff;
  if(!readFile(&buff, argv[1]))
    return -2;
  Image* img = Image::decodeBuffer(&buff[0], buff.size());
  if(!img)
    return -3;
  // reencode
  char* outbuff;
  size_t outlen;
  // write to jpeg
  img->encodeJPEG(&outbuff, &outlen);
  std::ofstream jpegout("out.jpg", std::ofstream::binary);
  jpegout.write(outbuff, outlen);
  jpegout.close();
  free(outbuff);
  // write to png
  img->encodePNG(&outbuff, &outlen);
  std::ofstream pngout("out.png", std::ofstream::binary);
  pngout.write(outbuff, outlen);
  pngout.close();
  free(outbuff);
  // all done
  delete img;
  return 0;
}
#endif
