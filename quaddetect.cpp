#include "quaddetect.h"
#include "imagewarp.h"  // isConvexQuad

#include <stdint.h>
#include <algorithm>
#include <vector>

// Tuning constants.  These are all "usually right" thresholds rather than exact quantities, so they are
//  collected here to be retuned against real photos rather than scattered through the code.
static const int TARGET_LONG_EDGE = 480;    // everything below runs on an image this size, so it is fast
static const float MIN_EDGE_MAGNITUDE = 10;  // Sobel |gx|+|gy| on 0-255 grey: a step of about 4 levels
static const float NOISE_MULTIPLE = 1.5f;   // ...or this many times the median gradient, if that is larger
static const int THETA_SPREAD_DEG = 6;      // how far either side of the gradient direction to vote
static const int NUM_THETA = 720;           // half a degree per bin, over the full circle (see findLines)
static const float THETA_STEP = 360.0f/NUM_THETA;
static const int NMS_RADIUS = 6;            // peak suppression window in the accumulator
static const int MAX_LINES_PER_FAMILY = 24; // candidates kept for each of the two orientations
static const float MIN_LINE_VOTES = 0.12f;  // a candidate line needs this many votes, relative to the short side
static const float MIN_SEPARATION = 0.25f;  // opposite edges must be this far apart, relative to the image
static const float MIN_AREA_FRACTION = 0.15f;  // the page must fill at least this much of the photo
static const int SUPPORT_SEARCH = 2;        // how far off a side an edge pixel may be and still support it
static const float SUPPORT_ANGLE_DEG = 22;  // ...and how well its gradient must point into the page
static const int CONTRAST_OFFSET = 4;       // the page side of an edge is sampled this far in, the outside this far out
static const float MIN_CONTRAST = 3;        // grey levels the inside must be brighter than the outside
static const float MIN_SIDE_SUPPORT = 0.25f;  // every side must be supported along this fraction of its length
static const float MIN_TOTAL_SUPPORT = 2.2f;   // ...and the four sides together this much (out of 4)
static const float EXTENSION_FRACTION = 0.15f;  // how far past each corner to look for an edge running on
static const float OVERRUN_PENALTY = 0.6f;  // ...and how much of a side's support that costs it
static const float AREA_WEIGHT = 0.3f;      // tie break toward the larger quad when support is equal
static const float REFIT_DISTANCE = 2.5f;   // how close an edge pixel must be to count toward a refit
static const float REFIT_ANGLE_DEG = 20;    // ...and how well its gradient must agree with the line
static const int MIN_REFIT_POINTS = 20;

struct GreyImage {
  std::vector<float> samples;
  int width = 0;
  int height = 0;
  int scale = 1;  // how many source pixels per sample, so corners can be scaled back up

  float at(int x, int y) const {
    x = std::min(width - 1, std::max(0, x));
    y = std::min(height - 1, std::max(0, y));
    return samples[size_t(y)*width + x];
  }
};

// Sobel output, kept per pixel because side scoring looks pixels up by position
struct GradientImage {
  std::vector<float> gx, gy;
  std::vector<uint8_t> isEdge;  // above threshold *and* a local maximum across the edge
  int width = 0;
  int height = 0;
};

// a pixel that sits on an edge, with the direction of its gradient (pointing toward the brighter side)
struct EdgePoint {
  float x = 0;
  float y = 0;
  float angleDeg = 0;
};

struct HoughLine {
  float thetaDeg = 0;  // angle of the line's *normal*
  float rho = 0;       // x*cos(theta) + y*sin(theta)
  int votes = 0;
};

// Box-average down to something small.  Detection does not need detail - it needs the page edge to
//  dominate, and throwing away resolution is the cheapest way to suppress paper texture and print.
static GreyImage downscaleToGrey(const unsigned int* pixels, int width, int height)
{
  GreyImage grey;
  int longEdge = std::max(width, height);
  grey.scale = std::max(1, (longEdge + TARGET_LONG_EDGE - 1)/TARGET_LONG_EDGE);
  grey.width = std::max(1, width/grey.scale);
  grey.height = std::max(1, height/grey.scale);
  grey.samples.assign(size_t(grey.width)*grey.height, 0.0f);
  const unsigned char* bytes = (const unsigned char*)pixels;
  for(int y = 0; y < grey.height; ++y) {
    for(int x = 0; x < grey.width; ++x) {
      int sum = 0, count = 0;
      for(int sy = y*grey.scale; sy < std::min((y + 1)*grey.scale, height); ++sy) {
        for(int sx = x*grey.scale; sx < std::min((x + 1)*grey.scale, width); ++sx) {
          const unsigned char* pixel = bytes + (size_t(sy)*width + sx)*4;
          sum += (pixel[0]*77 + pixel[1]*150 + pixel[2]*29) >> 8;
          ++count;
        }
      }
      grey.samples[size_t(y)*grey.width + x] = count > 0 ? float(sum)/count : 0.0f;
    }
  }
  return grey;
}

static void blur3x3(GreyImage& grey)
{
  if(grey.width < 3 || grey.height < 3)
    return;
  std::vector<float> blurred = grey.samples;
  for(int y = 1; y < grey.height - 1; ++y) {
    for(int x = 1; x < grey.width - 1; ++x) {
      const float* row = &grey.samples[size_t(y)*grey.width + x];
      int stride = grey.width;
      float sum = 4*row[0]
          + 2*(row[-1] + row[1] + row[-stride] + row[stride])
          + row[-stride - 1] + row[-stride + 1] + row[stride - 1] + row[stride + 1];
      blurred[size_t(y)*grey.width + x] = sum/16;
    }
  }
  grey.samples.swap(blurred);
}

// Sobel, then thin the edges to one pixel (Canny style non-maximum suppression) and threshold them.
// The threshold is absolute, with a noise floor, rather than a percentile of the photo's own gradients:
//  a percentile is set by whatever is strongest in the frame - wood grain, a checked tablecloth,
//  handwriting - and on a white desk the page outline is far weaker than any of those, so a relative
//  cut drops it.  Thinning is what keeps an absolute threshold from admitting whole blurred bands.
static GradientImage computeGradients(const GreyImage& grey, std::vector<EdgePoint>& edgePoints)
{
  GradientImage gradient;
  gradient.width = grey.width;
  gradient.height = grey.height;
  size_t nSamples = size_t(grey.width)*grey.height;
  gradient.gx.assign(nSamples, 0.0f);
  gradient.gy.assign(nSamples, 0.0f);
  gradient.isEdge.assign(nSamples, 0);
  if(grey.width < 8 || grey.height < 8)
    return gradient;
  std::vector<float> magnitude(nSamples, 0.0f);
  for(int y = 1; y < grey.height - 1; ++y) {
    for(int x = 1; x < grey.width - 1; ++x) {
      const float* row = &grey.samples[size_t(y)*grey.width + x];
      int stride = grey.width;
      float gx = (row[-stride + 1] + 2*row[1] + row[stride + 1])
               - (row[-stride - 1] + 2*row[-1] + row[stride - 1]);
      float gy = (row[stride - 1] + 2*row[stride] + row[stride + 1])
               - (row[-stride - 1] + 2*row[-stride] + row[-stride + 1]);
      size_t index = size_t(y)*grey.width + x;
      gradient.gx[index] = gx;
      gradient.gy[index] = gy;
      magnitude[index] = std::abs(gx) + std::abs(gy);
    }
  }

  // the median gradient is the noise level (most of any photo is flat), so sensor and JPEG noise on a
  //  dim photo raise the threshold instead of flooding the accumulator
  std::vector<float> sorted;
  sorted.reserve(nSamples/16 + 1);
  for(size_t ii = 0; ii < nSamples; ii += 16)
    sorted.push_back(magnitude[ii]);
  std::nth_element(sorted.begin(), sorted.begin() + sorted.size()/2, sorted.end());
  float threshold = std::max(MIN_EDGE_MAGNITUDE, NOISE_MULTIPLE*sorted[sorted.size()/2]);

  for(int y = 2; y < grey.height - 2; ++y) {
    for(int x = 2; x < grey.width - 2; ++x) {
      size_t index = size_t(y)*grey.width + x;
      float value = magnitude[index];
      if(value < threshold)
        continue;
      // neighbors across the edge, along the gradient direction quantized to 45 degrees
      float gx = gradient.gx[index], gy = gradient.gy[index];
      int stepX = 0, stepY = 0;
      float absX = std::abs(gx), absY = std::abs(gy);
      if(absX > 2.414f*absY) stepX = 1;
      else if(absY > 2.414f*absX) stepY = 1;
      else { stepX = 1;  stepY = (gx > 0) == (gy > 0) ? 1 : -1; }
      float before = magnitude[size_t(y - stepY)*grey.width + x - stepX];
      float after = magnitude[size_t(y + stepY)*grey.width + x + stepX];
      if(value < before || value <= after)
        continue;
      gradient.isEdge[index] = 1;
      EdgePoint point;
      point.x = float(x);
      point.y = float(y);
      // the gradient points across the edge, so it is the normal direction of the line through it
      point.angleDeg = float(std::atan2(gy, gx)*180/M_PI);
      edgePoints.push_back(point);
    }
  }
  return gradient;
}

// Gradient-directed Hough: each edge pixel only votes for orientations near its own gradient, which is
//  both much cheaper than voting for all of them and much less prone to inventing lines out of texture.
// The accumulator covers a full 360 degrees rather than 180, so a line keeps its *polarity*: its normal
//  points toward the brighter side.  A page edge is always bright on the page side, and keeping the two
//  polarities apart matters most where they sit next to each other - the page's drop shadow puts a
//  dark-to-light edge two or three pixels outside the light-to-dark page edge, and with 180 degrees the
//  two merged into one peak that sat on the shadow and missed the page.
static std::vector<HoughLine> findLines(const std::vector<EdgePoint>& edgePoints, int width, int height)
{
  std::vector<HoughLine> lines;
  if(edgePoints.empty())
    return lines;
  std::vector<float> cosTable(NUM_THETA), sinTable(NUM_THETA);
  for(int t = 0; t < NUM_THETA; ++t) {
    cosTable[t] = float(std::cos(degToRad(t*THETA_STEP)));
    sinTable[t] = float(std::sin(degToRad(t*THETA_STEP)));
  }
  int diagonal = int(std::sqrt(float(width*width + height*height))) + 1;
  int numRho = 2*diagonal + 1;
  std::vector<int32_t> accumulator(size_t(NUM_THETA)*numRho, 0);
  int spreadBins = int(THETA_SPREAD_DEG/THETA_STEP);
  for(size_t ii = 0; ii < edgePoints.size(); ++ii) {
    const EdgePoint& point = edgePoints[ii];
    int centerBin = int(std::floor(point.angleDeg/THETA_STEP + 0.5f));
    for(int offset = -spreadBins; offset <= spreadBins; ++offset) {
      int t = ((centerBin + offset) % NUM_THETA + NUM_THETA) % NUM_THETA;
      int rhoIndex = int(std::floor(point.x*cosTable[t] + point.y*sinTable[t] + 0.5f)) + diagonal;
      if(rhoIndex >= 0 && rhoIndex < numRho)
        ++accumulator[size_t(t)*numRho + rhoIndex];
    }
  }

  int minVotes = std::max(int(std::min(width, height)*MIN_LINE_VOTES), 8);
  // local maxima only, so one strong edge yields one line rather than a smear of near duplicates.  Theta
  //  wraps around: bin 0 and bin NUM_THETA-1 are neighbors.
  for(int t = 0; t < NUM_THETA; ++t) {
    for(int r = 0; r < numRho; ++r) {
      int votes = accumulator[size_t(t)*numRho + r];
      if(votes < minVotes)
        continue;
      bool isPeak = true;
      for(int dt = -NMS_RADIUS; dt <= NMS_RADIUS && isPeak; ++dt) {
        int nt = ((t + dt) % NUM_THETA + NUM_THETA) % NUM_THETA;
        for(int dr = -NMS_RADIUS; dr <= NMS_RADIUS; ++dr) {
          int nr = r + dr;
          if((dt == 0 && dr == 0) || nr < 0 || nr >= numRho)
            continue;
          int other = accumulator[size_t(nt)*numRho + nr];
          // ties go to one side only, so a flat-topped peak still yields exactly one line
          if(other > votes || (other == votes && (dt < 0 || (dt == 0 && dr < 0)))) { isPeak = false; break; }
        }
      }
      if(isPeak) {
        HoughLine line;
        line.thetaDeg = t*THETA_STEP;
        line.rho = float(r - diagonal);
        line.votes = votes;
        lines.push_back(line);
      }
    }
  }
  return lines;
}

// smallest difference between two directions, in degrees (0 - 180)
static float angleDelta(float aDeg, float bDeg)
{
  float delta = std::fmod(std::abs(aDeg - bDeg), 360.0f);
  return delta > 180 ? 360 - delta : delta;
}

// A candidate line, with how well each stretch of it looks like the edge of a page lying on its +normal
//  side.  Positions along the line are integer steps t from its foot point; the prefix sums make "how
//  much of the segment between these two corners is supported" an O(1) lookup, so every combination of
//  lines can be scored.
struct LineSupport {
  HoughLine line;
  Point foot;        // rho*normal
  Point normal;      // toward the brighter side, i.e. into the page
  Point direction;   // along the line
  int tMin = 0;      // t of the first entry in the prefix sums
  std::vector<int> insideImage;   // prefix count of samples that fall inside the image
  std::vector<int> supported;     // ...that look like a page edge
};

// Is there an edge here with the page (the brighter side) in direction inward?  Two tests, both needed:
//  an edge pixel nearby whose gradient points into the page, and the page side actually being brighter
//  a few pixels away.  The second is what rejects ruled lines, text lines and pencil marks *inside* the
//  page: they have edges of either polarity, but the paper either side of them is the same brightness.
static bool looksLikePageEdge(const GreyImage& grey, const GradientImage& gradient, Point at, Point inward)
{
  static const float minCos = float(std::cos(degToRad(SUPPORT_ANGLE_DEG)));
  bool foundEdge = false;
  for(int step = -SUPPORT_SEARCH; step <= SUPPORT_SEARCH && !foundEdge; ++step) {
    int x = int(std::floor(at.x + inward.x*step + 0.5));
    int y = int(std::floor(at.y + inward.y*step + 0.5));
    if(x < 0 || y < 0 || x >= gradient.width || y >= gradient.height)
      continue;
    size_t index = size_t(y)*gradient.width + x;
    if(!gradient.isEdge[index])
      continue;
    float gx = gradient.gx[index], gy = gradient.gy[index];
    float length = std::sqrt(gx*gx + gy*gy);
    foundEdge = length > 0 && (gx*inward.x + gy*inward.y) >= minCos*length;
  }
  if(!foundEdge)
    return false;
  Point inside = at + inward*CONTRAST_OFFSET, outside = at - inward*CONTRAST_OFFSET;
  float insideValue = grey.at(int(std::floor(inside.x + 0.5)), int(std::floor(inside.y + 0.5)));
  float outsideValue = grey.at(int(std::floor(outside.x + 0.5)), int(std::floor(outside.y + 0.5)));
  return insideValue - outsideValue >= MIN_CONTRAST;
}

static LineSupport measureLineSupport(const GreyImage& grey, const GradientImage& gradient, const HoughLine& line)
{
  LineSupport support;
  support.line = line;
  real theta = degToRad(line.thetaDeg);
  support.normal = Point(std::cos(theta), std::sin(theta));
  support.direction = Point(-support.normal.y, support.normal.x);
  support.foot = support.normal*line.rho;
  int reach = int(std::sqrt(real(grey.width*grey.width + grey.height*grey.height))) + 2;
  support.tMin = -reach;
  int nSteps = 2*reach + 1;
  support.insideImage.assign(nSteps + 1, 0);
  support.supported.assign(nSteps + 1, 0);
  for(int step = 0; step < nSteps; ++step) {
    Point at = support.foot + support.direction*real(step + support.tMin);
    bool inside = at.x >= 1 && at.y >= 1 && at.x < grey.width - 2 && at.y < grey.height - 2;
    bool isEdge = inside && looksLikePageEdge(grey, gradient, at, support.normal);
    support.insideImage[step + 1] = support.insideImage[step] + (inside ? 1 : 0);
    support.supported[step + 1] = support.supported[step] + (isEdge ? 1 : 0);
  }
  return support;
}

// prefix sum range [first, last) for the stretch of the line between positions ta and tb
static void stepRange(const LineSupport& support, real ta, real tb, int* first, int* last)
{
  int nSteps = int(support.insideImage.size()) - 1;
  *first = std::max(0, std::min(nSteps, int(std::ceil(std::min(ta, tb))) - support.tMin));
  *last = std::max(0, std::min(nSteps, int(std::floor(std::max(ta, tb))) - support.tMin + 1));
}

// How much of the side from corner a to corner b looks like page edge, from 0 to 1, less a penalty if
//  the edge carries on past the corners.  A page edge *ends* at the page's corners; a table edge, a
//  plank seam or a tablecloth stripe runs on, and that is the cheapest way to tell them apart when one
//  happens to lie close to where the page is.
static float sideSupport(const LineSupport& support, Point a, Point b)
{
  real ta = dot(a - support.foot, support.direction), tb = dot(b - support.foot, support.direction);
  int first = 0, last = 0;
  stepRange(support, ta, tb, &first, &last);
  int length = last - first;
  if(length <= 0)
    return 0;
  int inside = support.insideImage[last] - support.insideImage[first];
  int count = support.supported[last] - support.supported[first];
  // A side running out of the frame is judged on the part that is in it, so a page cut off by the photo's
  //  edge is still found - but at least half of the side must be visible, or a single short stub of edge
  //  could pass for a whole side.
  float fraction = float(count)/std::max(inside, (length + 1)/2);

  real extension = std::max(real(6), std::abs(tb - ta)*EXTENSION_FRACTION);
  real lowT = std::min(ta, tb), highT = std::max(ta, tb);
  float overrun = 0;
  int extFirst = 0, extLast = 0;
  stepRange(support, lowT - extension, lowT - 1, &extFirst, &extLast);
  int extInside = support.insideImage[extLast] - support.insideImage[extFirst];
  if(extInside > 0)
    overrun += float(support.supported[extLast] - support.supported[extFirst])/extInside;
  stepRange(support, highT + 1, highT + extension, &extFirst, &extLast);
  extInside = support.insideImage[extLast] - support.insideImage[extFirst];
  if(extInside > 0)
    overrun += float(support.supported[extLast] - support.supported[extFirst])/extInside;
  return fraction - OVERRUN_PENALTY*overrun/2;
}

// Replace a Hough line with a least squares fit through the edge pixels that support it.  The
//  accumulator only resolves the angle to half a degree, and half a degree across the long edge of a
//  4000 pixel photo is tens of pixels of error at the corners - far more than the sub-pixel accuracy a
//  straight line fit gives for free.  Only pixels between the side's two corners count, so a table edge
//  or a ruled line that happens to be collinear beyond the page cannot pull the fit.
static bool refineLine(const std::vector<EdgePoint>& edgePoints, HoughLine& line, Point endA, Point endB)
{
  const float originalTheta = line.thetaDeg;
  HoughLine fitted = line;
  bool refined = false;
  Point along = endB - endA;
  real alongLengthSq = dot(along, along);
  if(alongLengthSq <= 0)
    return false;
  // Iterate: the first pass only sees the pixels within REFIT_DISTANCE of the accumulator's estimate,
  //  and half a degree of angular error across a long edge is already wider than that window - so the
  //  first fit is biased toward the middle of the edge.  Re-centering the window on the improved line
  //  picks up the ends that were missed, and it converges in two or three passes.
  for(int pass = 0; pass < 3; ++pass) {
    real theta = degToRad(fitted.thetaDeg);
    real normalX = std::cos(theta), normalY = std::sin(theta);
    real sumX = 0, sumY = 0, sumXX = 0, sumYY = 0, sumXY = 0;
    int count = 0;
    for(size_t ii = 0; ii < edgePoints.size(); ++ii) {
      const EdgePoint& point = edgePoints[ii];
      if(std::abs(point.x*normalX + point.y*normalY - fitted.rho) > REFIT_DISTANCE)
        continue;
      if(angleDelta(point.angleDeg, fitted.thetaDeg) > REFIT_ANGLE_DEG)
        continue;
      // stay off the corners too: near them the other side's edge pixels are within reach
      real position = dot(Point(point.x, point.y) - endA, along)/alongLengthSq;
      if(position < 0.03 || position > 0.97)
        continue;
      sumX += point.x;      sumY += point.y;
      sumXX += real(point.x)*point.x;
      sumYY += real(point.y)*point.y;
      sumXY += real(point.x)*point.y;
      ++count;
    }
    if(count < MIN_REFIT_POINTS)
      break;
    real meanX = sumX/count, meanY = sumY/count;
    real varX = sumXX/count - meanX*meanX;
    real varY = sumYY/count - meanY*meanY;
    real covXY = sumXY/count - meanX*meanY;
    // principal axis of the supporting points is the line direction; total least squares rather than a
    //  y-on-x fit, because the page edges can be near vertical
    real axisAngle = 0.5*std::atan2(2*covXY, varX - varY);
    real fitNormalX = -std::sin(axisAngle), fitNormalY = std::cos(axisAngle);
    // the axis has no sign; keep the normal pointing the way the line's did, toward the page
    if(fitNormalX*normalX + fitNormalY*normalY < 0) {
      fitNormalX = -fitNormalX;
      fitNormalY = -fitNormalY;
    }
    real fitTheta = std::atan2(fitNormalY, fitNormalX)*180/M_PI;
    if(fitTheta < 0)
      fitTheta += 360;
    real fitRho = meanX*fitNormalX + meanY*fitNormalY;
    fitted.thetaDeg = float(fitTheta);
    fitted.rho = float(fitRho);
    refined = true;
  }
  // if the fit wandered a long way from where the accumulator put it, it latched onto something else
  if(!refined || angleDelta(fitted.thetaDeg, originalTheta) > 5)
    return false;
  line.thetaDeg = fitted.thetaDeg;
  line.rho = fitted.rho;
  return true;
}

// two points on the line, for lineIntersection()
static void lineToPoints(const HoughLine& line, Point* onLine, Point* along)
{
  real theta = degToRad(line.thetaDeg);
  real cosT = std::cos(theta), sinT = std::sin(theta);
  *onLine = Point(line.rho*cosT, line.rho*sinT);
  *along = *onLine + Point(-sinT, cosT)*1000;
}

static Point intersectLines(const HoughLine& first, const HoughLine& second)
{
  Point firstOn, firstAlong, secondOn, secondAlong;
  lineToPoints(first, &firstOn, &firstAlong);
  lineToPoints(second, &secondOn, &secondAlong);
  return lineIntersection(firstOn, firstAlong, secondOn, secondAlong);
}

// corners TL, TR, BR, BL from left, right, top, bottom lines; false if they do not make a usable quad
static bool quadFromEdges(const HoughLine& left, const HoughLine& right, const HoughLine& top,
    const HoughLine& bottom, int width, int height, Point quad[4])
{
  quad[0] = intersectLines(left, top);
  quad[1] = intersectLines(right, top);
  quad[2] = intersectLines(right, bottom);
  quad[3] = intersectLines(left, bottom);
  for(int ii = 0; ii < 4; ++ii) {
    if(quad[ii].isNaN())
      return false;
  }
  // reject a quad that ran far outside the photo, which means the lines were not really page edges
  real marginX = width*0.25, marginY = height*0.25;
  for(int ii = 0; ii < 4; ++ii) {
    if(quad[ii].x < -marginX || quad[ii].x > width + marginX
        || quad[ii].y < -marginY || quad[ii].y > height + marginY)
      return false;
  }
  if(!isConvexQuad(quad))
    return false;
  // the orientation must be TL, TR, BR, BL: clockwise on screen, i.e. positive cross with y down
  if(cross(quad[1] - quad[0], quad[2] - quad[1]) <= 0)
    return false;
  // shoelace area, as a sanity check that we found the page and not some detail printed on it
  real area = 0;
  for(int ii = 0; ii < 4; ++ii)
    area += cross(quad[ii], quad[(ii + 1) % 4]);
  return std::abs(area)/2 >= width*height*MIN_AREA_FRACTION;
}

static real quadArea(const Point quad[4])
{
  real area = 0;
  for(int ii = 0; ii < 4; ++ii)
    area += cross(quad[ii], quad[(ii + 1) % 4]);
  return std::abs(area)/2;
}

// keep the strongest few lines of one family, so combining them stays cheap even on a busy background
static void keepStrongest(std::vector<HoughLine>& lines)
{
  std::sort(lines.begin(), lines.end(), [](const HoughLine& a, const HoughLine& b){ return a.votes > b.votes; });
  if(lines.size() > size_t(MAX_LINES_PER_FAMILY))
    lines.resize(MAX_LINES_PER_FAMILY);
}

// Choose the page outline from the candidate lines.  Every combination of a left, right, top and bottom
//  line is a candidate quad, scored by how much of each of its four *sides* - the segments between its
//  corners, not the infinite lines - looks like a page edge.  This replaced picking the outermost line of
//  each orientation, which is right on a plain dark background and wrong on nearly any real one: a table
//  edge, a plank seam, a laptop, a tablecloth pattern or a second sheet beyond the page is always further
//  out than the page, so on real photos the detector returned the clutter.  Lines from the clutter do not
//  survive the side test: extended to meet them, the page's real edges run through empty table and lose
//  their support, and the clutter line itself runs on past the corners.
static bool pickPageQuad(const GreyImage& grey, const GradientImage& gradient,
    const std::vector<HoughLine>& lines, HoughLine edges[4], Point bestQuad[4])
{
  // Families by where the normal - toward the brighter side, so into the page - points: a left edge's
  //  points right (0 degrees), a top edge's down (90), and so on.
  std::vector<HoughLine> families[4];  // left, top, right, bottom
  for(const HoughLine& line : lines) {
    int family = int(std::floor(std::fmod(line.thetaDeg + 45, 360.0f)/90)) & 3;
    families[family].push_back(line);
  }
  std::vector<LineSupport> supports[4];
  for(int family = 0; family < 4; ++family) {
    keepStrongest(families[family]);
    if(families[family].empty())
      return false;
    for(const HoughLine& line : families[family])
      supports[family].push_back(measureLineSupport(grey, gradient, line));
  }

  // how far the image center lies on the page side of a line; for two opposite edges, the sum is the
  //  distance between them, wherever the page is in the frame
  Point center(grey.width/2.0, grey.height/2.0);
  auto inwardOffset = [&](const LineSupport& support) { return dot(support.normal, center) - support.line.rho; };
  real imageArea = real(grey.width)*grey.height;
  float bestScore = -1;
  for(const LineSupport& left : supports[0]) {
    for(const LineSupport& right : supports[2]) {
      if(inwardOffset(left) + inwardOffset(right) < grey.width*MIN_SEPARATION)
        continue;
      for(const LineSupport& top : supports[1]) {
        for(const LineSupport& bottom : supports[3]) {
          if(inwardOffset(top) + inwardOffset(bottom) < grey.height*MIN_SEPARATION)
            continue;
          Point quad[4];
          if(!quadFromEdges(left.line, right.line, top.line, bottom.line, grey.width, grey.height, quad))
            continue;
          float sides[4] = {
            sideSupport(left, quad[3], quad[0]),
            sideSupport(right, quad[1], quad[2]),
            sideSupport(top, quad[0], quad[1]),
            sideSupport(bottom, quad[2], quad[3])
          };
          float total = 0, weakest = 1;
          for(float side : sides) {
            total += side;
            weakest = std::min(weakest, side);
          }
          if(weakest < MIN_SIDE_SUPPORT || total < MIN_TOTAL_SUPPORT)
            continue;
          float score = total + AREA_WEIGHT*float(quadArea(quad)/imageArea);
          if(score > bestScore) {
            bestScore = score;
            edges[0] = left.line;  edges[1] = right.line;  edges[2] = top.line;  edges[3] = bottom.line;
            std::copy(quad, quad + 4, bestQuad);
          }
        }
      }
    }
  }
  return bestScore > 0;
}

void defaultDocumentQuad(int width, int height, real insetFraction, Point quad[4])
{
  real insetX = width*insetFraction, insetY = height*insetFraction;
  quad[0] = Point(insetX, insetY);
  quad[1] = Point(width - insetX, insetY);
  quad[2] = Point(width - insetX, height - insetY);
  quad[3] = Point(insetX, height - insetY);
}

bool detectDocumentQuad(const unsigned int* pixels, int width, int height, Point quad[4])
{
  if(!pixels || width < 16 || height < 16)
    return false;
  GreyImage grey = downscaleToGrey(pixels, width, height);
  blur3x3(grey);
  std::vector<EdgePoint> edgePoints;
  GradientImage gradient = computeGradients(grey, edgePoints);
  std::vector<HoughLine> lines = findLines(edgePoints, grey.width, grey.height);
  HoughLine edges[4];  // left, right, top, bottom
  Point coarse[4];
  if(!pickPageQuad(grey, gradient, lines, edges, coarse))
    return false;
  // refit each side between the corners the coarse lines give it; keeps the Hough estimate if the refit
  //  is not trustworthy
  refineLine(edgePoints, edges[0], coarse[3], coarse[0]);
  refineLine(edgePoints, edges[1], coarse[1], coarse[2]);
  refineLine(edgePoints, edges[2], coarse[0], coarse[1]);
  refineLine(edgePoints, edges[3], coarse[2], coarse[3]);
  Point found[4];
  if(!quadFromEdges(edges[0], edges[1], edges[2], edges[3], grey.width, grey.height, found))
    std::copy(coarse, coarse + 4, found);
  // back to source pixels, clamped to the image; the +0.5 puts the sample at its pixel's center
  for(int ii = 0; ii < 4; ++ii) {
    quad[ii].x = std::min(real(width), std::max(real(0), (found[ii].x + 0.5)*grey.scale));
    quad[ii].y = std::min(real(height), std::max(real(0), (found[ii].y + 0.5)*grey.scale));
  }
  return true;
}
