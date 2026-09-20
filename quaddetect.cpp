#include "quaddetect.h"
#include "imagewarp.h"  // isConvexQuad

#include <stdint.h>
#include <vector>

// Tuning constants.  These are all "usually right" thresholds rather than exact quantities, so they are
//  collected here to be retuned against real photos rather than scattered through the code.
static const int TARGET_LONG_EDGE = 480;    // everything below runs on an image this size, so it is fast
static const float EDGE_FRACTION = 0.08f;   // fraction of pixels kept as edges, by gradient magnitude
static const float MIN_EDGE_STRENGTH = 0.1f;   // ...but never weaker than this fraction of the strongest
static const int THETA_SPREAD_DEG = 6;      // how far either side of the gradient direction to vote
static const int NUM_THETA = 360;           // half a degree per bin
static const float THETA_STEP = 180.0f/NUM_THETA;
static const int NMS_RADIUS = 6;            // peak suppression window in the accumulator
static const float MIN_VOTE_FRACTION = 0.2f;   // a line must poll this fraction of the best line
static const float MIN_SEPARATION = 0.25f;  // opposite edges must be this far apart, relative to the image
static const float MIN_AREA_FRACTION = 0.15f;  // the page must fill at least this much of the photo
static const float REFIT_DISTANCE = 2.5f;   // how close an edge pixel must be to count toward a refit
static const float REFIT_ANGLE_DEG = 20;    // ...and how well its gradient must agree with the line
static const int MIN_REFIT_POINTS = 20;

struct GreyImage {
  std::vector<float> samples;
  int width = 0;
  int height = 0;
  int scale = 1;  // how many source pixels per sample, so corners can be scaled back up
};

// a pixel that sits on an edge, with the direction of that edge's normal
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

// Sobel, then keep only the pixels that look like they are on an edge.
static std::vector<EdgePoint> findEdgePoints(const GreyImage& grey)
{
  std::vector<EdgePoint> edgePoints;
  if(grey.width < 8 || grey.height < 8)
    return edgePoints;
  std::vector<float> magnitude(size_t(grey.width)*grey.height, 0.0f);
  std::vector<float> angleDeg(size_t(grey.width)*grey.height, 0.0f);
  float maxMagnitude = 0;
  for(int y = 1; y < grey.height - 1; ++y) {
    for(int x = 1; x < grey.width - 1; ++x) {
      const float* row = &grey.samples[size_t(y)*grey.width + x];
      int stride = grey.width;
      float gx = (row[-stride + 1] + 2*row[1] + row[stride + 1])
               - (row[-stride - 1] + 2*row[-1] + row[stride - 1]);
      float gy = (row[stride - 1] + 2*row[stride] + row[stride + 1])
               - (row[-stride - 1] + 2*row[-stride] + row[-stride + 1]);
      size_t index = size_t(y)*grey.width + x;
      magnitude[index] = std::abs(gx) + std::abs(gy);
      // the gradient points across the edge, so it is the normal direction of the line through it
      angleDeg[index] = float(std::atan2(gy, gx)*180/M_PI);
      maxMagnitude = std::max(maxMagnitude, magnitude[index]);
    }
  }
  if(maxMagnitude <= 0)
    return edgePoints;

  // Keep the strongest EDGE_FRACTION of pixels - a percentile rather than a fixed threshold, so a low
  //  contrast photo is not simply discarded.  The floor matters as much as the percentile: a page
  //  outline is only about 1% of the pixels, so "top 8%" on a clean photo reaches well down into the
  //  flat interior, and flat pixels have a gradient of (0,0), whose atan2 is 0 - which would send the
  //  entire background to the same orientation bin and bury the real edges under invented lines.
  int histogram[256] = {0};
  size_t nSamples = magnitude.size();
  for(size_t ii = 0; ii < nSamples; ++ii)
    ++histogram[int(magnitude[ii]*255/maxMagnitude)];
  size_t wanted = size_t(nSamples*EDGE_FRACTION);
  size_t counted = 0;
  int cutoffBin = 255;
  for(int bin = 255; bin >= 0; --bin) {
    counted += histogram[bin];
    if(counted >= wanted) { cutoffBin = bin; break; }
  }
  float threshold = std::max(cutoffBin*maxMagnitude/255, maxMagnitude*MIN_EDGE_STRENGTH);

  for(int y = 1; y < grey.height - 1; ++y) {
    for(int x = 1; x < grey.width - 1; ++x) {
      size_t index = size_t(y)*grey.width + x;
      if(magnitude[index] < threshold)
        continue;
      EdgePoint point;
      point.x = float(x);
      point.y = float(y);
      point.angleDeg = angleDeg[index];
      edgePoints.push_back(point);
    }
  }
  return edgePoints;
}

// gradient-directed Hough: each edge pixel only votes for orientations near its own gradient, which is
//  both much cheaper than voting for all of them and much less prone to inventing lines out of texture
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
    int centerBin = int(point.angleDeg/THETA_STEP + 0.5f);
    for(int offset = -spreadBins; offset <= spreadBins; ++offset) {
      // theta and theta+180 describe the same line, and using the normalized bin's own cos/sin makes
      //  rho come out with the matching sign automatically
      int t = (centerBin + offset) % NUM_THETA;
      if(t < 0)
        t += NUM_THETA;
      int rhoIndex = int(point.x*cosTable[t] + point.y*sinTable[t] + 0.5f) + diagonal;
      if(rhoIndex >= 0 && rhoIndex < numRho)
        ++accumulator[size_t(t)*numRho + rhoIndex];
    }
  }

  int bestVotes = 0;
  for(size_t ii = 0; ii < accumulator.size(); ++ii)
    bestVotes = std::max(bestVotes, accumulator[ii]);
  if(bestVotes <= 0)
    return lines;
  int minVotes = std::max(int(bestVotes*MIN_VOTE_FRACTION), 8);

  // local maxima only, so one strong edge yields one line rather than a smear of near duplicates
  for(int t = 0; t < NUM_THETA; ++t) {
    for(int r = 0; r < numRho; ++r) {
      int votes = accumulator[size_t(t)*numRho + r];
      if(votes < minVotes)
        continue;
      bool isPeak = true;
      for(int dt = -NMS_RADIUS; dt <= NMS_RADIUS && isPeak; ++dt) {
        for(int dr = -NMS_RADIUS; dr <= NMS_RADIUS; ++dr) {
          int nt = t + dt, nr = r + dr;
          if(nt < 0 || nt >= NUM_THETA || nr < 0 || nr >= numRho || (dt == 0 && dr == 0))
            continue;
          if(accumulator[size_t(nt)*numRho + nr] > votes) { isPeak = false; break; }
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

// smallest difference between two orientations, treating a direction and its opposite as the same
static float orientationDelta(float aDeg, float bDeg)
{
  float delta = std::fmod(std::abs(aDeg - bDeg), 180.0f);
  return delta > 90 ? 180 - delta : delta;
}

// Replace a Hough line with a least squares fit through the edge pixels that support it.  The
//  accumulator only resolves the angle to half a degree, and half a degree across the long edge of a
//  4000 pixel photo is tens of pixels of error at the corners - far more than the sub-pixel accuracy a
//  straight line fit gives for free.
static bool refineLine(const std::vector<EdgePoint>& edgePoints, HoughLine& line)
{
  const float originalTheta = line.thetaDeg;
  HoughLine fitted = line;
  bool refined = false;
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
      if(orientationDelta(point.angleDeg, fitted.thetaDeg) > REFIT_ANGLE_DEG)
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
    real fitTheta = std::atan2(fitNormalY, fitNormalX)*180/M_PI;
    real fitRho = meanX*fitNormalX + meanY*fitNormalY;
    // keep the same representation as the input, so callers that already normalized theta stay consistent
    while(fitTheta - fitted.thetaDeg > 90) { fitTheta -= 180;  fitRho = -fitRho; }
    while(fitted.thetaDeg - fitTheta > 90) { fitTheta += 180;  fitRho = -fitRho; }
    fitted.thetaDeg = float(fitTheta);
    fitted.rho = float(fitRho);
    refined = true;
  }
  // if the fit wandered a long way from where the accumulator put it, it latched onto something else
  if(!refined || orientationDelta(fitted.thetaDeg, originalTheta) > 10)
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

// Split the lines into the two families and take the outermost of each.  "Outermost" is measured along
//  each line's own normal, relative to the image center, so it still works on a page photographed askew.
static bool pickPageEdges(const std::vector<HoughLine>& lines, int width, int height, HoughLine edges[4])
{
  real centerX = width/2.0, centerY = height/2.0;
  bool haveVertical = false, haveHorizontal = false;
  real minVerticalOffset = 0, maxVerticalOffset = 0, minHorizontalOffset = 0, maxHorizontalOffset = 0;
  for(size_t ii = 0; ii < lines.size(); ++ii) {
    HoughLine line = lines[ii];
    // normalize the near-vertical family to theta in [-45, 45) so that lines at 1 and 179 degrees, which
    //  are the same orientation, do not end up in opposite halves of the range
    if(line.thetaDeg >= 135) {
      line.thetaDeg -= 180;
      line.rho = -line.rho;
    }
    real theta = degToRad(line.thetaDeg);
    real offset = line.rho - (centerX*std::cos(theta) + centerY*std::sin(theta));
    bool isVertical = line.thetaDeg < 45;  // a vertical line has a horizontal normal
    if(isVertical) {
      if(!haveVertical) {
        edges[0] = edges[1] = line;
        minVerticalOffset = maxVerticalOffset = offset;
        haveVertical = true;
      }
      else if(offset < minVerticalOffset) { edges[0] = line;  minVerticalOffset = offset; }
      else if(offset > maxVerticalOffset) { edges[1] = line;  maxVerticalOffset = offset; }
    }
    else {
      if(!haveHorizontal) {
        edges[2] = edges[3] = line;
        minHorizontalOffset = maxHorizontalOffset = offset;
        haveHorizontal = true;
      }
      else if(offset < minHorizontalOffset) { edges[2] = line;  minHorizontalOffset = offset; }
      else if(offset > maxHorizontalOffset) { edges[3] = line;  maxHorizontalOffset = offset; }
    }
  }
  if(!haveVertical || !haveHorizontal)
    return false;
  // if one line was picked as both sides of a pair, we only found one edge of that pair
  return maxVerticalOffset - minVerticalOffset > width*MIN_SEPARATION
      && maxHorizontalOffset - minHorizontalOffset > height*MIN_SEPARATION;
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
  std::vector<EdgePoint> edgePoints = findEdgePoints(grey);
  std::vector<HoughLine> lines = findLines(edgePoints, grey.width, grey.height);
  HoughLine edges[4];  // left, right, top, bottom
  if(!pickPageEdges(lines, grey.width, grey.height, edges))
    return false;
  for(int ii = 0; ii < 4; ++ii)
    refineLine(edgePoints, edges[ii]);  // keeps the Hough estimate if the refit is not trustworthy

  Point leftOn, leftAlong, rightOn, rightAlong, topOn, topAlong, bottomOn, bottomAlong;
  lineToPoints(edges[0], &leftOn, &leftAlong);
  lineToPoints(edges[1], &rightOn, &rightAlong);
  lineToPoints(edges[2], &topOn, &topAlong);
  lineToPoints(edges[3], &bottomOn, &bottomAlong);
  Point found[4] = {
    lineIntersection(leftOn, leftAlong, topOn, topAlong),
    lineIntersection(rightOn, rightAlong, topOn, topAlong),
    lineIntersection(rightOn, rightAlong, bottomOn, bottomAlong),
    lineIntersection(leftOn, leftAlong, bottomOn, bottomAlong)
  };
  for(int ii = 0; ii < 4; ++ii) {
    if(found[ii].isNaN())
      return false;
  }
  if(!isConvexQuad(found))
    return false;
  // shoelace area, as a sanity check that we found the page and not some detail printed on it
  real area = 0;
  for(int ii = 0; ii < 4; ++ii)
    area += cross(found[ii], found[(ii + 1) % 4]);
  area = std::abs(area)/2;
  if(area < grey.width*grey.height*MIN_AREA_FRACTION)
    return false;
  // reject a quad that ran far outside the photo, which means the lines were not really page edges
  real marginX = grey.width*0.25, marginY = grey.height*0.25;
  for(int ii = 0; ii < 4; ++ii) {
    if(found[ii].x < -marginX || found[ii].x > grey.width + marginX
        || found[ii].y < -marginY || found[ii].y > grey.height + marginY)
      return false;
  }
  // back to source pixels, clamped to the image; the +0.5 puts the sample at its pixel's center
  for(int ii = 0; ii < 4; ++ii) {
    quad[ii].x = std::min(real(width), std::max(real(0), (found[ii].x + 0.5)*grey.scale));
    quad[ii].y = std::min(real(height), std::max(real(0), (found[ii].y + 0.5)*grey.scale));
  }
  return true;
}
