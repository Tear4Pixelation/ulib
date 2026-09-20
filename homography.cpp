#include "homography.h"

// The square-to-quad closed form is Heckbert's ("Fundamentals of Texture Mapping and Image Warping", 1989);
//  quad-to-quad is then just unitToQuad(dst) * unitToQuad(src)^-1.  Doing it this way instead of solving an
//  8x8 system keeps it short and avoids needing a general linear solver in ulib.

Point Transform3D::map(const Point& p) const
{
  real w = m[2]*p.x + m[5]*p.y + m[8];
  if(w == 0)
    return Point(NaN, NaN);
  real invw = 1/w;
  return Point((m[0]*p.x + m[3]*p.y + m[6])*invw, (m[1]*p.x + m[4]*p.y + m[7])*invw);
}

bool Transform3D::isValid() const
{
  real magnitude = 0;
  for(int ii = 0; ii < 9; ++ii) {
    if(!std::isfinite(m[ii]))
      return false;
    magnitude = std::max(magnitude, std::abs(m[ii]));
  }
  if(magnitude == 0)
    return false;
  // a degenerate (rank < 3) homography has zero determinant; the matrix is only defined up to scale, so
  //  the tolerance has to be derived from the matrix magnitude rather than being an absolute epsilon
  real det = at(0,0)*(at(1,1)*at(2,2) - at(1,2)*at(2,1))
           - at(0,1)*(at(1,0)*at(2,2) - at(1,2)*at(2,0))
           + at(0,2)*(at(1,0)*at(2,1) - at(1,1)*at(2,0));
  return std::abs(det) > 1e-12*magnitude*magnitude*magnitude;
}

Transform3D Transform3D::inverse() const
{
  // adjugate matrix, in row-major order (adjRC = cofactor for output row R, col C)
  real adj00 = at(1,1)*at(2,2) - at(1,2)*at(2,1);
  real adj01 = at(0,2)*at(2,1) - at(0,1)*at(2,2);
  real adj02 = at(0,1)*at(1,2) - at(0,2)*at(1,1);
  real adj10 = at(1,2)*at(2,0) - at(1,0)*at(2,2);
  real adj11 = at(0,0)*at(2,2) - at(0,2)*at(2,0);
  real adj12 = at(0,2)*at(1,0) - at(0,0)*at(1,2);
  real adj20 = at(1,0)*at(2,1) - at(1,1)*at(2,0);
  real adj21 = at(0,1)*at(2,0) - at(0,0)*at(2,1);
  real adj22 = at(0,0)*at(1,1) - at(0,1)*at(1,0);
  real det = at(0,0)*adj00 + at(0,1)*adj10 + at(0,2)*adj20;
  // a homography is only defined up to scale, so the unnormalized adjugate would already be a valid
  //  inverse; we still divide by det to keep the bottom-right entry near 1 and the entries in range
  if(det != 0) {
    real invdet = 1/det;
    adj00 *= invdet;  adj01 *= invdet;  adj02 *= invdet;
    adj10 *= invdet;  adj11 *= invdet;  adj12 *= invdet;
    adj20 *= invdet;  adj21 *= invdet;  adj22 *= invdet;
  }
  return Transform3D(adj00, adj10, adj20, adj01, adj11, adj21, adj02, adj12, adj22);  // column-major
}

Transform3D operator*(const Transform3D& a, const Transform3D& b)
{
  Transform3D out;
  for(int row = 0; row < 3; ++row) {
    for(int col = 0; col < 3; ++col)
      out.at(row, col) = a.at(row, 0)*b.at(0, col) + a.at(row, 1)*b.at(1, col) + a.at(row, 2)*b.at(2, col);
  }
  return out;
}

bool approxEq(const Transform3D& a, const Transform3D& b, real eps)
{
  // normalize both by their last element, since the matrices are only defined up to scale
  if(a.m[8] == 0 || b.m[8] == 0)
    return false;
  for(int ii = 0; ii < 9; ++ii) {
    if(!approxEq(a.m[ii]/a.m[8], b.m[ii]/b.m[8], eps))
      return false;
  }
  return true;
}

Transform3D Transform3D::unitToQuad(const Point quad[4])
{
  // solving for the row-major coefficients of
  //   x' = (xu*u + xv*v + x1)/(wu*u + wv*v + 1),  y' = (yu*u + yv*v + y1)/(wu*u + wv*v + 1)
  real sumx = quad[0].x - quad[1].x + quad[2].x - quad[3].x;
  real sumy = quad[0].y - quad[1].y + quad[2].y - quad[3].y;
  real xu, xv, x1, yu, yv, y1, wu, wv;
  if(sumx == 0 && sumy == 0) {
    // opposite edges are parallel, i.e. the quad is a parallelogram and the mapping is affine
    xu = quad[1].x - quad[0].x;  xv = quad[2].x - quad[1].x;  x1 = quad[0].x;
    yu = quad[1].y - quad[0].y;  yv = quad[2].y - quad[1].y;  y1 = quad[0].y;
    wu = 0;  wv = 0;
  }
  else {
    real dx1 = quad[1].x - quad[2].x, dx2 = quad[3].x - quad[2].x;
    real dy1 = quad[1].y - quad[2].y, dy2 = quad[3].y - quad[2].y;
    real den = dx1*dy2 - dx2*dy1;
    if(den == 0)  // three or more corners collinear
      return Transform3D(NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN);
    real invden = 1/den;
    wu = (sumx*dy2 - dx2*sumy)*invden;
    wv = (dx1*sumy - sumx*dy1)*invden;
    xu = quad[1].x - quad[0].x + wu*quad[1].x;
    xv = quad[3].x - quad[0].x + wv*quad[3].x;
    x1 = quad[0].x;
    yu = quad[1].y - quad[0].y + wu*quad[1].y;
    yv = quad[3].y - quad[0].y + wv*quad[3].y;
    y1 = quad[0].y;
  }
  return Transform3D(xu, yu, wu, xv, yv, wv, x1, y1, 1);  // column-major
}

Transform3D Transform3D::quadToUnit(const Point quad[4])
{
  return unitToQuad(quad).inverse();
}

Transform3D Transform3D::quadToQuad(const Point src[4], const Point dst[4])
{
  return unitToQuad(dst) * unitToQuad(src).inverse();
}
