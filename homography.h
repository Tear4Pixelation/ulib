#pragma once

#include "geom.h"

// 3x3 projective transform (homography), i.e. the perspective generalization of Transform2D - needed for
//  document scanning, where a photo of a page taken at an angle has to be flattened to a head-on view.
// Transform2D deliberately stays affine (6 elements, 12 mults) since that covers everything the renderer
//  does; this is a separate class so nothing pays for the extra row unless it actually needs it.
// Storage is column-major, matching Transform2D:
// [ m0 m3 m6 ] [x]
// [ m1 m4 m7 ] [y]
// [ m2 m5 m8 ] [1]
// so x' = (m0*x + m3*y + m6)/w, y' = (m1*x + m4*y + m7)/w, with w = m2*x + m5*y + m8

class Transform3D
{
public:
  real m[9];

  Transform3D() : m{1, 0, 0, 0, 1, 0, 0, 0, 1} {}
  Transform3D(real m0, real m1, real m2, real m3, real m4, real m5, real m6, real m7, real m8)
      : m{m0, m1, m2, m3, m4, m5, m6, m7, m8} {}
  explicit Transform3D(const Transform2D& tf)
      : m{tf.m[0], tf.m[1], 0, tf.m[2], tf.m[3], 0, tf.m[4], tf.m[5], 1} {}

  // row, col accessor for readability in the math below
  real at(int row, int col) const { return m[col*3 + row]; }
  real& at(int row, int col) { return m[col*3 + row]; }

  Point map(const Point& p) const;
  real* asArray() { return &m[0]; }
  const real* asArray() const { return &m[0]; }

  bool isAffine() const { return m[2] == 0 && m[5] == 0 && m[8] == 1; }
  bool isValid() const;  // all finite and not degenerate
  Transform3D inverse() const;

  friend Transform3D operator*(const Transform3D& a, const Transform3D& b);
  friend bool approxEq(const Transform3D& a, const Transform3D& b, real eps);

  // quad corners are always ordered TL, TR, BR, BL (i.e. CW starting from the origin corner)
  // maps the unit square (0,0), (1,0), (1,1), (0,1) onto quad
  static Transform3D unitToQuad(const Point quad[4]);
  static Transform3D quadToUnit(const Point quad[4]);
  static Transform3D quadToQuad(const Point src[4], const Point dst[4]);
};
