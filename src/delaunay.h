// Triangulate Efficient Triangulation Algorithm Suitable for Terrain Modelling
// or 
// An Algorithm for Interpolating Irregularly-Spaced Data with Applications in Terrain Modelling
//
// http://paulbourke.net/papers/triangulate/
//
// based on implementations by by Gilles Dumoulin
// http://paulbourke.net/papers/triangulate/cpp.zip
//
#pragma once
#ifndef DELAUNY_H
#define DELAUNY_H

#include <cstdint>
#include <vector>
#include "glm/glm.hpp"

const double EPSILON = 0.000001;

struct ITRIANGLE {
  int32_t p1, p2, p3;
};

struct IEDGE {
  int32_t p1, p2;
};

struct XYZ{
  double x, y, z;
};

struct XYZI{
  double  x, y, z;
  int32_t       i; // index
};

int XYZICompare(const void* v1, const void* v2);

class Delaunay {
public:
  void    reset();
  int32_t add_point(const glm::vec3& pos);
  int32_t add_point(float x, float y, float z);
  int32_t triangulate();

  void    set_point_at_index(const glm::vec3& p, int index);
/*
  glm::vec3         getPointNear(glm::vec3& pos, float minDist, int& index); //returns actual point AND index to point
  ITRIANGLE         getTriangleForPos(glm::vec2& pos); //returns ITRIANGLE(0,0,0) if none found!
  void              removePointAtIndex(int index); //invalidates triangles and mesh
  void              setPointAtIndex(glm::vec3& p, int index); //invalidates mesh
  vector<glm::vec3> getPointsForITriangle(ITRIANGLE t);
  ITRIANGLE         getTriangleAtIndex(int index);
*/
  std::vector<glm::vec3> triangle_vertices;
  std::vector<int32_t>   triangle_indexes;

private:
  std::vector<XYZI>       vertices; // only input of triangulate();
  std::vector<ITRIANGLE> triangles; // output of triangulate();
  int                         ntri; // num triangles

  int circum_circle(double xp, double yp, double x1, double y1, double x2, double y2, double x3, double y3, double& xc, double& yc, double& r);
  int triangulate_internal(int nv, XYZ pxyz[], ITRIANGLE v[], int &ntri);
};

#endif // DELAUNY_H
