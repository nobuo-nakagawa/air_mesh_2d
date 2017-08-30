// Triangulate Efficient Triangulation Algorithm Suitable for Terrain Modelling
// or 
// An Algorithm for Interpolating Irregularly-Spaced Data with Applications in Terrain Modelling
//
// http://paulbourke.net/papers/triangulate/
//
// based on implementations by by Gilles Dumoulin
// http://paulbourke.net/papers/triangulate/cpp.zip
//
#include "delaunay.h"

/*
bool point_in_triangle(const glm::vec2 & p, const XYZ & p0, const XYZ & p1, const XYZ& p2) {
  float A = 0.5f * (-p1.y * p2.x + p0.y * (-p1.x + p2.x) + p0.x * (p1.y - p2.y) + p1.x * p2.y);
  float sign = A < 0.0f ? -1.0f : 1.0f;
  float s = (p0.y * p2.x - p0.x * p2.y + (p2.y - p0.y) * p.x + (p0.x - p2.x) * p.y) * sign;
  float t = (p0.x * p1.y - p0.y * p1.x + (p0.y - p1.y) * p.x + (p1.x - p0.x) * p.y) * sign;

  return s > 0.0f && t > 0.0f && (s + t) < 2.0f * A * sign;
}
*/

void Delaunay::reset(){
  vertices.clear();
  triangles.clear();
  //triangleMesh.clear();
  ntri = 0;
}

int Delaunay::add_point(const glm::vec3& pos){
  return add_point(pos.x, pos.y, pos.z);
}

int Delaunay::add_point(float x, float y, float z){
  XYZI v;
  v.x = x;
  v.y = y;
  v.z = z;
  v.i = vertices.size();
  vertices.push_back(v);
  return vertices.size();
}

int Delaunay::triangulate(){

  if (vertices.size() < 3) {
    return NULL;
  }

  int nv = vertices.size();

  // make clone not to destroy vertices
  std::vector<XYZI> verticesTemp = vertices;
  qsort( &verticesTemp[0], verticesTemp.size(), sizeof( XYZI ), XYZICompare );

  // vertices required for Triangulate
  std::vector<XYZ> verticesXYZ;

  // copy XYZIs to XYZ
  for (int i = 0; i < nv; i++) {
    XYZ v;
    v.x = verticesTemp.at(i).x;
    v.y = verticesTemp.at(i).y;
    v.z = verticesTemp.at(i).z;
    verticesXYZ.push_back(v);
  }

  // add 3 emptly slots, required by the Triangulate call
  verticesXYZ.push_back(XYZ());
  verticesXYZ.push_back(XYZ());
  verticesXYZ.push_back(XYZ());

  // allocate space for triangle indices
  triangles.resize(3 * nv);

  triangulate_internal(nv, &verticesXYZ[0], &triangles[0], ntri);

  triangle_vertices.clear();
  for (int i = 0; i < nv; i++){
    triangle_vertices.push_back(glm::vec3(vertices[i].x, vertices[i].y, vertices[i].z));
  }
  for (int i = 0; i < ntri; i++) {
    triangle_indexes.push_back(verticesTemp.at(triangles[ i ].p1).i);
    triangle_indexes.push_back(verticesTemp.at(triangles[ i ].p2).i);
    triangle_indexes.push_back(verticesTemp.at(triangles[ i ].p3).i);
  }
  return ntri;
}

void Delaunay::set_point_at_index(const glm::vec3& p, int index){
  if (index >= 0 && index < vertices.size()){
    XYZI pp; pp.x = p.x; pp.y = p.y; pp.z = p.z; pp.i = index;
    vertices[index] = pp;
  }
//  triangulate();
}

/*
glm::vec3 Delaunay::getPointNear(glm::vec3& pos, float minimumDist, int & index){

	XYZI ret;
	XYZ p; p.x = pos.x; p.y = pos.y; p.z = pos.z;
	float minDist = FLT_MAX;
	for(int i = 0; i < vertices.size() ; i++){
        glm::vec3 vtx(vertices[i].x, vertices[i].y, vertices[i].z);
        glm::vec3 pos(p.x, p.y, p.z);
		float d = glm::distance(vtx, pos);
		if(d < minDist ) {
			minDist = d;
			index = i;
			ret = vertices[i];
		}
	}
	if (minDist > minimumDist){
		ret.x = ret.y = ret.z = 0.0f;
		index = -1;
	}
  return glm::vec3(ret.x, ret.y, ret.z);
}

ITRIANGLE Delaunay::getTriangleAtIndex(int index){
	if (index >= 0 && index < ntri){
		return triangles[index];
	}
	return ITRIANGLE();
}

void Delaunay::removePointAtIndex(int index){
	if (index >= 0 && index < vertices.size()){
		vertices.erase(vertices.begin()+index);
	}
	triangulate();
}

vector<glm::vec3> Delaunay::getPointsForITriangle(ITRIANGLE t){
 std::vector<glm::vec3> pts;
  pts.push_back( glm::vec3(vertices[t.p1].x, vertices[t.p1].y, vertices[t.p1].z));
  pts.push_back( glm::vec3(vertices[t.p2].x, vertices[t.p2].y, vertices[t.p2].z));
  pts.push_back( glm::vec3(vertices[t.p3].x, vertices[t.p3].y, vertices[t.p3].z));
  return pts;
}

ITRIANGLE Delaunay::getTriangleForPos(glm::vec2& pos){
  ITRIANGLE ti;
  for(int i = 0; i < ntri ; i++){
    XYZ p0; p0.x = vertices[triangles[i].p1].x; p0.y = vertices[triangles[i].p1].y; p0.z = vertices[triangles[i].p1].z;
    XYZ p1; p1.x = vertices[triangles[i].p2].x; p1.y = vertices[triangles[i].p2].y; p1.z = vertices[triangles[i].p2].z;
    XYZ p2; p2.x = vertices[triangles[i].p3].x; p2.y = vertices[triangles[i].p3].y; p2.z = vertices[triangles[i].p3].z;
    bool inside = point_in_triangle(pos, p0, p1, p2);
    if (inside) {
      ti = triangles[i];
      break;
    }
  }
  return ti;
}
*/

int XYZICompare(const void *v1, const void *v2){
  XYZI *p1, *p2;

  p1 = (XYZI*)v1;
  p2 = (XYZI*)v2;
  if(p1->x < p2->x)
    return(-1);
  else if(p1->x > p2->x)
    return(1);
  else
    return(0);
}

////////////////////////////////////////////////////////////////////////
// circum_circle() :
//   Return true if a point (xp,yp) is inside the circumcircle made up
//   of the points (x1,y1), (x2,y2), (x3,y3)
//   The circumcircle centre is returned in (xc,yc) and the radius r
//   Note : A point on the edge is inside the circumcircle
////////////////////////////////////////////////////////////////////////
int Delaunay::circum_circle(double xp, double yp, double x1, double y1, double x2, double y2, double x3, double y3, double& xc, double& yc, double& r){
  double m1, m2, mx1, mx2, my1, my2;
  double dx, dy, rsqr, drsqr;

  if (abs(y1 - y2) < EPSILON && abs(y2 - y3) < EPSILON) { // Check for coincident points
    return(false);
  }

  if (abs(y2-y1) < EPSILON) {
    m2 = - (x3 - x2) / (y3 - y2);
    mx2 = (x2 + x3) / 2.0;
    my2 = (y2 + y3) / 2.0;
    xc = (x2 + x1) / 2.0;
    yc = m2 * (xc - mx2) + my2;
  }else if(abs(y3 - y2) < EPSILON){
    m1 = - (x2 - x1) / (y2 - y1);
    mx1 = (x1 + x2) / 2.0;
    my1 = (y1 + y2) / 2.0;
    xc = (x3 + x2) / 2.0;
    yc = m1 * (xc - mx1) + my1;
  }else{
    m1 = - (x2 - x1) / (y2 - y1);
    m2 = - (x3 - x2) / (y3 - y2);
    mx1 = (x1 + x2) / 2.0;
    mx2 = (x2 + x3) / 2.0;
    my1 = (y1 + y2) / 2.0;
    my2 = (y2 + y3) / 2.0;
    xc = (m1 * mx1 - m2 * mx2 + my2 - my1) / (m1 - m2);
    yc = m1 * (xc - mx1) + my1;
  }

  dx = x2 - xc;
  dy = y2 - yc;
  rsqr = dx * dx + dy * dy;
  r = sqrt(rsqr);
  dx = xp - xc;
  dy = yp - yc;
  drsqr = dx * dx + dy * dy;

  return ((drsqr <= rsqr) ? true : false);
}

///////////////////////////////////////////////////////////////////////////////
// triangulate_internal() :
//   Triangulation subroutine
//   Takes as input NV vertices in array pxyz
//   Returned is a list of ntri triangular faces in the array v
//   These triangles are arranged in a consistent clockwise order.
//   The triangle array 'v' should be malloced to 3 * nv
//   The vertex array pxyz must be big enough to hold 3 more points
//   The vertex array must be sorted in increasing x values say
//
//   qsort(p,nv,sizeof(XYZ),XYZCompare);
///////////////////////////////////////////////////////////////////////////////
int Delaunay::triangulate_internal(int nv, XYZ pxyz[], ITRIANGLE v[], int &ntri) {
  int*   complete = NULL;
  IEDGE* edges = NULL;
  IEDGE* p_EdgeTemp;
  int nedge = 0;
  int trimax, emax = 200;
  int status = 0;

  int inside;
  int i, j, k;
  double xp, yp, x1, y1, x2, y2, x3, y3, xc, yc, r;
  double xmin, xmax, ymin, ymax, xmid, ymid;
  double dx, dy, dmax;

  // Allocate memory for the completeness list, flag for each triangle
  trimax = 4 * nv;
  complete = new int[trimax];
  // Allocate memory for the edge list
  edges = new IEDGE[emax];

  // Find the maximum and minimum vertex bounds.
  // This is to allow calculation of the bounding triangle
  xmin = pxyz[0].x;
  ymin = pxyz[0].y;
  xmax = xmin;
  ymax = ymin;
  for (i = 1; i < nv; i++) {
    if (pxyz[i].x < xmin) xmin = pxyz[i].x;
    if (pxyz[i].x > xmax) xmax = pxyz[i].x;
    if (pxyz[i].y < ymin) ymin = pxyz[i].y;
    if (pxyz[i].y > ymax) ymax = pxyz[i].y;
  }
  dx = xmax - xmin;
  dy = ymax - ymin;
  dmax = (dx > dy) ? dx : dy;
  xmid = (xmax + xmin) / 2.0;
  ymid = (ymax + ymin) / 2.0;

  // Set up the supertriangle
  // This is a triangle which encompasses all the sample points.
  // The supertriangle coordinates are added to the end of the
  // vertex list. The supertriangle is the first triangle in
  // the triangle list.
  pxyz[nv+0].x = xmid - 20 * dmax;
  pxyz[nv+0].y = ymid - dmax;
  pxyz[nv+0].z = 0.0;
  pxyz[nv+1].x = xmid;
  pxyz[nv+1].y = ymid + 20 * dmax;
  pxyz[nv+1].z = 0.0;
  pxyz[nv+2].x = xmid + 20 * dmax;
  pxyz[nv+2].y = ymid - dmax;
  pxyz[nv+0].z = 0.0;
  v[0].p1 = nv;
  v[0].p2 = nv+1;
  v[0].p3 = nv+2;
  complete[0] = false;
  ntri = 1;

  // Include each point one at a time into the existing mesh
  for (i = 0; i < nv; i++) {
    xp = pxyz[i].x;
    yp = pxyz[i].y;
    nedge = 0;

    // Set up the edge buffer.
    // If the point (xp,yp) lies inside the circumcircle then the
    // three edges of that triangle are added to the edge buffer
    // and that triangle is removed.
    for (j = 0; j < ntri; j++) {
      if (complete[j]) {
        continue;
      }
      x1 = pxyz[v[j].p1].x;
      y1 = pxyz[v[j].p1].y;
      x2 = pxyz[v[j].p2].x;
      y2 = pxyz[v[j].p2].y;
      x3 = pxyz[v[j].p3].x;
      y3 = pxyz[v[j].p3].y;
      inside = circum_circle(xp, yp, x1, y1, x2, y2, x3, y3, xc, yc, r);
      //    if (xc + r < xp)

      // Suggested
      // if (xc + r + EPSILON < xp)
      if (xc < xp && ((xp-xc)*(xp-xc)) > r) {
        complete[j] = true;
      }

      if (inside) {
        // Check that we haven't exceeded the edge list size
        if (nedge + 3 >= emax){
          emax += 100;
          p_EdgeTemp = new IEDGE[emax];
          for (int i = 0; i < nedge; i++) { // Fix by John Bowman
            p_EdgeTemp[i] = edges[i];
          }
          delete []edges;
          edges = p_EdgeTemp;
        }

        edges[nedge+0].p1 = v[j].p1;
        edges[nedge+0].p2 = v[j].p2;
        edges[nedge+1].p1 = v[j].p2;
        edges[nedge+1].p2 = v[j].p3;
        edges[nedge+2].p1 = v[j].p3;
        edges[nedge+2].p2 = v[j].p1;
        nedge += 3;
        v[j] = v[ntri-1];
        complete[j] = complete[ntri-1];
        ntri--;
        j--;
      }
    }

    // Tag multiple edges
    // Note: if all triangles are specified anticlockwise then all
    // interior edges are opposite pointing in direction.
    for (j = 0; j < nedge - 1; j++) {
      for (k = j + 1; k < nedge; k++) {
        if ((edges[j].p1 == edges[k].p2) && (edges[j].p2 == edges[k].p1)) {
          edges[j].p1 = -1;
          edges[j].p2 = -1;
          edges[k].p1 = -1;
          edges[k].p2 = -1;
        }
        // Shouldn't need the following, see note above */
        if ((edges[j].p1 == edges[k].p1) && (edges[j].p2 == edges[k].p2)) {
          edges[j].p1 = -1;
          edges[j].p2 = -1;
          edges[k].p1 = -1;
          edges[k].p2 = -1;
        }
      }
    }

    // Form new triangles for the current point
    // Skipping over any tagged edges.
    // All edges are arranged in clockwise order.
    for (j = 0; j < nedge; j++) {
      if (edges[j].p1 < 0 || edges[j].p2 < 0) {
        continue;
      }

      if (ntri >= trimax) {
        break;
      }

      v[ntri].p1 = edges[j].p1;
      v[ntri].p2 = edges[j].p2;
      v[ntri].p3 = i;
      complete[ntri] = false;
      ntri++;
    }

    if (ntri >= trimax) {
      break;
    }
  }

  // Remove triangles with supertriangle vertices
  // These are triangles which have a vertex number greater than nv
  for (i = 0; i < ntri; i++) {
    if (v[i].p1 >= nv || v[i].p2 >= nv || v[i].p3 >= nv) {
      v[i] = v[ntri-1];
      ntri--;
      i--;
    }
  }

  delete[] edges;
  delete[] complete;
  return 0;
}