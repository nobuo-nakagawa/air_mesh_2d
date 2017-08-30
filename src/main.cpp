//
// Copyright(c) 2017 by Nobuo NAKAGAWA @ Polyphony Digital Inc.
//
// We're Hiring!
// http://www.polyphony.co.jp/recruit/
//
#include <cstdio>
#include <cstdint>
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtx/norm.hpp"
#include "delaunay.h"

#if defined(WIN32)
#include <GL/glut.h>
#ifndef _DEBUG
//#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")
#endif // _DEBUG
#elif defined(__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#endif // MACOSX

static const glm::ivec2 WINDOW_SIZE(640, 480);

static const glm::vec2  BOUNDARY_MIN = glm::vec2(-1.4f); // ^
static const glm::vec2  BOUNDARY_MAX = glm::vec2(+1.4f); // |-->

static const glm::vec2  WORLD_MIN = BOUNDARY_MIN + glm::vec2(-0.1f);
static const glm::vec2  WORLD_MAX = BOUNDARY_MAX + glm::vec2(+0.1f);

static const glm::vec2  GRAVITY   = glm::vec2(0.0f, -98.0f);

static const glm::vec4  COLOR_RED  (1.0f, 0.0f, 0.0f, 1.0f);
static const glm::vec4  COLOR_WHITE(1.0f, 1.0f, 1.0f, 1.0f);
static const glm::vec4  COLOR_GRAY (0.5f, 0.5f, 0.5f, 1.0f);

static const GLfloat    MASS = 0.05f;

enum eMenu{
  eMenuDisplayVertex,
  eMenuDisplayEdge,
  eMenuDisplayFace,
  eMenuDisplayAir,
  eMenuMax,
};

struct sParticle {
  glm::vec2 pos;    //         position
  glm::vec2 ppos;   // predict position
  glm::vec2 vel;    //         velocity
  bool      is_fix; // don't move
  sParticle() : pos(), ppos(), is_fix(false) {}
  sParticle(const glm::vec2& in_pos, const glm::vec2& in_ppos, bool in_is_fix) :
    pos(in_pos),
    ppos(in_ppos),
    is_fix(in_is_fix){}
  void set(const sParticle& in) {
    pos    = in.pos;
    ppos   = in.ppos;
    is_fix = in.is_fix;
  }
};

struct sConstraint{
  float      distance;
  glm::vec2* p1;
  glm::vec2* p2;

  sConstraint(glm::vec2* in_p1, glm::vec2* in_p2) :
  distance(0.0f),
  p1(in_p1),
  p2(in_p2){
    glm::vec2 p1_to_p2 = *p1 - *p2;
    distance = glm::length(p1_to_p2);
  }

  void satisfy() {
    glm::vec2 p1_to_p2          = *p2 - *p1;
    float     diff              = glm::length(p1_to_p2) - distance;
    glm::vec2 correction_vector = glm::normalize(p1_to_p2) * diff * 0.5f;
    *p1 +=  correction_vector;
    *p2 += -correction_vector;
  }
};

struct sApplication {
  glm::vec2                viewport_size;
  std::vector<sParticle>   boundaries;
  std::vector<sParticle>   rigid_bodies;
  std::vector<sConstraint> distance_constraint;
  Delaunay                 triangulation;
  bool                     is_display_vertex;
  bool                     is_display_edge;
  bool                     is_display_face;
  bool                     is_display_air;
  GLfloat                  time;
  sApplication() :
    viewport_size(),
    boundaries(),
    is_display_vertex(true),
    is_display_edge(true),
    is_display_face(true),
    is_display_air(true),
    time(0.0f) {}
};

sApplication g_App;

void init_boundaries() {
  static const int32_t w_div   =  2; //10;
  static const int32_t h_div   =  2; //10;
  static const int32_t div_num = (w_div + h_div) * 2 - 4;  // 4 edges
  g_App.boundaries.reserve(div_num);
  static const float   w_dif = (BOUNDARY_MAX.x - BOUNDARY_MIN.x) / (w_div - 1);
  static const float   h_dif = (BOUNDARY_MAX.y - BOUNDARY_MIN.y) / (h_div - 1);
  for(int32_t w = 0; w < w_div; w++) { // top left - top right
    glm::vec2  pos(BOUNDARY_MIN.x + w_dif * w, BOUNDARY_MAX.y);
    glm::vec2 ppos(pos);
    g_App.boundaries.push_back(sParticle(pos, ppos, true));
  }
  for(int32_t h = 1; h < h_div; h++) { // top right - bottom right
    glm::vec2  pos(BOUNDARY_MAX.x, BOUNDARY_MAX.y - h_dif * h);
    glm::vec2 ppos(pos);
    g_App.boundaries.push_back(sParticle(pos, ppos, true));
  }
  for(int32_t w = 1; w < w_div; w++) { // bottom right - bottom left
    glm::vec2  pos(BOUNDARY_MAX.x - w_dif * w, BOUNDARY_MIN.y);
    glm::vec2 ppos(pos);
    g_App.boundaries.push_back(sParticle(pos, ppos, true));
  }
  for(int32_t h = 1; h < h_div - 1; h++) { // bottom left - top left
    glm::vec2  pos(BOUNDARY_MIN.x, BOUNDARY_MIN.y + h_dif * h);
    glm::vec2 ppos(pos);
    g_App.boundaries.push_back(sParticle(pos, ppos, true));
  }
}

void init_rigid_body() {
  static const int32_t rigid_body_w  =  4;
  static const int32_t rigid_body_h  =  4;
  static const int32_t rigid_body_sw =  0;
  static const int32_t rigid_body_sh =  0;
  static const int32_t rigid_body_ew =  2;
  static const int32_t rigid_body_eh =  2;
  static const float   w_dif         = (BOUNDARY_MAX.x - BOUNDARY_MIN.x) / rigid_body_w;
  static const float   h_dif         = (BOUNDARY_MAX.y - BOUNDARY_MIN.y) / rigid_body_h;
  //g_App.rigid_bodies.reserve(rigid_body_w * rigid_body_h * 4);
  for(int32_t w = rigid_body_sw; w < rigid_body_ew; w++) {
    for(int32_t h = rigid_body_sh; h < rigid_body_eh; h++) {
      glm::vec2 offset( w_dif / 4.0f, -h_dif / 4.0f);
      glm::vec2 pos(BOUNDARY_MIN.x + w_dif * w, BOUNDARY_MAX.y - h_dif * h);
      pos += offset;
      glm::vec2 ppos(pos);
      g_App.rigid_bodies.push_back(sParticle(pos, ppos, false)); // left  top
      pos.x += (w_dif - offset.x * 2.0f);
      ppos = pos;
      g_App.rigid_bodies.push_back(sParticle(pos, ppos, false)); // right top
      pos.y -= (h_dif + offset.y * 2.0);
      ppos = pos;
      g_App.rigid_bodies.push_back(sParticle(pos, ppos, false)); // right bottom
      pos.x -= (w_dif - offset.x * 2.0);
      ppos = pos;
      g_App.rigid_bodies.push_back(sParticle(pos, ppos, false)); // left  bottom
    }
  }
  int32_t rigid_bodies_rectangles = g_App.rigid_bodies.size() / 4;
  for (int32_t i = 0; i < rigid_bodies_rectangles; i++) {
    int32_t idx = i * 4;
    g_App.distance_constraint.push_back(sConstraint(&g_App.rigid_bodies[idx + 0].ppos, &g_App.rigid_bodies[idx + 1].ppos)); // p0-p1 
    g_App.distance_constraint.push_back(sConstraint(&g_App.rigid_bodies[idx + 1].ppos, &g_App.rigid_bodies[idx + 2].ppos)); // p1-p2 
    g_App.distance_constraint.push_back(sConstraint(&g_App.rigid_bodies[idx + 2].ppos, &g_App.rigid_bodies[idx + 3].ppos)); // p2-p3 
    g_App.distance_constraint.push_back(sConstraint(&g_App.rigid_bodies[idx + 3].ppos, &g_App.rigid_bodies[idx + 0].ppos)); // p3-p0 
    g_App.distance_constraint.push_back(sConstraint(&g_App.rigid_bodies[idx + 0].ppos, &g_App.rigid_bodies[idx + 2].ppos)); // p0-p2 shear
    g_App.distance_constraint.push_back(sConstraint(&g_App.rigid_bodies[idx + 1].ppos, &g_App.rigid_bodies[idx + 3].ppos)); // p1-p3 shear
  }
}

void create_delaunay_triangles() {
  Delaunay&                     triangulation = g_App.triangulation;
  const std::vector<sParticle>& rigid_bodies  = g_App.rigid_bodies;
  const std::vector<sParticle>& boundaries    = g_App.boundaries;
  int32_t rigid_bodies_size = rigid_bodies.size();
  for (int32_t i = 0; i < rigid_bodies_size; i++) {
    triangulation.add_point(glm::vec3(rigid_bodies[i].pos.x, rigid_bodies[i].pos.y, 0.0f));
  }
  int32_t boundaries_size = boundaries.size();
  for (int32_t i = 0; i < boundaries_size; i++) {
    triangulation.add_point(glm::vec3(boundaries[i].pos.x, boundaries[i].pos.y, 0.0f));
  }
  triangulation.triangulate();
}

void gen_air_mesh() {
  create_delaunay_triangles();
}

void init() {
  init_boundaries();
  init_rigid_body();
  gen_air_mesh();
  g_App.time = (float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f; // to avoid first frame uncollected dt
}

void keyboard(unsigned char key , int x , int y) {
  switch(key){
  case 27: exit(EXIT_SUCCESS); break; // ESC to quit
  }
}

void calc_predict_position(GLfloat dt) {
  std::vector<sParticle>& rigid_bodies = g_App.rigid_bodies;
  Delaunay&               triangulation = g_App.triangulation;
  int32_t rigid_bodies_size = rigid_bodies.size();
  for (int32_t i = 0; i < rigid_bodies_size; i++) {
    if (g_App.rigid_bodies[i].is_fix == false) {
      rigid_bodies[i].vel  = GRAVITY * dt;
      rigid_bodies[i].ppos = rigid_bodies[i].pos  + rigid_bodies[i].vel * dt;
    }
  }
}

void calc_distance_constraint(GLfloat dt) {
  static int32_t iteraion = 10;
  for(int i = 0; i < iteraion; i++){
    std::vector<sConstraint>::iterator constraint;
    for(constraint = g_App.distance_constraint.begin(); constraint != g_App.distance_constraint.end(); constraint++){
      (*constraint).satisfy();
    }
  }
}

void calc_air_mesh_constraint(GLfloat dt) {
  Delaunay& triangulation = g_App.triangulation;
  int32_t tri_sz   = triangulation.triangle_indexes.size();
  bool    is_dirty = false;
  for (int32_t i = 0; i < tri_sz / 3; i++) { // 3 = triangle
    int32_t idx0 = triangulation.triangle_indexes[i*3  ];
    int32_t idx1 = triangulation.triangle_indexes[i*3+1];
    int32_t idx2 = triangulation.triangle_indexes[i*3+2];
    glm::vec2 x0(triangulation.triangle_vertices[idx0].x, triangulation.triangle_vertices[idx0].y);
    glm::vec2 x1(triangulation.triangle_vertices[idx1].x, triangulation.triangle_vertices[idx1].y);
    glm::vec2 x2(triangulation.triangle_vertices[idx2].x, triangulation.triangle_vertices[idx2].y);
    glm::vec2 p  = x1 - x0;
    glm::vec2 q  = x2 - x0;
    GLfloat   C  = p.x * q.y - p.y * q.x;    // Eq.1 C_air in "Air Meshes for Robust Collision Handling" from triangle area : 1/2(AxBy-AyBx) helon's formula
    C *= -1;                                 // negative clock wise triangle
    if (C >= 0.0f) {                         // 
      continue;                              // unilateral constrain (only negative)
    }                                        // Fig.4 in "Air Meshes for Robust Collision Handling"
    glm::vec2 dC[3];                         //   x1______x2   x0, x1, x2  : triangle verticies
    dC[0] = glm::vec2(p.y - q.y, q.x - p.x); //     \    /     
    dC[1] = glm::vec2(      q.y,      -q.x); //     p\  /q     p = x1 - x0 : vector
    dC[2] = glm::vec2(     -p.y,       p.x); //       \/       q = x2 - x0 : vector
                                             //       x0       *perpendicular to the line
    GLfloat w[3];
    GLfloat s = 0.0f;
    for (int32_t k = 0; k < 3; k++) {
      w[k]  = (1.0 / MASS);
      s    += w[k] * glm::length2(dC[k]);    // Eq.8 in "Position based dynamics"
    }
    s = -C / s;                              // negative clock wise triangle
    glm::vec2 dp[3];
    for (int32_t k = 0; k < 3; k++){
      dp[k] = -1.0f * s * w[k] * dC[k];      // Eq.9 in "Position based dynamics"
    }
    x0 += dp[0];
    x1 += dp[1];
    x2 += dp[2];
    int32_t sz = g_App.rigid_bodies.size();                                                // don't move
    if(idx0 < sz) {
      triangulation.set_point_at_index(glm::vec3(x0.x, x0.y, 0.0f), idx0);
      g_App.rigid_bodies[idx0].ppos = x0;
    }
    if(idx1 < sz) {
      triangulation.set_point_at_index(glm::vec3(x1.x, x1.y, 0.0f), idx1);
      g_App.rigid_bodies[idx1].ppos = x1;
    }
    if(idx2 < sz) {
      triangulation.set_point_at_index(glm::vec3(x2.x, x2.y, 0.0f), idx2);
      g_App.rigid_bodies[idx2].ppos = x2;
    }
    //triangulation.triangulate();
  }
}

void calc_position(float dt) {
  std::vector<sParticle>& rigid_bodies  = g_App.rigid_bodies;
  Delaunay&               triangulation = g_App.triangulation;
  int32_t rigid_bodies_size = rigid_bodies.size();
  for (int32_t i = 0; i < rigid_bodies_size; i++) {
    rigid_bodies[i].vel = (rigid_bodies[i].ppos - rigid_bodies[i].pos) / dt;
    rigid_bodies[i].pos = rigid_bodies[i].ppos;
    triangulation.set_point_at_index(glm::vec3(rigid_bodies[i].pos.x, rigid_bodies[i].pos.y, 0.0f), i);
  }
  triangulation.triangulate();
}

void update(GLfloat dt) {
  calc_predict_position(dt);
  calc_distance_constraint(dt);
  calc_air_mesh_constraint(dt);
  calc_position(dt);
}

void idle(void) {
#if 0
  GLfloat time = (float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
  GLfloat dt = time - g_App.time;
  update(dt);
  g_App.time = time;
#else
  update(1.0f / 60.0f);
#endif
  glutPostRedisplay();
}

void display_boundary() {
  int32_t sz = g_App.boundaries.size();
  if (g_App.is_display_edge) {
    glBegin(GL_LINE_LOOP);
      glColor4fv((GLfloat*)&COLOR_GRAY);
      for(int32_t i = 0; i < sz - 1; i++) {
        glVertex2fv((GLfloat*)&g_App.boundaries[i  ].pos);
        glVertex2fv((GLfloat*)&g_App.boundaries[i+1].pos);
      }
    glEnd();
  }
  if (g_App.is_display_vertex) {
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for(int32_t i = 0; i < sz; i++) {
      glColor4fv((GLfloat*)&COLOR_RED);
      glVertex2fv((GLfloat*)&g_App.boundaries[i].pos);
    }
    glEnd();
  }
}

void display_rigid_body() {
  int32_t sz = g_App.rigid_bodies.size();
  if (g_App.is_display_face) {
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0, 1.0);
    static const glm::vec4 orange(1.0f, 0.64f, 0.0f, 1.0f);
    glColor4fv((GLfloat*)&orange);
    for(int32_t i = 0; i < sz / 4; i++) { // 4 = rectangle edge
      int32_t idx = i * 4;
      glBegin(GL_POLYGON);
      glVertex2fv((GLfloat*)&g_App.rigid_bodies[idx  ].pos);
      glVertex2fv((GLfloat*)&g_App.rigid_bodies[idx+1].pos);
      glVertex2fv((GLfloat*)&g_App.rigid_bodies[idx+2].pos);
      glVertex2fv((GLfloat*)&g_App.rigid_bodies[idx+3].pos);
      glEnd();
    }
  }

  if (g_App.is_display_edge) {
    glColor4fv((GLfloat*)&COLOR_GRAY);
    for(int32_t i = 0; i < sz / 4; i++) {
      int32_t idx = i * 4;
      glBegin(GL_LINE_LOOP);
      glVertex2fv((GLfloat*)&g_App.rigid_bodies[idx  ].pos);
      glVertex2fv((GLfloat*)&g_App.rigid_bodies[idx+1].pos);
      glVertex2fv((GLfloat*)&g_App.rigid_bodies[idx+2].pos);
      glVertex2fv((GLfloat*)&g_App.rigid_bodies[idx+3].pos);
      glEnd();
    }
  }
  if (g_App.is_display_vertex) {
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for(int32_t i = 0; i < sz; i++) {
      glColor4fv((GLfloat*)&COLOR_RED);
      glVertex2fv((GLfloat*)&g_App.rigid_bodies[i].pos);
    }
    glEnd();
  }
}

void display_air_mesh() {
  if (g_App.is_display_air) {
    glColor4fv((GLfloat*)&COLOR_GRAY);
    int32_t tri_sz = g_App.triangulation.triangle_indexes.size();
    for (int32_t i = 0; i < tri_sz / 3; i++) {
      int32_t idx0 = g_App.triangulation.triangle_indexes[i*3  ];
      int32_t idx1 = g_App.triangulation.triangle_indexes[i*3+1];
      int32_t idx2 = g_App.triangulation.triangle_indexes[i*3+2];
      int32_t rigid_bodies_size = g_App.rigid_bodies.size();
      if ((idx0 < rigid_bodies_size) && (idx1 < rigid_bodies_size) && (idx2 < rigid_bodies_size)) {
        int32_t group0 = idx0 / 4; // 4 = vertices in rigid body
        int32_t group1 = idx1 / 4;
        int32_t group2 = idx2 / 4;
        if ((group0 == group1) && (group0 == group2)) { // same rigid body
          continue;                                     // remove triangles in rigid body
        }
      }
      glm::vec2 p0 = glm::vec2(g_App.triangulation.triangle_vertices[idx0].x, g_App.triangulation.triangle_vertices[idx0].y);
      glm::vec2 p1 = glm::vec2(g_App.triangulation.triangle_vertices[idx1].x, g_App.triangulation.triangle_vertices[idx1].y);
      glm::vec2 p2 = glm::vec2(g_App.triangulation.triangle_vertices[idx2].x, g_App.triangulation.triangle_vertices[idx2].y);
      glBegin(GL_LINE_LOOP);
        glVertex2fv((GLfloat*)&p0);
        glVertex2fv((GLfloat*)&p1);
        glVertex2fv((GLfloat*)&p2);
      glEnd();
    }
  }
}

void display(void){
  glClearColor(COLOR_WHITE.r, COLOR_WHITE.g, COLOR_WHITE.b, COLOR_WHITE.a);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluOrtho2D(WORLD_MIN.x, WORLD_MAX.x, WORLD_MIN.y, WORLD_MAX.y); // left, right, bottom, top

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  display_boundary();
  display_rigid_body();
  display_air_mesh();

  glutSwapBuffers();

  unsigned int error_num = glGetError();
  if (error_num != GL_NO_ERROR) {
    printf("glGetError() = [%s]\n", gluErrorString(error_num));
  }
  assert(glGetError() == GL_NO_ERROR);
}

void menu(int value){
  switch(value) {
  case eMenuDisplayVertex: g_App.is_display_vertex = !g_App.is_display_vertex; break;
  case eMenuDisplayEdge:   g_App.is_display_edge   = !g_App.is_display_edge;   break;
  case eMenuDisplayFace:   g_App.is_display_face   = !g_App.is_display_face;   break;
  case eMenuDisplayAir:    g_App.is_display_air    = !g_App.is_display_air;    break;
  }
};

void reshape(int in_w, int in_h){
  if (in_w < WINDOW_SIZE.x || in_h < WINDOW_SIZE.y) {
    glutReshapeWindow(WINDOW_SIZE.x, WINDOW_SIZE.y);
  }
  g_App.viewport_size.x = static_cast<float>(in_w);
  g_App.viewport_size.y = static_cast<float>(in_h);
  glutPostRedisplay();
}

int main(int argc, char * argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(WINDOW_SIZE.x, WINDOW_SIZE.y);
  glutCreateWindow("Air Mesh for Robust Collision Handling");

  init();

  glutDisplayFunc(display);
//  glutReshapeFunc(reshape);
  //glutMouseFunc(mouse);
  //glutMotionFunc(motion);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  //glutSpecialFunc(special);

  glutCreateMenu(menu);
  glutAddMenuEntry("diaply vertex   [on,off]", eMenuDisplayVertex);
  glutAddMenuEntry("diaply edge     [on,off]", eMenuDisplayEdge);
  glutAddMenuEntry("diaply face     [on,off]", eMenuDisplayFace);
  glutAddMenuEntry("diaply air mesh [on,off]", eMenuDisplayAir);
  glutAttachMenu(GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
