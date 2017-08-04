//----------------------------------------------------------------------------
// UD Ray V6

// 2016
// * internal image format is unsigned bytes instead of float -- for texture compatibility
// * glfw instead of glut

// 2014
// * Cube map / solid background

// 2012
// * Added Euler rotations for .obj objects

// November, 2008:  
// * Added world-to-camera transform for spheres in parse_scene_file()
// * Added VectHomogeneousToRegular() and calls to it in parse_scene_file(), glm_transform()
 
// copyright 2014, University of Delaware
// Christopher Rasmussen
//----------------------------------------------------------------------------

#include "udray.hh"

// Include standard headers
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

//----------------------------------------------------------------------------
// Globals
//----------------------------------------------------------------------------

vector < GLMmodel * > model_list;
vector < Surface * > model_surf_list;   // material properties associated with .obj models (to avoid changing glm.hh)
vector < Sphere * > sphere_list;    // material properties are linked inside each sphere struct
vector < Light * > light_list;

Camera *ray_cam;

Ray *eye_ray;
int image_i, image_j;
bool wrote_image;

int maxlevel;          // maximum depth of ray recursion
double minweight;      // minimum fractional contribution to color
double rayeps;         // round-off error tolerance     

// cubemap

unsigned char *background_cubemap_im = NULL;
int background_cubemap_w, background_cubemap_h;
float fbackground_cubemap_sidelength;
int background_cubemap_sidelength;

// solid (use when cubemap image is NULL)

float background_r = 0.0;
float background_g = 0.0;
float background_b = 0.0;

//----------------------------------------------------------------------------
// Functions
//----------------------------------------------------------------------------

// cross product v3 = v1 x v2

void VectCross(Vect v1, Vect v2, Vect v3)
{
  v3[X] = v1[Y]*v2[Z] - v1[Z]*v2[Y];
  v3[Y] = v1[Z]*v2[X] - v1[X]*v2[Z];
  v3[Z] = v1[X]*v2[Y] - v1[Y]*v2[X];
}

//----------------------------------------------------------------------------

void VectPrint(Vect v)
{
  printf("%.2lf %.2lf %.2lf\n", v[X], v[Y], v[Z]);
}

//----------------------------------------------------------------------------

// dst = src

void VectCopy(Vect dst, Vect src)
{
  dst[X] = src[X];
  dst[Y] = src[Y];
  dst[Z] = src[Z];
}

//----------------------------------------------------------------------------

// scaled addition v3 = t * v1 + v2

void VectAddS(double t, Vect v1, Vect v2, Vect v3)
{
  v3[X] = t * v1[X] + v2[X];
  v3[Y] = t * v1[Y] + v2[Y];
  v3[Z] = t * v1[Z] + v2[Z];
}

//----------------------------------------------------------------------------

// v3 = v1 - v2

void VectSub(Vect v1, Vect v2, Vect v3)
{
  v3[X] = v1[X] - v2[X];
  v3[Y] = v1[Y] - v2[Y];
  v3[Z] = v1[Z] - v2[Z];
}

//----------------------------------------------------------------------------

// vector length

double VectMag(Vect v)
{
  return sqrt(v[X]*v[X] + v[Y]*v[Y] + v[Z]*v[Z]);
}

//----------------------------------------------------------------------------

// make vector have unit length; return original length

double VectUnit(Vect v)
{
  double mag;

  mag = VectMag(v);
  v[X] /= mag;
  v[Y] /= mag;
  v[Z] /= mag;

  return mag;
}

//----------------------------------------------------------------------------

// negate all components of vector

void VectNegate(Vect v, Vect vneg)
{
  vneg[X] = -v[X];
  vneg[Y] = -v[Y];
  vneg[Z] = -v[Z];
}

//----------------------------------------------------------------------------

// limit all components to range [low, high]

void VectClamp(Vect v, double low, double high)
{
  for (int i = 0; i < 3; i++) {

    if (v[i] < low)
      v[i] = low;
    else if (v[i] > high)
      v[i] = high;
  }
}

//----------------------------------------------------------------------------

// dot product of two vectors

double VectDotProd(Vect v1, Vect v2)
{
  return v1[X]*v2[X] + v1[Y]*v2[Y] + v1[Z]*v2[Z];
}


//----------------------------------------------------------------------------

// divide 4-vector components by W coord. to go from homogeneous 
// to non-homogeneous coordinates

void VectHomogeneousToRegular(Vect V)
{
  V[X] /= V[W];
  V[Y] /= V[W];
  V[Z] /= V[W];
  V[W] = 1.0;
}

//----------------------------------------------------------------------------

// multiply vector by 4 x 4 matrix transform

// answer remains in homogeneous coordinates; if W coord. may become
// != 1, user must explicitly call VectHomogeneousToRegular()

void TransformVect(Transform M, Vect V, Vect V_prime)
{
  V_prime[X] = M[0]*V[X] + M[4]*V[Y] +  M[8]*V[Z] + M[12]*V[W];
  V_prime[Y] = M[1]*V[X] + M[5]*V[Y] +  M[9]*V[Z] + M[13]*V[W];
  V_prime[Z] = M[2]*V[X] + M[6]*V[Y] + M[10]*V[Z] + M[14]*V[W]; 
  V_prime[W] = M[3]*V[X] + M[7]*V[Y] + M[11]*V[Z] + M[15]*V[W]; 
}

//----------------------------------------------------------------------------

// print a 4 x 4 matrix

void TransformPrint(Transform M)
{
  int r, c;

  for (r = 0; r < 4; r++) {
    for (c = 0; c < 4; c++)
      printf("%6.3lf ", MATRC(M, r, c));
    printf("\n");
  }
  printf("\n");
}

//----------------------------------------------------------------------------

// set 4 x 4 matrix to be identity

void TransformIdentity(Transform M)
{
  int i, r;

  for (i = 0; i < 16; i++)
    M[i] = 0.0;

  for (r = 0; r < 4; r++) 
    MATRC(M, r, r) = 1.0;
}

//----------------------------------------------------------------------------

// multiply two 4 x 4 matrices: M3 = M1 * M2

void TransformProd(Transform M1, Transform M2, Transform M3)
{
  int r, c, k;

  for (r = 0; r < 4; r++) 
    for (c = 0; c < 4; c++) {
      MATRC(M3, r, c) = 0.0;
      for (k = 0; k < 4; k++)  
	MATRC(M3, r, c) += MATRC(M1, r, k) * MATRC(M2, k, c);
    }
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// set up some variables before we begin to draw

void init_raytracing()
{
  ray_cam = make_camera();

  maxlevel = 2;
  minweight = 0.01;
  rayeps = 1e-7;

  eye_ray = make_ray();

  image_i = 0;
  image_j = 0;

  wrote_image = false;
}

//----------------------------------------------------------------------------

// given a pixel location, turn it into an eye ray and trace it to get the color

void raytrace_one_pixel(int i, int j)
{
  double x, y;
  Vect eye_color;

  x = 0.5 + (double) i;   // the "center" of the pixel
  y = 0.5 + (double) j;

  set_pixel_ray_direction(x, y, ray_cam, eye_ray);

  eye_color[R] = eye_color[G] = eye_color[B] = 0.0;
  trace_ray(0, 1.0, eye_ray, eye_color);

  draw_point(i, j, eye_color, ray_cam->im);
}

//----------------------------------------------------------------------------

// figure out the parametric 3-D line in camera coordinates that corresponds to 
// pixel coordinates (x, y), where (0, 0) is the upper-left hand corner of the image

// ray direction should be a unit vector

void set_pixel_ray_direction(double x, double y, Camera *cam, Ray *ray)
{  
  // convert i, j to x frac. and y frac. (where 0, 0 is straight ahead)
  
  double u = x / (double) cam->im->w;  
  double v = y / (double) cam->im->h;

//   ray->orig[X] = cam->eye[X];
//   ray->orig[Y] = cam->eye[Y];
//   ray->orig[Z] = cam->eye[Z];

  ray->orig[X] = ray->orig[Y] = ray->orig[Z] = 0.0;
 
  ray->dir[X] = cam->clip[LEFT] + u * (cam->clip[RIGHT] - cam->clip[LEFT]);
  ray->dir[Y] = cam->clip[TOP] + v * (cam->clip[BOTTOM] - cam->clip[TOP]);
  ray->dir[Z] = -cam->clip[NEAR];

  //  printf("%lf %lf -> %lf %lf %lf\n", x, y, ray->dir[X], ray->dir[Y], ray->dir[Z]);
  
  VectUnit(ray->dir);

  //  printf("unit %lf %lf -> %lf %lf %lf\n\n", x, y, ray->dir[X], ray->dir[Y], ray->dir[Z]);

}

//----------------------------------------------------------------------------

// inter = current intersection (possibly NULL)
// nearest_inter = nearest intersection so far (also possibly NULL)

void update_nearest_intersection(Intersection **inter, Intersection **nearest_inter)
{
  // only do something if this was a hit

  if (*inter) {

    // this is the first object hit 
    
    if (!*nearest_inter) 
      *nearest_inter = *inter;
	
    // this is closer than any previous hit

    else if ((*inter)->t < (*nearest_inter)->t) {
      free(*nearest_inter);
      *nearest_inter = *inter;
    }

    // something else is closer--move along
      
    else 
      free(*inter);  
  }
}

//----------------------------------------------------------------------------

// intersect a 3-D ray with a 3D triangle

// from http://www.softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm

// Copyright 2001, softSurfer (www.softsurfer.com)
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.

#define SMALL_NUM  0.00000001 // anything that avoids division overflow

//    Input:  a ray R, and a triangle T
//    Return: intersection information (when it exists); NULL otherwise

Intersection *intersect_ray_triangle(Ray *ray, Vect V0, Vect V1, Vect V2)
{
  Vect    u, v, n;        // triangle vectors
  Vect    w0, w;          // ray vectors
  float   a, b;           // params to calc ray-plane intersect
  float t;
  Vect I;
  Intersection *inter;

  // get triangle edge vectors and plane normal

  VectSub(V1, V0, u);
  VectSub(V2, V0, v);
  VectCross(u, v, n);  
  if (n[X] == 0 && n[Y] == 0 && n[Z] == 0)            // triangle is degenerate; do not deal with this case
    return NULL;               

  VectSub(ray->orig, V0, w0);
  a = -VectDotProd(n,w0);
  b = VectDotProd(n,ray->dir);

  if (fabs(b) < SMALL_NUM) {     // ray is parallel to triangle plane
    if (a == 0)                  // case 1: ray lies in triangle plane
      return NULL;
    else return NULL;               // case 2: ray disjoint from plane
  }
  
  // get intersect point of ray with triangle plane

  t = a / b;
  if (t < rayeps)                   // triangle is behind/too close to ray => no intersect
    return NULL;                 // for a segment, also test if (t > 1.0) => no intersect

  // intersect point of ray and plane
   
  VectAddS(t, ray->dir, ray->orig, I);        

  // is I inside T?

  float    uu, uv, vv, wu, wv, D;
  uu = VectDotProd(u,u);
  uv = VectDotProd(u,v);
  vv = VectDotProd(v,v);
  VectSub(I, V0, w);
  wu = VectDotProd(w,u);
  wv = VectDotProd(w,v);
  D = uv * uv - uu * vv;
  
  // get and test parametric (i.e., barycentric) coords

  float p, q;  // were s, t in original code
  p = (uv * wv - vv * wu) / D;
  if (p < 0.0 || p > 1.0)        // I is outside T
    return NULL;
  q = (uv * wu - uu * wv) / D;
  if (q < 0.0 || (p + q) > 1.0)  // I is outside T
    return NULL;
  
  inter = make_intersection();
  inter->t = t;
  VectCopy(inter->P, I);
  return inter;                      // I is in T
}

//----------------------------------------------------------------------------

// multiply vertices of object by M

void glm_transform(Transform M, GLMmodel *model)
{
  Vect V, V_prime;

  // directly iterate over vertices -- indices seems to start at 1

  for (int i = 1; i <= model->numvertices; i++) {

    V[X] = model->vertices[3 * i];
    V[Y] = model->vertices[3 * i + 1];
    V[Z] = model->vertices[3 * i + 2];
    V[W] = 1.0;
    
    TransformVect(M, V, V_prime);
    VectHomogeneousToRegular(V_prime);

    model->vertices[3 * i] = V_prime[X];
    model->vertices[3 * i + 1] = V_prime[Y];
    model->vertices[3 * i + 2] = V_prime[Z];
    
  }
}

//----------------------------------------------------------------------------

// intersect ray with .obj model (a bunch of triangles with precomputed normals)
// if we hit something, set the color and return true

Intersection *intersect_ray_glm_object(Ray *ray, GLMmodel *model)
{
  static GLMgroup* group;
  static GLMtriangle* triangle;
  Vect V0, V1, V2;
  Intersection *nearest_inter = NULL;
  Intersection *inter = NULL;

  // iterate over all groups in the model

  for (group = model->groups; group; group = group->next) {

    // iterate over all triangles in this group

    for (int i = 0; i < group->numtriangles; i++) {

      triangle = &model->triangles[group->triangles[i]];

      // get triangle vertices

      V0[X] = model->vertices[3 * triangle->vindices[0]];
      V0[Y] = model->vertices[3 * triangle->vindices[0] + 1];
      V0[Z] = model->vertices[3 * triangle->vindices[0] + 2];

      V1[X] = model->vertices[3 * triangle->vindices[1]];
      V1[Y] = model->vertices[3 * triangle->vindices[1] + 1];
      V1[Z] = model->vertices[3 * triangle->vindices[1] + 2];

      V2[X] = model->vertices[3 * triangle->vindices[2]];
      V2[Y] = model->vertices[3 * triangle->vindices[2] + 1];
      V2[Z] = model->vertices[3 * triangle->vindices[2] + 2];

      // test for intersection

      inter = intersect_ray_triangle(ray, V0, V1, V2);
      
      // we have a hit in front of the camera...

      if (inter) {

	// set normal

	inter->N[X] = model->facetnorms[3 * triangle->findex];
	inter->N[Y] = model->facetnorms[3 * triangle->findex + 1];
	inter->N[Z] = model->facetnorms[3 * triangle->findex + 2];

	inter->surf = model_surf_list[model->index];

	// this the first hit 
	
	if (!nearest_inter) {
	  nearest_inter = inter;
	}
	
	// this is closer than any previous hit

	else if (inter->t < nearest_inter->t) {
	  free(nearest_inter);
	  nearest_inter = inter;
	}

	// something else is closer--move along

	else {
	  free(inter);
	}	  
      }
    }
  } 

  return nearest_inter;
}

//----------------------------------------------------------------------------

void reflection_direction(Vect incoming, Vect norm, Vect outgoing)
{
  VectAddS(-2 * VectDotProd(incoming, norm), norm, incoming, outgoing);
}

//----------------------------------------------------------------------------

// set color based on facet normal of intersection point

void shade_ray_false_color_normal(Intersection *inter, Vect color)
{
  color[R] = fabs(inter->N[X]);
  color[G] = fabs(inter->N[Y]);
  color[B] = fabs(inter->N[Z]);
}

//----------------------------------------------------------------------------

// started here http://www.unc.edu/~zimmons/cs238/maps/environment.html
// and modified this: http://www.unc.edu/~zimmons/cs238/maps/cubeind.html

void ray_to_cubemap(Camera *cam,
		    Vect raydir,  // float rx, float ry, float rz, 
		    int & side, float & s, float & t)
{
  float sc, tc, ma;
  Vect raydir_prime;
  float rx, ry, rz;

  // this is so the cube map "knows" about the camera pose and is transformed appropriately,
  // since raydir is in camera coordinates

  TransformVect(cam->W2C_invrot, raydir, raydir_prime);
  rx = raydir_prime[X];
  ry = raydir_prime[Y];
  rz = raydir_prime[Z];

  float arx = fabs(rx);
  float ary = fabs(ry);
  float arz = fabs(rz);

  if (arx >= ary && arx >= arz) {
    ma = arx;
    if (rx >= 0) {
      sc = -rz;
      tc = -ry;
      side = POSITIVE_X;
    }
    else {
      sc = +rz;
      tc = -ry;
      side = NEGATIVE_X;
    }
  }
  else if (ary >= arx && ary >= arz) {
    ma = ary;
    if (ry >= 0) {
      sc = +rx;
      tc = +rz;
      side = POSITIVE_Y;
    }
    else {
      sc = +rx;
      tc = -rz;
      side = NEGATIVE_Y;
    }
  }
  else {
    ma = arz;
    if (rz >= 0) {
      sc = +rx;
      tc = -ry;
      side = POSITIVE_Z;
    }
    else {
      sc = -rx;
      tc = -ry;
      side = NEGATIVE_Z;
    }
  }

  s = ((sc/ma) + 1) / 2;
  t = ((tc/ma) + 1) / 2;
}

//----------------------------------------------------------------------------

//replacment for rint (as it is not supported in VS2010
double rint(double a)
{
    const double two_to_52 = 4.5035996273704960e+15;
    double fa = fabs(a);
    double r = two_to_52 + fa;
    if (fa >= two_to_52) {
        r = a;
    } else {
        r = r - two_to_52;
        r = _copysign(r, a);
    }
    return r;
}

// turn cubemap side index and texture coords s and t into pixel coordinates in 
// overall image and read color there

void read_cubemap_color(int side, float s, float t, Vect color)
{
  int r, g, b;

  // for debugging, this just colors each side a solid

  /*
  if (side == POSITIVE_X) {
    r = 255;
    g = b = 0;
  }
  else if (side == NEGATIVE_X) {
    r = 64;
    g = b = 0;
  }
  else if (side == POSITIVE_Y) {
    g = 255;
    r = b = 0;
  }
  else if (side == NEGATIVE_Y) {
    g = 64;
    r = b = 0;
  }
  else if (side == POSITIVE_Z) {
    b = 255;
    r = g = 0;
  }
  else if (side == NEGATIVE_Z) {
    b = 64;
    r = g = 0;
  }
  */

  // this actually does the color lookup in the cubemap image...upright cross layout is assumed

  int x, y;

  if (side == POSITIVE_X) {
    x = rint(2.0 * fbackground_cubemap_sidelength + (1-s) * fbackground_cubemap_sidelength);
    y = rint(1.0 * fbackground_cubemap_sidelength + t * fbackground_cubemap_sidelength);
  }
  else if (side == NEGATIVE_X) {
    x = rint(0.0 * fbackground_cubemap_sidelength + (1-s) * fbackground_cubemap_sidelength);
    y = rint(1.0 * fbackground_cubemap_sidelength + t * fbackground_cubemap_sidelength);
  }
  else if (side == POSITIVE_Y) {
    x = rint(1.0 * fbackground_cubemap_sidelength + s * fbackground_cubemap_sidelength);
    y = rint(0.0 * fbackground_cubemap_sidelength + (1-t) * fbackground_cubemap_sidelength);
  }
  else if (side == NEGATIVE_Y) {
    x = rint(1.0 * fbackground_cubemap_sidelength + s * fbackground_cubemap_sidelength);
    y = rint(2.0 * fbackground_cubemap_sidelength + (1-t) * fbackground_cubemap_sidelength);
  }
  else if (side == POSITIVE_Z) {
    x = rint(1.0 * fbackground_cubemap_sidelength + s * fbackground_cubemap_sidelength);
    y = rint(3.0 * fbackground_cubemap_sidelength + (1-t) * fbackground_cubemap_sidelength);
  }
  else if (side = NEGATIVE_Z) {
    x = rint(1.0 * fbackground_cubemap_sidelength + (1-s) * fbackground_cubemap_sidelength);
    y = rint(1.0 * fbackground_cubemap_sidelength + t * fbackground_cubemap_sidelength);
  }

  int bpp = 3;

  r = ((unsigned char *) background_cubemap_im)[bpp * (y * background_cubemap_w + x)];
  g = ((unsigned char *) background_cubemap_im)[bpp * (y * background_cubemap_w + x) + 1];
  b = ((unsigned char *) background_cubemap_im)[bpp * (y * background_cubemap_w + x) + 2];

  // convert to [0, 1] range -- probably not the most efficient way to do this

  color[R] = (float) r / 255.0;
  color[G] = (float) g / 255.0;
  color[B] = (float) b / 255.0;
}

//----------------------------------------------------------------------------

// color to draw if ray hits nothing

// either solid or cube map lookup

void shade_ray_background(Ray *ray, Vect color)
{
  int side;
  float s, t;

  // cubemap: convert ray direction to texture coordinates and look it up

  if (background_cubemap_im) {

    ray_to_cubemap(ray_cam, ray->dir, side, s, t);
    read_cubemap_color(side, s, t, color);
  }

  // solid color 

  else {
    color[R] = background_r;
    color[G] = background_g;
    color[B] = background_b;
  }
}

//----------------------------------------------------------------------------

// opposite of background--just set a constant color for any
// ray that hits something

void shade_ray_intersection_mask(Vect color)
{
  color[R] = color[G] = color[B] = 1.0;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

Camera *make_camera()
{
  Camera *c;

  c = (Camera *) malloc(sizeof(Camera));

  return c;
}

//----------------------------------------------------------------------------

// allocate buffer for pixels -- set default color to red

Image *make_image(int w, int h)
{
  int i, j;
  Image *im;
  Vect red;

  im = (Image *) malloc(sizeof(Image));
  im->w = w;
  im->h = h;

  im->data = (GLubyte *) calloc(4 * w * h, sizeof(GLubyte));

  red[0] = 1.0;
  red[1] = red[2] = 0.0;
  red[3] = 1.0;

  for (j = 0; j < h; j++)
    for (i = 0; i < w; i++)
      draw_point(i, j, red, im);

  return im;
}

//----------------------------------------------------------------------------

Ray *make_ray()
{
  Ray *r;

  r = (Ray *) malloc(sizeof(Ray));

  return r;
}

//----------------------------------------------------------------------------

Ray *make_ray(Vect orig, Vect dir)
{
  Ray *r;

  r = make_ray();
  VectCopy(r->orig, orig);
  VectCopy(r->dir, dir);

  return r;
}

//----------------------------------------------------------------------------

Intersection *make_intersection()
{
  Intersection *inter;

  inter = (Intersection *) malloc(sizeof(Intersection));
  ///inter->p = NULL;
  inter->medium = NULL;

  return inter;
}

//----------------------------------------------------------------------------

void free_intersection(Intersection *inter)
{
  free(inter);
}

//----------------------------------------------------------------------------

// set up rigid transform of world coordinates to camera coordinates

void setup_lookat_transform(Transform M, Vect eye, Vect center, Vect up)
{
  Vect n, u, v, t;
  int i;

  // rotational components

  VectSub(eye, center, n);

  // u = up x n

  VectCross(up, n, u);

  // v = n x u

  VectCross(n, u, v);

  // normalize n, u, v

  VectUnit(n);
  VectUnit(u);
  VectUnit(v);

  // translation

  t[X] = -VectDotProd(eye, u);
  t[Y] = -VectDotProd(eye, v);
  t[Z] = -VectDotProd(eye, n);

  // put it together

  for (i = 0; i < 3; i++)
    MATRC(M, 0, i) = u[i];

  for (i = 0; i < 3; i++)
    MATRC(M, 1, i) = v[i];

  for (i = 0; i < 3; i++)
    MATRC(M, 2, i) = n[i];

  for (i = 0; i < 3; i++)
    MATRC(M, 3, i) = 0;

  for (i = 0; i < 3; i++)
    MATRC(M, i, 3) = t[i];

  MATRC(M, 3, 3) = 1;

//   printf("lookat M = \n");
//   TransformPrint(M);
}

//----------------------------------------------------------------------------

unsigned char *read_cubemap_PPM(char *filename, int *width, int *height)
{
  int i, j, w, h;
  FILE *fp;
  int r, g, b, t;
  bool iscomment;
  unsigned char ur, ug, ub;
  char imtype[3];
  char str[80];
  char c;
  unsigned char *im;
  int bpp;

  // get size info

  fp = fopen(filename, "rb");
  if (!fp) {
    printf("read_cubemap_PPM(): no such file");
    exit(1);
  }

  fscanf(fp, "%s\n", &imtype);

  // attempt to eat comments

  do {
    iscomment = false;
    c = fgetc(fp);
    ungetc(c, fp);
    if (c == '#') {
      iscomment = true;
      fgets (str, 79, fp);
    }
  } while (iscomment);

  // read image dimensions

  fscanf(fp, "%i %i\n255\n", &w, &h);    
  *width = w;
  *height = h;

  // allocate image

  bpp = 3;

  im = (unsigned char *) calloc(w * h * bpp, sizeof(unsigned char));

  // actually read it in

  // ascii

  if (!strcmp(imtype, "P3")) {

    for (j = 0, t = 0; j < h; j++) {
      for (i = 0; i < w; i++, t += bpp) {
	fscanf(fp, "%i %i %i ", &r, &g, &b);
	im[t] = (unsigned char) r;
	im[t+1] = (unsigned char) g;
	im[t+2] = (unsigned char) b;
      }
    }
    fscanf(fp, "\n");
  }

  // binary

  else if (!strcmp(imtype, "P6")) {
    
    for (j = 0, t = 0; j < h; j++) {
      for (i = 0; i < w; i++, t += bpp) {
	fscanf(fp, "%c%c%c", &ur, &ug, &ub);
	im[t] = (unsigned char) ur;
	im[t+1] = (unsigned char) ug;
	im[t+2] = (unsigned char) ub;
      }
    }
  }

  // unknown

  else {
    printf("unrecognized ppm file type");
    exit(1);
  }

  // finish up

  fclose(fp);

  return im;
}

//----------------------------------------------------------------------------

void parse_scene_file(char *filename, Camera *cam)
{
  char c, background_type_string[64], background_cubemap_filename[64], obj_filename[64];
  int w, h;
  FILE *fp;
  float sx, sy, sz, rx, ry, rz, dx, dy, dz;   // scale factors, rotation angles (degrees), translation factors
  Transform Tmat, RXmat, RYmat, RZmat, RXYmat, Rmat, Smat, RSmat, TRSmat, Mmat;
  Vect P;
  Light *L;
  Surface *S;
  Sphere *Sph;
  int row, col;

  // read file

  fp = fopen(filename, "r");

  // camera position

  fscanf(fp, "camera %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
	 &cam->eye[X], &cam->eye[Y], &cam->eye[Z], 
	 &cam->center[X], &cam->center[Y], &cam->center[Z], 
	 &cam->up[X], &cam->up[Y], &cam->up[Z]);

  setup_lookat_transform(cam->W2C, cam->eye, cam->center, cam->up);

  // inverse of rotation-only version, for transforming rays to cube map

  for (row = 0; row < 4; row++)
    for (col = 0; col < 4; col++)
      MATRC(cam->W2C_invrot, row, col) = MATRC(cam->W2C, col, row);
  for (col = 0; col < 3; col++)
    MATRC(cam->W2C_invrot, 3, col) = 0.0;
 
  // clip planes

  fscanf(fp, "clip %lf %lf %lf %lf %lf %lf\n", 
	 &cam->clip[LEFT], &cam->clip[RIGHT], 
	 &cam->clip[BOTTOM], &cam->clip[TOP],  
	 &cam->clip[NEAR], &cam->clip[FAR]); 

  // image dimensions
  
  fscanf(fp, "image %i %i\n", &w, &h);
  cam->im = make_image(w, h);

  // objects, lights, background

  model_list.clear();
  model_surf_list.clear();
  sphere_list.clear();
  light_list.clear();

  while (1) {
    
    c = fgetc(fp);

    // end of file

    if (c == EOF)
      return;

    // it's a comment

    if (c == '#') {
      do { c = fgetc(fp); /* printf("eating %c\n", c); */ } 
      while (c != '\n' && c != EOF);
      if (c == EOF)
	return;
//      printf("done\n");
    }

    // it's the background

    else if (c == 'b') {

      fscanf(fp, "ackground %s ", background_type_string);
      if (!strcmp(background_type_string, "solid"))
	fscanf(fp, "%f %f %f\n", &background_r, &background_g, &background_b);
      else if (!strcmp(background_type_string, "cubemap")) {
	fscanf(fp, "%s\n", background_cubemap_filename);
	background_cubemap_im = read_cubemap_PPM(background_cubemap_filename, &background_cubemap_w, &background_cubemap_h);
	fbackground_cubemap_sidelength = background_cubemap_sidelength = background_cubemap_w / 3;   // here we assume upright cross arrangement
	printf("cubemap %i x %i (%i %f)\n", background_cubemap_w, background_cubemap_h, background_cubemap_sidelength, fbackground_cubemap_sidelength);
      }
      else {
	printf("unknown scene background type\n");
	exit(1);
      }

    }

    // it's an object
    
    else if (c == 'o') {

      S = (Surface *) malloc(sizeof(Surface));

      fscanf(fp, "bj %s  %f %f %f  %f %f %f  %f %f %f  %lf %lf %lf  %lf %lf %lf  %lf %lf %lf  %lf %lf  %lf %lf\n", 
	     obj_filename, 
	     &dx, &dy, &dz, 
	     &sx, &sy, &sz, 
	     &rx, &ry, &rz,
	     &(S->amb[R]), &(S->amb[G]), &(S->amb[B]),
	     &(S->diff[R]), &(S->diff[G]), &(S->diff[B]),
	     &S->spec[R], &S->spec[G], &S->spec[B],
	     &S->spec_exp, &S->ior, 
	     &S->reflectivity, &S->transparency);

      GLMmodel *model = glmReadOBJ(obj_filename);

      if (!model) {
	printf("no such glm model %s\n", obj_filename);
	exit(1);
      }
      
      // scale and center model on origin
      
      glmUnitize(model);
      
      // build model transformations (including lookat world -> camera transform)

      // translation

      TransformIdentity(Tmat);
      MATRC(Tmat, 0, 3) = dx;
      MATRC(Tmat, 1, 3) = dy;
      MATRC(Tmat, 2, 3) = dz;

      // scale

      TransformIdentity(Smat);
      MATRC(Smat, 0, 0) = sx;
      MATRC(Smat, 1, 1) = sy;
      MATRC(Smat, 2, 2) = sz;

      // rotation around x axis

      TransformIdentity(RXmat);
      MATRC(RXmat, 0, 0) = 1;
      MATRC(RXmat, 1, 0) = 0;
      MATRC(RXmat, 2, 0) = 0;
      MATRC(RXmat, 0, 1) = 0;
      MATRC(RXmat, 1, 1) = cos(DEG2RAD(rx));
      MATRC(RXmat, 2, 1) = -sin(DEG2RAD(rx));
      MATRC(RXmat, 0, 2) = 0;
      MATRC(RXmat, 1, 2) = sin(DEG2RAD(rx));
      MATRC(RXmat, 2, 2) = cos(DEG2RAD(rx));

      // rotation around y axis

      TransformIdentity(RYmat);
      MATRC(RYmat, 0, 0) = cos(DEG2RAD(ry));
      MATRC(RYmat, 1, 0) = 0;
      MATRC(RYmat, 2, 0) = sin(DEG2RAD(ry));
      MATRC(RYmat, 0, 1) = 0;
      MATRC(RYmat, 1, 1) = 1;
      MATRC(RYmat, 2, 1) = 0;
      MATRC(RYmat, 0, 2) = -sin(DEG2RAD(ry));
      MATRC(RYmat, 1, 2) = 0;
      MATRC(RYmat, 2, 2) = cos(DEG2RAD(ry));

      // rotation around z axis

      TransformIdentity(RZmat);
      MATRC(RZmat, 0, 0) = cos(DEG2RAD(rz));
      MATRC(RZmat, 1, 0) = -sin(DEG2RAD(rz));
      MATRC(RZmat, 2, 0) = 0;
      MATRC(RZmat, 0, 1) = sin(DEG2RAD(rz));
      MATRC(RZmat, 1, 1) = cos(DEG2RAD(rz));
      MATRC(RZmat, 2, 1) = 0;
      MATRC(RZmat, 0, 2) = 0;
      MATRC(RZmat, 1, 2) = 0;
      MATRC(RZmat, 2, 2) = 1;

      TransformProd(RXmat, RYmat, RXYmat); 
      TransformProd(RXYmat, RZmat, Rmat); 

      // create combined transformation

      TransformProd(Rmat, Smat, RSmat); 
      TransformProd(Tmat, RSmat, TRSmat); 
      TransformProd(cam->W2C, TRSmat, Mmat); 
      //TransformPrint(Mmat);

      // apply transform to model
      
      glm_transform(Mmat, model);
      
      // calculate normals
      
      glmFacetNormals(model);
      glmVertexNormals(model, 90.0);
      
      model->index = model_list.size();
      model_list.push_back(model);

      model_surf_list.push_back(S);

    }
    
    // it's a sphere

    else if (c == 's') {

      S = (Surface *) malloc(sizeof(Surface));
      Sph = (Sphere *) malloc(sizeof(Sphere));

      fscanf(fp, "phere %lf %lf %lf  %f  %lf %lf %lf  %lf %lf %lf  %lf %lf %lf  %lf %lf  %lf %lf\n", 
	     &P[X], &P[Y], &P[Z],    // world position
	     &sx,              // size
	     &(S->amb[R]), &(S->amb[G]), &(S->amb[B]),
	     &(S->diff[R]), &(S->diff[G]), &(S->diff[B]),
	     &S->spec[R], &S->spec[G], &S->spec[B],
	     &S->spec_exp, &S->ior, 
	     &S->reflectivity, &S->transparency);

      Sph->radius = sx;

      Sph->surf = S;

      // move to camera coordinates

      P[W] = 1.0;
      TransformVect(cam->W2C, P, Sph->P);
      VectHomogeneousToRegular(Sph->P);

      // addd to list

      sphere_list.push_back(Sph);
    }

    // it's a light

    else if (c == 'l') {

      L = (Light *) malloc(sizeof(Light));

      fscanf(fp, "ight %lf %lf %lf  %lf %lf %lf  %lf %lf %lf  %lf %lf %lf\n", 
	     &P[X], &P[Y], &P[Z],
	     &L->amb[R], &L->amb[G], &L->amb[B],
	     &L->diff[R], &L->diff[G], &L->diff[B],
	     &L->spec[R], &L->spec[G], &L->spec[B]);

      // move to camera coordinates

      P[W] = 1.0;
      TransformVect(cam->W2C, P, L->P);
      VectHomogeneousToRegular(L->P);

      // add to list

      light_list.push_back(L);
    }
    
    // unknown

    else if (c == '\n' && c != ' ' && c != '\t') {
      printf("bad scene syntax %c\n", c);
      exit(1);
    }
  }
}

//----------------------------------------------------------------------------

// lower-left corner of image is (0, 0)
// alpha is not used here--you need to change this for blending to work

void draw_point(int x, int y, Vect color, Image *im)
{
  im->data[4 * (x + im->w * y)] = (GLubyte) (255.0*color[R]);
  im->data[1 + 4 * (x + im->w * y)] = (GLubyte) (255.0*color[G]);
  im->data[2 + 4 * (x + im->w * y)] = (GLubyte) (255.0*color[B]);
  im->data[3 + 4 * (x + im->w * y)] = (GLubyte) (255.0*color[A]);
}

//----------------------------------------------------------------------------

int my_round(double x)
{
  return (int) floor(0.5 + x);
}

//----------------------------------------------------------------------------

// write binary PPM (expects data to be 4 ubytes per pixel)

void write_PPM(char *filename, Image *im)
{
  int i, j, t;
  FILE *fp;

  // write size info

  fp = fopen(filename, "wb");

  fprintf(fp, "P6\n%i %i\n255\n", im->w, im->h);

  // write binary image data

  for (j = 0, t = 0; j < im->h; j++) 
    for (i = 0; i < im->w; i++, t += 4) 
      fprintf(fp, "%c%c%c", 
	      (int) im->data[t], 
	      (int) im->data[t+1], 
	      (int) im->data[t+2]);

  // finish up

  fclose(fp);
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------



