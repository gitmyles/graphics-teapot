//----------------------------------------------------------------------------
// UD Ray V6

// copyright 2016, University of Delaware
// Christopher Rasmussen w/ contributions from Myles Johnson-Gray
//----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <string>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <glfw3.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <common/shader.hpp>
#include <common/texture.hpp>

using namespace glm;
using namespace std;

#include "udray.hh"

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

GLFWwindow* window;

GLuint programID;
GLuint MatrixID;
GLuint VertexArrayID;

int billboard_num_vertices = 6;  // 2 triangles to make a quad
GLfloat billboard_depth = -0.1;

GLfloat *billboard_vertex_buffer_data;
GLfloat *billboard_uv_buffer_data;
GLuint billboard_vertexbuffer;
GLuint billboard_uvbuffer;

GLuint teximage;
GLuint texID;

glm::mat4 Projection;
glm::mat4 View;
glm::mat4 Model;

//----------------------------------------------------------------------------

extern Camera *ray_cam;       // camera info
extern int image_i, image_j;  // current pixel being shaded
extern bool wrote_image;      // has the last pixel been shaded?

// reflection/refraction recursion control

extern int maxlevel;          // maximum depth of ray recursion 
extern double minweight;      // minimum fractional contribution to color

// these describe the scene

extern vector < GLMmodel * > model_list;
extern vector < Sphere * > sphere_list;
extern vector < Light * > light_list;

// camera location in world space
float x_cam = 0.0;
float y_cam = 0.0;
float z_cam = 5.0;

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// intersect a ray with the entire scene (.obj models + spheres)

// x, y are in pixel coordinates with (0, 0) the upper-left hand corner of the image.
// color variable is result of this function--it carries back info on how to draw the pixel

void trace_ray(int level, double weight, Ray *ray, Vect color)
{
  Intersection *nearest_inter = NULL;
  Intersection *inter = NULL;
  Intersection *tempInter;
  int i;

  // test for intersection with all .obj models
 
  for (i = 0; i < model_list.size(); i++) {
    inter = intersect_ray_glm_object(ray, model_list[i]);
	tempInter = nearest_inter;
	update_nearest_intersection(&inter, &nearest_inter);
  }

  // test for intersection with all spheres

  for (i = 0; i < sphere_list.size(); i++) {
    inter = intersect_ray_sphere(ray, sphere_list[i]);
    update_nearest_intersection(&inter, &nearest_inter);
  }

  // "color" the ray according to intersecting surface properties

  // choose one of the simpler options below to debug or preview your scene more quickly.
  // another way to render faster is to decrease the image size.

  if (nearest_inter) {
    //shade_ray_false_color_normal(nearest_inter, color);
    //shade_ray_intersection_mask(color);  
    shade_ray_diffuse(ray, nearest_inter, color);
	//shade_ray_local(ray, nearest_inter, color);
    //shade_ray_recursive(level, weight, ray, nearest_inter, color);
  }

  // color the ray using a default

  else
    shade_ray_background(ray, color); 
}

//----------------------------------------------------------------------------

// test for ray-sphere intersection; return details of intersection if true

Intersection *intersect_ray_sphere(Ray *ray, Sphere *S)
{
	Vect    p;                // sphere center
	Vect p0,p1;     //ray-sphere intersection points
	VectCopy(p,S->P);       
	double  r=S->radius;   //sphere radius
	Vect    O, Dir,tDir;    // ray vectors
	VectCopy(O, ray->orig);
	VectCopy(Dir, ray->dir);
	float   t0, t1;   // params to calc ray-sphere intersect
	double tca, thc, d2;// measurements required to calculate t0 and t1
	Vect L;        //Vector from center of sphere to origin of ray
	Vect N1,N2;   //Normals from the intersection points of ray-sphere

	float t;
	Intersection *inter1,*inter2;

	// geometric solution
	VectSub(p,O,L);
	tca = VectDotProd(L,Dir);
	if (tca < 0) return NULL; //No intersection
	d2 = (VectDotProd(L,L)) - tca * tca;
	if (d2 > r*r) return NULL;
	thc = sqrt(r*r - d2);
	t0 = tca - thc;
	t1 = tca + thc;
	VectAddS(t0, Dir, O, p0);
	VectAddS(t1, Dir, O, p1);
	N1[0] = (p0[0] - O[0])/abs(p0[0]-O[0]) ;
	N1[1] = (p0[1] - O[1]) / abs(p0[1] - O[1]);
	N1[2] = (p0[2] - O[2]) / abs(p0[2] - O[2]);
	N2[0] = (p1[0] - O[0]) / abs(p0[0] - O[0]);
	N2[1] = (p1[1] - O[1]) / abs(p0[1] - O[1]);
	N2[2] = (p1[2] - O[2]) / abs(p0[2] - O[2]);
	



    if (t0 < 0) {
		t0 = t1; // if t0 is negative, let's use t1 instead 
		if (t0 < 0) return NULL; // both t0 and t1 are negative 
	}


	//one intersection
    if (t0 == t1)
	{
		
		inter1 = make_intersection();
		inter1->t = t0;
		VectCopy(inter1->P, p0);
		VectCopy(inter1->N,N1);
		inter1->surf=S->surf;
		return inter1;
	}
	//two intersections
	else 
	{
		VectAddS(t0, Dir, O, p0);
		inter1 = make_intersection();
		inter1->t = t0;
		VectCopy(inter1->P, p0);
		VectCopy(inter1->N, N1);
		inter1->surf = S->surf;
		inter2 = make_intersection();
		inter2->t = t1;
		VectCopy(inter2->P, p0);
		VectCopy(inter2->N, N2);
		inter2->surf = S->surf;
		return inter1,inter2;
	}

 
}
//----------------------------------------------------------------------------

float clamp(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

//----------------------------------------------------------------------------

// only local, ambient + diffuse lighting (no specular, shadows, reflections, or refractions)
void shade_ray_diffuse(Ray *ray, Intersection *inter, Vect color)
{
  //Vect L; handles intersection normals for diffuse lighting
  glm::vec3 iNorm = vec3(0,0,0); //handles intersections normals for diffuse lighting
  glm::vec3 lNorm = vec3(0,0,0); // handles light positions and vectors
  float lightPower = 45.0f; //light power (feel free to experiment!)
  //color[R] = 0; 
  //color[G] = 0;
  //color[B] = 0;
  
  double diff_factor;

  // iterate over lights

  for (int i = 0; i < light_list.size(); i++) {

    // AMBIENT
    color[R] += inter->surf->amb[R] * light_list[i]->amb[R];
    color[G] += inter->surf->amb[G] * light_list[i]->amb[G];
    color[B] += inter->surf->amb[B] * light_list[i]->amb[B];
    
    // DIFFUSE 
	iNorm = vec3(inter->N[R], inter->N[G], inter->N[B]); //normals already parameterized
	lNorm = vec3(light_list[i]->P[R], light_list[i]->P[G], light_list[i]->P[B]); //intialized to light position
	
	//calc distance between light and vertex in world space
    float distance = length( lNorm - iNorm );

	//calculate light vector and normalize
	lNorm = vec3(light_list[i]->P[R] - inter->N[R], light_list[i]->P[G] - inter->N[G], light_list[i]->P[B]- inter->N[B]);
	lNorm = normalize(-lNorm);

	//calculate cosTheta
    float cosTheta = clamp(dot(iNorm,lNorm), 0.0, 1.0);

	// handles diffuse lighting...intialized to surface diff reflectance
	vec3 diffLight = vec3(inter->surf->diff[R], inter->surf->diff[G], inter->surf->diff[B]);

	//calculate total diffuse lighting
	diffLight[R] = diffLight[R] * light_list[i]->diff[R] * lightPower * cosTheta / pow(distance,2);
	diffLight[G] = diffLight[G] * light_list[i]->diff[G] * lightPower * cosTheta / pow(distance,2);
	diffLight[B] = diffLight[B] * light_list[i]->diff[B] * lightPower * cosTheta / pow(distance,2);

	//add diffuse lighting to existing ambient lighting
	color[R] += diffLight[R];
	color[G] += diffLight[G];
	color[B] += diffLight[B];
  }

  // clamp color to [0, 1]
  VectClamp(color, 0, 1);
}

//----------------------------------------------------------------------------

// same as shade_ray_diffuse(), but add specular lighting + shadow rays (i.e., full Phong illumination model)

void shade_ray_local(Ray *ray, Intersection *inter, Vect color)
{
  //mostly copied from shade_ray_diffuse:

  //Vect L; handles intersection normals for diffuse lighting
  glm::vec3 iNorm = vec3(0,0,0); //handles intersections normals for diffuse lighting
  glm::vec3 lNorm = vec3(0,0,0); // handles light positions and vectors
  //glm::vec3 eNorm = vec3(ray_cam->eye[R],ray_cam->eye[G],ray_cam->eye[B]); // to handle eye direction vector...initialized to cam position
  glm::vec3 eNorm = vec3(0,0,0); // to handle eye direction vector...initialized to cam position
  float lightPower = 60.0f; //light power (feel free to experiment!)
  color[R] = 0; 
  color[G] = 0;
  color[B] = 0;
  bool isShadowed = 0; //boolean that determines if the intersection is in a shadowed area (0:false, 1:true)
  
  double diff_factor;

  // iterate over lights

  for (int i = 0; i < light_list.size(); i++) {

    //intitialize vectors to intersection and light positions (for shadow rays)
    Vect interOrigin;
    interOrigin[R] = inter->P[R];
    interOrigin[G] = inter->P[G];
    interOrigin[B] = inter->P[B];
    Vect lightOrigin;
    lightOrigin[R] = light_list[i]->P[R];
    lightOrigin[G] = light_list[i]->P[G];
    lightOrigin[B] = light_list[i]->P[B];
	Vect sRayDir; // to hold the direction of the shadow ray
	
	Vect tempVec; // auxillary vectors
	Vect tempVec2;
	Vect tempVec3; //stores shadow ray intersection position

	VectSub(interOrigin, lightOrigin, sRayDir); //difference between intersection and light
	VectUnit(sRayDir);

	//the ray between the intersection and light
	Ray *shadowRay = make_ray(interOrigin, sRayDir);

	//SHADOWS
	// test to see if there is an object between the intersection and light    
	//intersection between the shadow ray and a model (obj or sphere)
	Intersection *shadowInter = NULL;

	// test for intersection with all .obj models
    for (int j = 0; j < model_list.size(); j++) {
		//if (j != currentModel) //except the obj it belongs to
		//{
			if (intersect_ray_glm_object(shadowRay, model_list[j]) != NULL) // intersection is between orgin and light?
			{
				//isShadowed = 1; //prepare for shadows?
				shadowInter = intersect_ray_glm_object(shadowRay, model_list[j]);
				tempVec3[R] = shadowInter->N[R];
				tempVec3[G] = shadowInter->N[G];
				tempVec3[B] = shadowInter->N[B];

				VectSub(lightOrigin,  interOrigin, tempVec);
				VectSub(tempVec3,  interOrigin, tempVec2);

				Vect tempVec4;
				VectCross(tempVec, tempVec2, tempVec4);

				//if(VectDotProd(tempVec, tempVec2) > 0)
				if(tempVec4 == 0 && VectDotProd(tempVec, tempVec2) > 0)
				{
					isShadowed = 1; //prepare for shadows?
				}
			}
		//}
    }

	if(isShadowed == 1){} 
	else{ // calculate light for the point
		
		// AMBIENT
		color[R] += inter->surf->amb[R] * light_list[i]->amb[R];
		color[G] += inter->surf->amb[G] * light_list[i]->amb[G];
		color[B] += inter->surf->amb[B] * light_list[i]->amb[B];
    
		// DIFFUSE 
		iNorm = vec3(inter->N[R], inter->N[G], inter->N[B]); //normals already parameterized
		lNorm = vec3(light_list[i]->P[R], light_list[i]->P[G], light_list[i]->P[B]); //intialized to light position
	
		//calc distance between light and vertex in world space
		float distance = length( lNorm - iNorm );

		//normalize intersection normals
		iNorm = normalize(iNorm);

		//calculate light vector and normalize
		lNorm = vec3(light_list[i]->P[R] - inter->N[R], light_list[i]->P[G] - inter->N[G], light_list[i]->P[B]- inter->N[B]);
		lNorm = normalize(lNorm);

		//calculate vector from camera to intersection hitpoint
		eNorm[R] = eNorm[R] - inter->P[R];
		eNorm[G] = eNorm[G] - inter->P[G];
		eNorm[B] = eNorm[B] - inter->P[B];

		//calculate cosAlpha
		eNorm = normalize(eNorm);
		glm::vec3 rNorm = reflect(-lNorm,iNorm);
		float cosAlpha = glm::max(0.0f, dot( eNorm,rNorm ));

		//calculate cosTheta
		float cosTheta = clamp(dot(iNorm,lNorm), 0.0, 1.0);

		//handles diffuse lighting and specular lighting
		vec3 diffLight = vec3(inter->surf->diff[R], inter->surf->diff[G], inter->surf->diff[B]); //intialized to surface diff reflectance
		vec3 specLight = vec3(inter->surf->spec[R], inter->surf->spec[G], inter->surf->spec[B]); //intialized to surface spec reflectance

		//calculate total diffuse lighting
		diffLight[R] = diffLight[R] * light_list[i]->diff[R] * lightPower * cosTheta / pow(distance,2);
		diffLight[G] = diffLight[G] * light_list[i]->diff[G] * lightPower * cosTheta / pow(distance,2);
		diffLight[B] = diffLight[B] * light_list[i]->diff[B] * lightPower * cosTheta / pow(distance,2);

		specLight[R] = specLight[R] * light_list[i]->spec[R] * lightPower * pow(cosAlpha, inter->surf->spec_exp) / (distance*distance);
		specLight[G] = specLight[G] * light_list[i]->spec[G] * lightPower * pow(cosAlpha, inter->surf->spec_exp) / (distance*distance);
		specLight[B] = specLight[B] * light_list[i]->spec[B] * lightPower * pow(cosAlpha, inter->surf->spec_exp) / (distance*distance);

		//add diffuse and specular lighting to existing ambient lighting
		color[R] += diffLight[R] + specLight[R];
		color[G] += diffLight[G] + specLight[G];
		color[B] += diffLight[B] + specLight[B];
	}
  }

  // clamp color to [0, 1]
  VectClamp(color, 0, 1);
}

//----------------------------------------------------------------------------

// full shading model: ambient/diffuse/specular lighting, shadow rays, recursion for reflection, refraction

// level = recursion level (only used for reflection/refraction)

void shade_ray_recursive(int level, double weight, Ray *ray, Intersection *inter, Vect color)
{
  Surface *surf = NULL;
  int i=0;
  glm::vec3 n = vec3(0, 0, 0);
  glm::vec3 v = vec3(0, 0, 0);
  glm::vec3 e = vec3(ray_cam->eye[0], ray_cam->eye[1], ray_cam->eye[2]);
  double exp = inter->surf->spec_exp;
  Intersection *reflectInter = NULL;

  //calculate vector from camera to intersection hitpoint
  e[0] = e[0] - inter->P[0];
  e[1] = e[1] - inter->P[1];
  e[2] = e[2] - inter->P[2];
  e = normalize(e);

  Vect t1,r1;

  // initialize color to Phong reflectance model

  shade_ray_local(ray, inter, color);

  // if not too deep, recurse

  if (level + 1 < maxlevel) {

    // add reflection component to color

    if (inter->surf->reflectivity * weight > minweight) 
	{	
		glm::vec3 n = vec3(inter->N[0], inter->N[1], inter->N[2]);
		//glm::vec3 v = vec3(ray->orig[0], ray->orig[1], ray->orig[2]);
		glm::vec3 v = vec3(x_cam - inter->P[R], y_cam - inter->P[G], z_cam - inter->P[B]);
		//glm::vec3 v = vec3(ray->dir[0], ray->dir[1], ray->dir[2]);

		
		//calculate cosAlpha
		float distance = length(n-v);
		n = normalize(n);
		v = normalize(v);
		
		//glm::vec3 r = v-(glm::vec3(2*(dot(n,v)))*n);
		glm::vec3 r = v-(glm::vec3(2*(dot(n,v)))*n);
		float cosAlpha = clamp(dot(e, r), 0.0f, 1.0f);
		
		r1[0] = r[0]; r1[1] = r[1]; r1[2] = r[2];
		Ray ray_reflected = *make_ray(inter->P, r1);
		Ray *rayr = &ray_reflected;

		for (int i = 0; i < light_list.size(); i++) {
			for (int j = 0; j < model_list.size(); j++) {
				if (intersect_ray_glm_object(rayr, model_list[j]) != NULL) // intersection is between orgin and light?
				{
					reflectInter = intersect_ray_glm_object(rayr, model_list[j]);
					color[0] += (reflectInter->surf->diff[R] * light_list[i]->diff[R]) + (reflectInter->surf->spec[R]* light_list[i]->diff[R]) * pow(cosAlpha, exp) / (distance * distance);
					color[1] += (reflectInter->surf->diff[G] * light_list[i]->diff[G]) + (reflectInter->surf->spec[G]* light_list[i]->diff[G]) * pow(cosAlpha, exp) / (distance * distance);
					color[2] += (reflectInter->surf->diff[B] * light_list[i]->diff[B]) + (reflectInter->surf->spec[B]* light_list[i]->diff[B]) * pow(cosAlpha, exp) / (distance * distance);
				}
			}


		//reflected color added to previously computed color
		//color[R] += inter->surf->reflectivity*pow(cosAlpha, exp) / (distance * distance);
		//color[G] += inter->surf->reflectivity*pow(cosAlpha, exp) / (distance * distance);
		//color[B] += inter->surf->reflectivity*pow(cosAlpha, exp) / (distance * distance);
		}
		level += 1;
		shade_ray_recursive(level, weight, rayr, inter, color);

    // add refraction component to color

    if (inter->surf->transparency * weight > minweight) 
	{
		glm::vec3 n = vec3(inter->N[0], inter->N[1], inter->N[2]);
		glm::vec3 v = vec3(ray->dir[0], ray->dir[1], ray->dir[2]);
		glm::vec3 t = vec3(0, 0, 0);
		
		float distance = length(n - v);
		n = normalize(n);
		v = normalize(v);
		
		if (inter->entering == true)
		{

			if (inter->surf->ior > 1)
			{
				
				double n1 = 1.0;
				double n2 = inter->surf->ior;
				double nr = n1 / n2;

				double cosTheta1 = dot(-v, n);
				double cosTheta2 = sqrt(1 - (nr*nr)*(1 - cosTheta1*cosTheta1));
			 t=(glm::vec3(nr)*v) + (glm::vec3(nr*cosTheta1 - cosTheta2)*n);
			 t1[0] = t[0]; t1[1] = t[1]; t1[2] = t[2];
			Ray ray_refracted = *make_ray(inter->P,t1);
			Ray *rayt = &ray_refracted;
			float cosTheta = clamp(dot(e, t), 0.0f, 1.0f);
			
				color[R] += inter->surf->transparency*(cosTheta/ (distance * distance));
				color[G] += inter->surf->transparency*(cosTheta / (distance * distance));
				color[B] += inter->surf->transparency*(cosTheta / (distance * distance));
				
				level += 1;
				shade_ray_recursive(level, weight,rayt, inter, color);

			}
			else
			{
				double n1 = inter->surf->ior;
				double n2 = 1.0;
				double nr = n1 / n2;

				double cosTheta1 = dot(-v, n);
				double cosTheta2 = sqrt(1 - (nr*nr)*(1 - cosTheta1*cosTheta1));
				t = (glm::vec3(nr)*v) + (glm::vec3(nr*cosTheta1 - cosTheta2)*n);
				t1[0] = t[0]; t1[1] = t[1]; t1[2] = t[2];
				Ray ray_refracted = *make_ray(inter->P, t1);
				Ray *rayt = &ray_refracted;
				float cosTheta = clamp(dot(e, t), 0.0f, 1.0f);

				color[R] += inter->surf->transparency*(cosTheta / (distance * distance));
				color[G] += inter->surf->transparency*(cosTheta / (distance * distance));
				color[B] += inter->surf->transparency*(cosTheta / (distance * distance));

				level += 1;
				shade_ray_recursive(level, weight, rayt, inter, color);
			}
		}

    }
	
  }
}
}
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// describe the quad that we will affix ray trace image texture to

void generate_billboard() 
{
  glGenVertexArrays(1, &VertexArrayID);
  glBindVertexArray(VertexArrayID);

  billboard_vertex_buffer_data = (GLfloat *) malloc(3 * billboard_num_vertices * sizeof(GLfloat));
  billboard_uv_buffer_data = (GLfloat *) malloc(2 * billboard_num_vertices * sizeof(GLfloat));
  
  // unit square in the XY plane, centered on origin
  // broken into two right triangles

  billboard_vertex_buffer_data[0]  = -0.5;
  billboard_vertex_buffer_data[1]  = -0.5;

  billboard_vertex_buffer_data[3]  =  0.5;
  billboard_vertex_buffer_data[4]  =  0.5;

  billboard_vertex_buffer_data[6]  = -0.5;
  billboard_vertex_buffer_data[7]  =  0.5;


  billboard_vertex_buffer_data[9]  = -0.5;
  billboard_vertex_buffer_data[10] = -0.5;

  billboard_vertex_buffer_data[12] =  0.5;
  billboard_vertex_buffer_data[13] = -0.5;

  billboard_vertex_buffer_data[15] =  0.5;
  billboard_vertex_buffer_data[16] =  0.5;

  // finish with Z all the same 

  for (int i = 0; i < billboard_num_vertices; i++)
    billboard_vertex_buffer_data[3 * i + 2] = billboard_depth;

  // tex coords

  billboard_uv_buffer_data[0]  = 0.0;
  billboard_uv_buffer_data[1]  = 1.0;

  billboard_uv_buffer_data[2]  = 1.0;
  billboard_uv_buffer_data[3]  = 0.0;

  billboard_uv_buffer_data[4]  = 0.0;
  billboard_uv_buffer_data[5]  = 0.0;


  billboard_uv_buffer_data[6]  = 0.0;
  billboard_uv_buffer_data[7]  = 1.0;

  billboard_uv_buffer_data[8]  = 1.0;
  billboard_uv_buffer_data[9]  = 1.0;

  billboard_uv_buffer_data[10] = 1.0;
  billboard_uv_buffer_data[11] = 0.0;

  // bind

  glGenBuffers(1, &billboard_vertexbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, billboard_vertexbuffer);
  glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(GLfloat) * billboard_num_vertices, billboard_vertex_buffer_data, GL_STATIC_DRAW);

  glGenBuffers(1, &billboard_uvbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, billboard_uvbuffer);
  glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(GLfloat) * billboard_num_vertices, billboard_uv_buffer_data, GL_STATIC_DRAW);
}

//----------------------------------------------------------------------------

void draw_billboard()
{    
  // 1st attribute buffer : vertices

  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, billboard_vertexbuffer);
  glVertexAttribPointer(
			0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
			);
  
  // 2nd attribute buffer : texture coordinates

  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, billboard_uvbuffer);
  glVertexAttribPointer(
			1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
			2,                                // size : U+V => 2
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
			);

  // Draw!

  glDrawArrays(GL_TRIANGLES, 0, billboard_num_vertices); 

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);

}

//----------------------------------------------------------------------------

// orthographic camera at (0, 0, 1), looking at origin, up vector (0, 1, 0), with view volume scaled to fit window
// model transform is identity

void setup_ortho2d_camera(int win_w, int win_h)
{
  // parametrize M, V, and P

  Model      = glm::mat4(1.0f);

  Projection = glm::ortho(-0.5f, 0.5f, -0.5f, 0.5f, 0.1f, 10.0f);

  View = glm::lookAt(glm::vec3(x_cam,y_cam,z_cam),   // camera location in world space
		     glm::vec3(0,0,0),   // aim point
		     glm::vec3(0,1,0)    // up vector
		     );
  
  // multiply to get MVP
  
  glm::mat4 MVP = Projection * View * Model;
  
  // make this transform available to shaders  
  
  glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
  
}

//----------------------------------------------------------------------------

int main(int argc, char** argv)
{
  // Initialise GLFW

  if ( !glfwInit() ) {
    fprintf( stderr, "Failed to initialize GLFW\n" );
    return -1;
  }

  // initialize scene (must be done before scene file is parsed)

  init_raytracing();
  
  // parse scene file (expected on command line)

  if (argc == 2)
    parse_scene_file(argv[1], ray_cam);
  else {
    printf("missing .scene file\n");
    exit(1);
  }

  // finish GLFW business

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  
  // Open a window and create its OpenGL context

  window = glfwCreateWindow( ray_cam->im->w, ray_cam->im->h, "udray", NULL, NULL);
  if( window == NULL ){
    fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  
  // Initialize GLEW

  glewExperimental = true; // Needed for core profile
  if (glewInit() != GLEW_OK) {
    fprintf(stderr, "Failed to initialize GLEW\n");
    return -1;
  }
  
  // Ensure we can capture the escape key being pressed below

  glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
  
  // clear the window

  glClearColor(0.0f, 0.0f, 1.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT);   // only need to do this once
  
  // Create and compile our GLSL program from the shaders

  //cant find my shader for some reason?
  programID = LoadShaders( "C:/Users/Myles/Documents/School/UD/Graphics/ogl-OpenGL-tutorialV2/ogl-OpenGL-tutorial_0015_33/UDRay/UDRayVertexShader.vertexshader", 
						   "C:/Users/Myles/Documents/School/UD/Graphics/ogl-OpenGL-tutorialV2/ogl-OpenGL-tutorial_0015_33/UDRay/UDRayFragmentShader.fragmentshader");
  
  // Get a handle for our "MVP" uniform

  MatrixID = glGetUniformLocation(programID, "MVP");

  // use default background image as initial texture

  texID = glGetUniformLocation(programID, "tex_udray");

  glGenTextures(1, &teximage);
  glBindTexture(GL_TEXTURE_2D, teximage);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ray_cam->im->w, ray_cam->im->h, 0, GL_RGBA, GL_UNSIGNED_BYTE, ray_cam->im->data);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); 
  glGenerateMipmap(GL_TEXTURE_2D);

  // compute billboard vertices/tex coords

  generate_billboard();

  // Use our shader
  
  glUseProgram(programID);

  // initialize "camera" (not for ray tracing -- just for viewing quad that has ray traced image texture on it)
  
  setup_ortho2d_camera(ray_cam->im->w, ray_cam->im->h);

  // draw/event loop

  do {
    
    // ray trace another pixel if the image isn't finished yet...

    if (image_j < ray_cam->im->h) {

      raytrace_one_pixel(image_i, image_j);

      image_i++;

      // finished a row

      if (image_i == ray_cam->im->w) {

	printf("%i [%i]\n", image_j, ray_cam->im->w);
	fflush(stdout);

	image_i = 0;
	image_j++;
      }    
	  continue;
    }

    // ...or write rendered image to file when done
    
    else if (!wrote_image) {
      
      write_PPM("output.ppm", ray_cam->im);

      wrote_image = true;

      printf("Done rendering and image written...quit when ready\n");
    }

    // rebind texture (so we see latest pixel)

    glBindTexture(GL_TEXTURE_2D, teximage);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ray_cam->im->w, ray_cam->im->h, 0, GL_RGBA, GL_UNSIGNED_BYTE, ray_cam->im->data);

    // draw quad with new texture affixed

    draw_billboard();

    // Swap buffers

    glfwSwapBuffers(window);
    glfwPollEvents();
    
  } // Check if the ESC key was pressed or the window was closed
  while ( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS && glfwWindowShouldClose(window) == 0 );

  // Cleanup 

  glDeleteBuffers(1, &billboard_vertexbuffer);
  glDeleteBuffers(1, &billboard_uvbuffer);
  glDeleteProgram(programID);
  glDeleteTextures(1, &texID);

  glDeleteVertexArrays(1, &VertexArrayID);

  // Close OpenGL window and terminate GLFW

  glfwTerminate();
  
  return 0;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
