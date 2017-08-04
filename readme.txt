readme.txt

Myles Johnson-Gray & Aadish Chaubal - HW#4 Assignment

-In Visual Studio 2010, the rint function is not supported in udray.cpp, so I replaced it with similar code found on the internet.
-We also included <math.h> and math DEFINES to udray.cpp

-both team members contributed to all parts of the project but specifically we handled:
 Myles- diffuse lighting, specular lighting, shadows, scene generation
 Aadish- reflectance, refraction, ray_intersect sphere, scene generation

 -ray_intersect_sphere is not working as intended...howwver, we think we we're close to an implementation that we were not able to add to the code:

				Intersection *intersect_ray_sphere(Ray *ray, Sphere *S)
				{
					Intersection *nearest_inter = NULL;
					Intersection *inter = NULL;

					Vect p;                // sphere center
					Vect p0,p1,p2;     //ray-sphere intersection points
					VectCopy(p,S->P);  
					double  r=S->radius;   //sphere radius
					Vect O, Dir,tDir;    // ray vectors
					VectCopy(O, ray->orig);
					VectCopy(Dir, ray->dir);
					float   t0, t1, t2;   // params to calc ray-sphere intersect
					double tca, thc, d2;// measurements required to calculate t0 and t1
					Vect L;        //Vector from center of sphere to origin of ray
					Vect N1,N2;   //Normals from the intersection points of ray-sphere


					const double PI = 3.141592653589793238462643383279502884197;
					// Iterate through phi, theta then convert r,theta,phi to  XYZ
					for (double phi = 0.; phi < 2*PI; phi += PI/10.) // Azimuth [0, 2PI]
					{
					for (double theta = 0.; theta < PI; theta += PI/10.) // Elevation [0, PI]
					{
						p1[R] = r * cos(phi) * sin(theta) + p[R];
						p1[G] = r * sin(phi) * sin(theta) + p[G];
						p1[B] = r * cos(theta) + p[B];

						//geometric solution
						VectSub(p1,O,L);
						tca = VectDotProd(L,Dir);
						if (tca < 0) return NULL;
						//VectSub(O, p1, p2);
						t0 = VectMag(L);

						N1[0] = (p1[0] - O[0]); // r;
						N1[1] = (p1[1] - O[1]); // r;
						N1[2] = (p1[2] - O[2]); // r;

						inter = make_intersection();
						inter->t = t0;
						VectCopy(inter->P, p1);
						inter->surf=S->surf;
						VectCopy(inter->N,N1);

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

					return nearest_inter;
				}



