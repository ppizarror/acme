// GEOMETRY TEST 5 - RAY/TRIANGLE INTERSECTION ON TRIANGLE EDGE

#include <fstream>
#include <iostream>
#include <string>

#include "acme.hh"
#include "acme_math.hh"
#include "acme_intersection.hh"
#include "acme_triangle.hh"
#include "acme_utilities.hh"

using namespace acme;

// Main function
int main()
{
  std::cout
      << " GEOMETRY TEST 5 - LINE/TRIANGLE INTERSECTION ON TRIANGLE EDGE" << std::endl
      << "Angle\tIntersections" << std::endl;

  vec3 V1[3];
  V1[0] = vec3(1.0, 0.0, 0.0);
  V1[1] = vec3(0.0, 1.0, 0.0);
  V1[2] = vec3(-1.0, 0.0, 0.0);

  vec3 V2[3];
  V2[0] = vec3(-1.0, 0.0, 0.0);
  V2[1] = vec3(0.0, -1.0, 0.0);
  V2[2] = vec3(1.0, 0.0, 0.0);

  // Initialize triangle
  triangle Triangle1(V1);
  triangle Triangle2(V2);
  triangle tmp_Triangle1(V1);
  triangle tmp_Triangle2(V2);

  // Initialize rotation matrix
  affine tmp_affine;
  tmp_affine = translate(0.0, 0.0, 0.0);

  // Initialize intersection point
  vec3 IntersectionPointTri1, IntersectionPointTri2;
  bool IntersectionBoolTri1, IntersectionBoolTri2;

  // Initialize Ray
  vec3 LineOrigin(0.0, 0.0, 1.0);
  vec3 LineDirection(0.0, 0.0, -1.0);
  line Line(LineOrigin, LineDirection);

  real_type step = PI / 360.0;
  // Perform intersection at 0.5° step
  for (real_type angle = 0;
       angle < PI;
       angle += step)
  {

    tmp_affine = rotate(angle, "X");
    angle += step;

    tmp_Triangle1 = Triangle1;
    tmp_Triangle2 = Triangle2;
    tmp_Triangle1.transform(tmp_affine);
    tmp_Triangle2.transform(tmp_affine);

    IntersectionBoolTri1 = intersection(Line, tmp_Triangle1, IntersectionPointTri1);
    IntersectionBoolTri2 = intersection(Line, tmp_Triangle2, IntersectionPointTri2);

    std::cout
        << angle * _180divPI << "°\t"
        << "T1 -> " << IntersectionBoolTri1 << ", T2 -> " << IntersectionBoolTri2 << std::endl;

    // ERROR if no one of the two triangles is hit
    if (!IntersectionBoolTri1 && !IntersectionBoolTri2)
    {
      std::cout << "Check coplanarity!" << std::endl;
    }
  }

  // Print triangle normal vector
  vec3 N1 = Triangle1.normal();
  vec3 N2 = Triangle2.normal();
  std::cout
      << std::endl
      << "Triangle 1 face normal = " << N1 << std::endl
      << "Triangle 2 face normal = " << N2 << std::endl
      << std::endl
      << std::endl
      << "GEOMETRY TEST 5: Completed" << std::endl;

  // Exit the program
  return 0;
}
