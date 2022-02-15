/*
(***********************************************************************)
(*                                                                     *)
(* The ACME project                                                    *)
(*                                                                     *)
(* Copyright (c) 2020, Davide Stocco and Enrico Bertolazzi.            *)
(*                                                                     *)
(* The ACME project and its components are supplied under the terms of *)
(* the open source BSD 2-Clause License. The contents of the ACME      *)
(* project and its components may not be copied or disclosed except in *)
(* accordance with the terms of the BSD 2-Clause License.              *)
(*                                                                     *)
(* URL: https://opensource.org/licenses/BSD-2-Clause                   *)
(*                                                                     *)
(*    Davide Stocco                                                    *)
(*    Department of Industrial Engineering                             *)
(*    University of Trento                                             *)
(*    e-mail: davide.stocco@unitn.it                                   *)
(*                                                                     *)
(*    Enrico Bertolazzi                                                *)
(*    Department of Industrial Engineering                             *)
(*    University of Trento                                             *)
(*    e-mail: enrico.bertolazzi@unitn.it                               *)
(*                                                                     *)
(***********************************************************************)
*/

// TEST 17 - DISK/TRIANGLE INTERSECTION

#include <fstream>
#include <iostream>
#include <string>

#include "acme.hh"
#include "acme_intersection.hh"
#include "acme_triangle.hh"
#include "acme_disk.hh"
#include "acme_segment.hh"
#include "acme_utils.hh"

using namespace acme;

// Main function
int main()
{
  std::cout
      << "TEST 17 - DISK/TRIANGLE INTERSECTION" << std::endl;

  // Initialize triangle
  disk Disk(0.3, point(0.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0));

  plane Plane(point(0.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0));

  // Initialize triangle
  triangle Triangle(vec3(0.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), vec3(0.0, 1.0, 1.0));

  // Initialize intersection point
  segment IntersectionSeg, IntersectionSeg1;
  bool IntersectionPtBool;

  //IntersectionPtBool = acme::intersection(Triangle, Disk, IntersectionSeg);
  IntersectionPtBool = acme::intersection(Plane, Triangle, IntersectionSeg1);
  IntersectionPtBool = acme::intersection(IntersectionSeg1, Disk, IntersectionSeg);

  // ERROR if no one of the two triangles is hit
  if (!IntersectionPtBool)
  {
    std::cout << "Check coplanarity!" << std::endl;
  }

  std::cout
      << std::endl
      << std::endl
      << "Intersection segment: " << real(IntersectionPtBool) << std::endl
      << IntersectionSeg.vertex(0) 
      << IntersectionSeg.vertex(1) << std::endl
      << "TEST 17: Completed" << std::endl;

  // Exit the program
  return 0;
}
