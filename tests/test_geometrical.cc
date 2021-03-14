#include <iostream>

#include "ddd.hh"
#include "ddd_utilities.hh"
#include "TicToc.hh"


int main(void)
{

  std::cout << "ACME GEOMETRICAL TESTS"
            << std::endl << std::endl;

  // Instantiate a TicToc object
  TicToc tictoc;

  // POINT TEST
  ddd::point<ddd::Float, 3> A(1.0, 0.0, 0.0); 
  ddd::point<> B(1.0, 0.0, 0.0);
  ddd::point<> D(1.0, 0.0, 0.0);
  ddd::point<ddd::Float, 3> C;
  C=B;
  B=D;

  std::cout << "POINT TESTS" << std::endl
            << "Point A = " << A << std::endl
            << "Point B = " << B << std::endl
            << "Point C = " << C << std::endl 
            << "Point D = " << D << std::endl 
            << std::endl;

  ddd::vector<ddd::Float, 3> vec1;
  ddd::vector<ddd::Float, 3> vec2(A);
  std::cout << vec1 << std::endl  << std::endl;
  vec1 = B;
  std::cout << vec1 << std::endl  << std::endl;
  

  ddd::segment<> seg(A, B);
  std::cout << seg << std::endl << std::endl;

  ddd::ray<ddd::Float, 3> ray;
  std::cout << ray << std::endl << std::endl;
  
  //ddd::triangle<ddd::Float, 3> triangle;
  //std::cout << triangle;
/*
  Eigen::Matrix<ddd::Float, 3, 3> m;
  m << 1, 2, 3,
       4, 5, 6,
       7, 8, 9;
  ddd::point3d<ddd::Float> E = m*A;
  
  std::cout << "POINT TESTS" << std::endl
            //<< "Point A = " << m*A << std::endl
            //<< "Point B = " << m << std::endl
            //<< "Point C = " << m << std::endl 
            << std::endl;*/

  /*/ TRIANGLE TESTS
  ddd::triangle3d Triangle = ddd::make_triangle<ddd::Float>(A, B, C);
  std::cout << "TRIANGLE TESTS" << std::endl
            << "Triangle = " << Triangle << std::endl
            << std::endl;

  // VECTOR TESTS
  ddd::vector3d<ddd::Float> Vector = ddd::make_vector<ddd::Float>(-1.0, -1.0, -1.0);
  std::cout << "VECTOR TESTS" << std::endl
            << "Vector = " << Vector << std::endl
            << std::endl;
  
  // RAY TESTS
  ddd::ray3d Ray = ddd::make_ray<ddd::Float>(D, Vector);
  std::cout << "RAY TESTS" << std::endl
            << "Ray = " << Ray << std::endl
            << std::endl;

  // TRIANGLE-RAY TESTS
  ddd::point3d<ddd::Float> IntPt = ddd::intersection_point(Ray, Triangle);
  std::cout << "RAY TESTS" << std::endl
            << "Intersection point = " << IntPt << std::endl
            << std::endl;

  unsigned int N = 10000;
  std::vector<ddd::point3d<ddd::Float>> IntPtVec;
  IntPtVec.resize(N);
  // Perform timing
  tictoc.tic();
  for (unsigned int i = 0; i < N; i++) {
    IntPtVec[i] = ddd::intersection_point(Ray, Triangle);
  }
  tictoc.toc();
  std::cout << "Time tot = " << tictoc.elapsed_us() << "us" << std::endl
            << "Time avg = " << tictoc.elapsed_us()/N << "us" << std::endl;*/

  return 0;
}
