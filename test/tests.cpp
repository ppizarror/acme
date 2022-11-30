#include <gtest/gtest.h>
#include <acme/acme.h>

using namespace acme;

// https://www.geogebra.org/m/uxx5pgpb
TEST(AcmeTest, PointOnSegment) { // acme: test_04.cc
    point pA(0.0, 0.0, 0.5);
    point pB(1.0, 1.0, 0.5);
    segment segment1(pA, pB);

    // Query points and intersection bools
    point pointIn(0.5, 0.5, 0.5);
    point pointOut(-1.0, -1.0, 0.5);
    point pointBorder(1.0, 1.0, 0.5);

    ASSERT_TRUE(segment1.isInside(pointIn));
    ASSERT_FALSE(segment1.isInside(pointOut));
    ASSERT_TRUE(segment1.isInside(pointBorder));

    // Test point near
    point pointNear(0.61, 0.57, 0.5);
    ASSERT_TRUE(segment1.isInside(pointNear, 0.1));

    // Test complex segment in 3D
    point pC(0.85, -0.91, -0.84);
    point pD(-0.52, 1.94, 0.43);
    point pE(0.74, -0.68, -0.74);
    point pF(0.12, 0.61, -0.16);
    point pG(-0.36, 1.61, 0.28);
    point pH(1.2, 0.55, 0.5);
    segment segment2(pC, pD);
    ASSERT_NEAR(segment2.length(), 3.40768, 1e-2);
    ASSERT_TRUE(segment2.isInside(pC));
    ASSERT_TRUE(segment2.isInside(pD));
    ASSERT_TRUE(segment2.isInside(pE, 1e-3));
    ASSERT_TRUE(segment2.isInside(pF, 1e-3));
    ASSERT_TRUE(segment2.isInside(pG, 1e-3));
    ASSERT_FALSE(segment2.isInside(pH));

// Create plane in Z axis and find intersection of segment 2
    plane planeZ(point(0, 0, 0), vec3(0, 0, 1));
    point segment2Intersection(0.0, 0.0, 1.0);
    point segment2IntersectionValue(-0.05614, 0.97504, 0);
    ASSERT_TRUE(Intersection(planeZ, segment2, segment2Intersection));
    ASSERT_TRUE(Intersection(segment2Intersection, segment2IntersectionValue, DUMMY_POINT, 1e-3));
}

TEST(AcmeTest, PointVectorTransformation) { // acme: test_08.cc
// Initialize point and vector
    point point_tmp(0.0, 0.0, 1.0);
    vec3 vector_tmp(0.0, 0.0, 1.0);

// Initialize rotation matrix
    affine transformation = translate(0.0, 0.0, -1.0) * angleaxis(PI / 2, UNITX_VEC3);

// Transform objects
    point mov_point(point_tmp);
    vec3 mov_vector(vector_tmp);
    mov_point.transform(transformation);
    Transform(mov_vector, transformation);

// Transform objects back
    point ground_point(mov_point);
    vec3 ground_vector(mov_vector);
    ground_point.transform(transformation.inverse());
    Transform(ground_vector, transformation.inverse());

// Display results
    ASSERT_TRUE(Intersection(point_tmp, ground_point));
    ASSERT_TRUE(Intersection(vector_tmp, ground_vector));
}

TEST(AcmeTEST, RayPointIntersection) {
    point Origin(0, 0, 0);
    vec3 Direction(0, 1, 0);
    ray Ray(Origin, Direction);

    ASSERT_FALSE(Ray.isInside(point(0, -1, 0)));
    ASSERT_FALSE(Ray.isInside(point(0, -100, 0)));
    ASSERT_FALSE(Ray.isInside(point(0.1, 1, 0)));
    ASSERT_TRUE(Ray.isInside(point(0, 0, 0)));
    ASSERT_TRUE(Ray.isInside(point(0, 10, 0)));
}

// https://www.geogebra.org/m/sgxvstab
TEST(AcmeTest, PlaneTriangleIntersection) { // acme: test_11.cc
    point V1[3];
    V1[0] = point(1.0, 0.0, 0.0); // a
    V1[1] = point(0.0, 1.0, 0.0); // b
    V1[2] = point(-1.0, 0.0, 0.0); // c

// Initialize triangle
    triangle Triangle1(V1);

// Initialize triangle
    point PlaneOrigin(0, 0, 0);
    vec3 PlaneNormal(1, 0, 0);
    plane Plane1(PlaneOrigin, PlaneNormal);

// Initialize intersection point
    segment IntersectionSeg;
    bool IntersectionSegBool;

    IntersectionSegBool = Intersection(Plane1, Triangle1, IntersectionSeg);
    ASSERT_TRUE(IntersectionSegBool);
    ASSERT_TRUE(IntersectionSeg.isSegment());
    ASSERT_TRUE(Intersection(IntersectionSeg.vertex(0), V1[1]));
    ASSERT_TRUE(Intersection(IntersectionSeg.vertex(1), vec3(0.0, 0.0, 0.0)));

// Test plane intersection points
    point pD(0, 1.4, 0.8);
    point pDOut(0.1, 1.4, 0.8);
    point pE(0, 1.4, -0.94);
    point pF(0, 1.4, 0.8);
    point pG(0, 1.4, 0.8);
    ASSERT_TRUE(Plane1.isInside(PlaneOrigin));
    ASSERT_TRUE(Plane1.isInside(pD));
    ASSERT_FALSE(Plane1.isInside(pDOut));
    ASSERT_TRUE(Plane1.isInside(pE));
    ASSERT_TRUE(Plane1.isInside(pF));
    ASSERT_TRUE(Plane1.isInside(pG));
}

// https://www.geogebra.org/m/fwahuagn
TEST(AcmeTest, RayRayIntersection) { // acme: test_12.cc
    point pA(0.0, 0.0, 0.0);
    point pB(0.0, -1.0, 0.0);
    point pC(0.0, 1.0, 0.0);
    ray ray0(pA, pB); // pA - pB
    ray ray1(pB, pC); // pB - pC

    segment segment_out;
    ray ray_out;
    bool bool_segment = Intersection(ray0, ray1, segment_out, EPSILON);
    bool bool_ray = Intersection(ray0, ray1, ray_out, EPSILON);

// This condition creates a segment between A and B
    ASSERT_TRUE(bool_segment);
    ASSERT_EQ(segment_out.vertex(0), pA);
    ASSERT_EQ(segment_out.vertex(1), pB);

// There is no ray!
    ASSERT_FALSE(bool_ray);
    ASSERT_TRUE(ray_out.origin().hasNaN());
    ASSERT_TRUE(ray_out.direction().hasNaN());

// Now, create ray from pA to pC and intersect along ray1, which should create a ray from pA to PC
    ray ray2(pA, pC);
    ASSERT_TRUE(Intersection(ray2, ray1, ray_out, EPSILON));
    ASSERT_EQ(ray_out.origin(), pA);
    ASSERT_EQ(ray_out.direction(), pC);
}

TEST(AcmeTest, SegmentSegmentIntersection) { // acme: test_14.cc
    segment segment0(point(0.0, 2.0, 1.0), point(0.0, -1.0, 1.0));
    segment segment1(point(0.0, 1.0, 1.0), point(0.0, -2.0, 1.0));

    segment segment_out;
    ray ray_out;
    bool bool_segment = Intersection(segment0, segment1, segment_out, EPSILON);
    ASSERT_TRUE(bool_segment);
    ASSERT_EQ(segment_out.vertex(0), point(0, -1, 1));
    ASSERT_EQ(segment_out.vertex(1), point(0, 1, 1));
}

// https://www.geogebra.org/m/fvvfptkf
TEST(AcmeTest, Triangle) {
// First, define points
    point pA(2.2, -6.28, -2);
    point pB(6.65, 3.1, 0.43);
    point pC(-3.85, 2.69, 5);
    point pD(-4.71, -5.15, -1.43);
    point pE(1.72, 5.28, -2);
    point pF(2.67, -2.94, 6);
    point pG(-2.32876, -2.94221, -0.27057); // Point inside `T2`
    point pH(2.14156, -2.26423, 4.77723); // Point inside `T2`
    point pI(6.4, -2.97, -3.61);
    point pJ(-2.33, 2.06, 6.81);
    point pK(0.2211, -1.06296, 1.37255); // Point inside `interT1T2Segment`
    point pL(3.16692, -2.17187, 2.8404); // Point inside `pFInterTriLineT1Segment`
    point pM(0.21924, 2.84565, -1.86696); // Point inside `sED`

// Create auxiliar `planeZ`
    plane planeZ(point(0, 0, 0), vec3(0, 0, 1));

// Create triangle `T1` and test
    double TOL = 1e-4;
    segment sAB(pA, pB);
    segment sAC(pA, pC);
    segment sBC(pB, pC);
    triangle T1(pA, pB, pC);
    ASSERT_TRUE(T1.isInside(pA));
    ASSERT_TRUE(T1.isInside(pB));
    ASSERT_TRUE(T1.isInside(pC));
    ASSERT_NEAR(sAB.length(), 10.66264, TOL);
    ASSERT_NEAR(sAC.length(), 12.88656, TOL);
    ASSERT_NEAR(sBC.length(), 11.45875, TOL);
    ASSERT_NEAR(T1.area(), 57.81551, TOL);

// Intersection between `T1` and `planeZ`
    point interT1PlaneZp1(0.47143, -3.71714, 0);
    point interT1PlaneZp2(5.86255, 1.44016, 0);
    segment interT1PlaneZSegment; // Intersection segment
    ASSERT_TRUE(Intersection(planeZ, T1, interT1PlaneZSegment));
    ASSERT_TRUE(Intersection(interT1PlaneZSegment.vertex(0), interT1PlaneZp1, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(interT1PlaneZSegment.vertex(1), interT1PlaneZp2, DUMMY_POINT, TOL));
    ASSERT_NEAR(interT1PlaneZSegment.length(), 7.4607, TOL);

// Create triangle `T2` and test
    segment sDF(pD, pF);
    segment sED(pE, pD);
    segment sEF(pE, pF);
    triangle T2(pD, pF, pE);
    ASSERT_NEAR(sDF.length(), 10.70296, TOL);
    ASSERT_NEAR(sED.length(), 12.266, TOL);
    ASSERT_NEAR(sEF.length(), 11.5096, TOL);
    ASSERT_NEAR(T2.area(), 56.66474, TOL);
    ASSERT_TRUE(T2.isInside(pG, TOL));
    ASSERT_TRUE(T2.isInside(pH, TOL));
    ASSERT_TRUE(sED.isInside(pM, TOL));

// Intersection between `T2` and `planeZ`
    point interT2PlaneZp1(1.9575, 3.225, 0);
    point interT2PlaneZp2(-3.28962, -4.72466, 0);
    segment interT2PlaneZSegment;
    ASSERT_TRUE(Intersection(planeZ, T2, interT2PlaneZSegment));
    ASSERT_TRUE(Intersection(interT2PlaneZSegment.vertex(0), interT2PlaneZp2, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(interT2PlaneZSegment.vertex(1), interT2PlaneZp1, DUMMY_POINT, TOL));
    ASSERT_NEAR(interT2PlaneZSegment.length(), 9.5252, TOL);

// Intersection between `T1` and `T2`
    point interT1T2p1(-0.59592, -2.13465, 1.23495);
    point interT1T2p2(2.15925, 1.47931, 1.69897);
    segment interT1T2Segment;
    ASSERT_TRUE(Intersection(T1, T2, interT1T2Segment, TOL));
    ASSERT_TRUE(Intersection(interT1T2Segment.vertex(0), interT1T2p1, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(interT1T2Segment.vertex(1), interT1T2p2, DUMMY_POINT, TOL));
    ASSERT_NEAR(interT1T2Segment.length(), 4.56803, TOL);
    ASSERT_TRUE(interT1T2Segment.isInside(pK, TOL));

// Create line `triLine` which intersects both triangles
    segment triLineSegment(pI, pJ);
    vec3 triLineDirection;
    triLineDirection = triLineSegment.toUnitVector();
    ASSERT_NEAR(triLineDirection.x(), -0.60229, TOL);
    ASSERT_NEAR(triLineDirection.y(), 0.347028, TOL);
    ASSERT_NEAR(triLineDirection.z(), 0.718893, TOL);
    line triLine(pI, triLineDirection);

// Check `triLine` intersection with `T1`, `T2` and `planeZ`
    point interTriLineT1(3.66866, -1.39627, -0.34991);
    point interTriLineT2(1.50687, -0.1507, 2.23037);
    point interTriLinePlaneZ(3.3755, -1.22736, 0);
    point interPoint; // Dummy point
    ASSERT_TRUE(Intersection(triLine, T1, interPoint));
    ASSERT_TRUE(Intersection(interPoint, interTriLineT1, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(triLine, T2, interPoint));
    ASSERT_TRUE(Intersection(interPoint, interTriLineT2, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(triLine, planeZ, interPoint));
    ASSERT_TRUE(Intersection(interPoint, interTriLinePlaneZ, DUMMY_POINT, TOL));
    segment interTriLineT1T2Segment(interTriLineT1, interTriLineT2);
    ASSERT_NEAR(interTriLineT1T2Segment.length(), 3.58924, TOL);

// Test segment from `pF` to intersection `interTriLineT1`
    segment pFInterTriLineT1Segment(pF, interTriLineT1);
    ASSERT_NEAR(pFInterTriLineT1Segment.length(), 6.61073, TOL);

// Assemble triangle ILE
    segment sEI(pE, pI);
    segment sIL(pI, pL);
    segment sLE(pL, pE);
    triangle T3(pL, pE, pI);
    ASSERT_NEAR(sEI.length(), 9.62065, TOL);
    ASSERT_NEAR(sIL.length(), 7.2593, TOL);
    ASSERT_NEAR(sLE.length(), 9.00296, TOL);
    ASSERT_NEAR(T3.area(), 31.01238, TOL);

// Create triangle `T4` which results of the intersection between `T1`, `T3`, and `planeZ`
    point interSILPlaneZ(4.59059, -2.52332, 0);
    point interT1PlaneZSegmentT3(3.68567, -0.6423, 0);
    point interSABT3(4.45308, -1.53082, -0.76967);
    ASSERT_TRUE(Intersection(planeZ, sIL, interPoint));
    ASSERT_TRUE(Intersection(interPoint, interSILPlaneZ, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(interT1PlaneZSegment, T3, interPoint));
    ASSERT_TRUE(Intersection(interPoint, interT1PlaneZSegmentT3, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(sAB, T3, interPoint));
    ASSERT_TRUE(Intersection(interPoint, interSABT3, DUMMY_POINT, TOL));
    triangle T4(interSILPlaneZ, interT1PlaneZSegmentT3, interSABT3);
    ASSERT_NEAR(T4.area(), 0.86458, TOL);

// Create triangle `T5`... which I don't know maybe this is too much??
    triangle T5(pA, pI, interSILPlaneZ);
    ASSERT_NEAR(T5.area(), 9.64858, TOL);

// Create triangle `T6`, which should be coplanar with `T2` as they share vertices
    triangle T6(interT2PlaneZp1, pG, pM);
    ASSERT_NEAR(T6.area(), 8.21055, TOL);
    ASSERT_TRUE(IsCoplanar(T2, T6, TOL));
}

TEST(AcmeTest, TriangleIsInside) {
    point p1A(5.8, -2.7, 3);
    point p1AA(5.8, -2.7 + 1e-6, 3);
    point p1B(0.39161, -4.78007, 2.34545);
    point p1C(1.5, 1.4, 0);
    triangle T(p1A, p1B, p1C);
    ASSERT_TRUE(T.isInside(p1AA, 1e-4));

    // Test point inside and coplanar, https://www.geogebra.org/m/f7ksrq2g
    point pA(4.2, -1.74, 0);
    point pB(1.5, -0.1, 0);
    point pC(0.6, -5.3, 0);
    triangle t1(pA, pB, pC);
    double TOL = 1e-5; // Same as geogebra

    point pD(2.21, -1.9, 0); // A nice coplanar point
    point pE(-1.13, -3.1, 0); // A point outside the triangle, thus the projection is not inside the triangle
    point pF(1.7, -3, 2); // A point with projection inside the triangle, but no way it is coplanar

    ASSERT_NEAR(t1.area(), 7.758, TOL); // Same as geogebra...
    ASSERT_TRUE(t1.isInside(pD, TOL));
    ASSERT_FALSE(t1.isInside(pE, TOL));
    ASSERT_FALSE(t1.isInside(pF, TOL));

    // Another triangle, but in 3D
    point pG(1.7, 4.9, 0);
    point pH(-0.8, 2, 0);
    point pI(3.8, 3.3, 2);
    triangle t2(pG, pH, pI);

    point pJ(1.747498, 3.575988, 0.6834); // Point within triangle... NICE
    point pK(1.1, 3, 0.3); // Point outside the triangle

    ASSERT_NEAR(t2.area(), 6.33341, TOL);
    ASSERT_TRUE(t2.isInside(pG, TOL));
    ASSERT_TRUE(t2.isInside(pJ, TOL));
    ASSERT_FALSE(t2.isInside(pK, TOL));
}

// https://www.geogebra.org/m/cwhugcww
TEST(AcmeTest, Coplanar) {
    point pA(5.18, -4.14, 5);
    point pB(-1, -6, 0);
    point pC(-2.38, -0.97, 0);
    triangle t(pA, pB, pC);
    plane p(pA, t.normal());
    ASSERT_TRUE(IsCoplanar(p, t));

    // Check points within coplanar
    point pD(-3.50312, -3.66466, -1.39187);
    point pE(-0.57977, 3.29487, 2.21987);
    point pF(3.0048, -5.99035, 2.99497);
    point pG(-0.54011, -2.97172, 0.96462);
    point pH(-0.13817, -4.26827, 0.99916);
    point pI(2.83708, -3.95799, 3.28633);
    point pJ(-1.68638, 0.69418, 0.8596);
    point pK(-2.07675, 0.77881, 0.58521);
    double TOL = 1e-4;

    // Test is inside distance plane
    ASSERT_TRUE(p.isInside(pA, TOL));
    ASSERT_TRUE(p.isInside(pB, TOL));
    ASSERT_TRUE(p.isInside(pC, TOL));
    ASSERT_TRUE(p.isInside(pD, TOL));
    ASSERT_TRUE(p.isInside(pE, TOL));
    ASSERT_TRUE(p.isInside(pF, TOL));
    ASSERT_TRUE(p.isInside(pG, TOL));
    ASSERT_TRUE(p.isInside(pH, TOL));
    ASSERT_TRUE(p.isInside(pI, TOL));

    // Test inside triangle
    ASSERT_TRUE(t.isInside(pA, TOL));
    ASSERT_TRUE(t.isInside(pB, TOL));
    ASSERT_TRUE(t.isInside(pC, TOL));
    ASSERT_FALSE(t.isInside(pD, TOL));
    ASSERT_FALSE(t.isInside(pE, TOL));
    ASSERT_FALSE(t.isInside(pF, TOL));
    ASSERT_TRUE(t.isInside(pG, TOL));
    ASSERT_TRUE(t.isInside(pH, TOL));
    ASSERT_TRUE(t.isInside(pI, TOL));

    // Now, check coplanar segment intersection
    segment sEF(pE, pF);
    point pTEMP;
    segment sTEMP;
    ASSERT_NEAR(sEF.length(), 9.98325, TOL);
    ASSERT_TRUE(Intersection(sEF, t, sTEMP, TOL));
    point interTsEF1(1.73239, -2.69437, 2.71983);
    point interTsEF2(2.59126, -4.91914, 2.90555);
    ASSERT_TRUE(Intersection(sTEMP.vertex(0), interTsEF1, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(sTEMP.vertex(1), interTsEF2, DUMMY_POINT, TOL));

    segment sHI(pH, pI);
    ASSERT_NEAR(sHI.length(), 3.76557, TOL);
    ASSERT_TRUE(Intersection(sHI, t, sTEMP, TOL));
    ASSERT_TRUE(Intersection(sTEMP.vertex(0), pH, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(sTEMP.vertex(1), pI, DUMMY_POINT, TOL));

    segment sAB(pA, pB);
    ASSERT_NEAR(sAB.length(), 8.16407, TOL);
    ASSERT_TRUE(Intersection(sAB, t, sTEMP, TOL));
    ASSERT_TRUE(Intersection(sAB.vertex(0), pA, DUMMY_POINT, TOL));
    ASSERT_TRUE(Intersection(sAB.vertex(1), pB, DUMMY_POINT, TOL));

    segment sDJ(pD, pJ);
    ASSERT_NEAR(sDJ.length(), 5.23155, TOL);
    ASSERT_TRUE(Intersection(sDJ, t, sTEMP, TOL));
    ASSERT_TRUE(Intersection(sTEMP.vertex(0), pC, DUMMY_POINT, TOL));

    segment sDK(pD, pK);
    ASSERT_NEAR(sDK.length(), 5.06832, TOL);
    ASSERT_FALSE(Intersection(sDK, t, sTEMP, TOL));
}

// https://www.geogebra.org/m/mny4rgtq
TEST(AcmeTest, TriangleSegmentIntersection) {
    point pA(3.5, -3, 0);
    point pB(-0.5, -2.5, 0);
    point pC(0.5, -7, 0);
    point pD(-1, -5.5, 0);
    point pE(2.5, -2, 0);
    point pF(1.5, -3.5, 0);
    point pG(0.5, -4.5, 0);
    point pH(1.92491, -5.10012, 0);
    point pI(4, -4.5, 0);
    point pJ(3.5, -2.5, 0);

    segment sDE(pD, pE);
    segment sBD(pB, pD);
    segment sFG(pF, pG);
    segment sCF(pC, pF);
    segment sHI(pH, pI);
    segment sFI(pF, pI);
    segment sEJ(pE, pJ);

    triangle t1(pA, pB, pC);
    double TOL = 1e-4;

    ASSERT_NEAR(sDE.length(), 4.94975, TOL);
    ASSERT_NEAR(sBD.length(), 3.04138, TOL);
    ASSERT_NEAR(sFG.length(), 1.41421, TOL);
    ASSERT_NEAR(sCF.length(), 3.64005, TOL);
    ASSERT_NEAR(sHI.length(), 2.16012, TOL);
    ASSERT_NEAR(sFI.length(), 2.69258, TOL);
    ASSERT_NEAR(sEJ.length(), 1.11803, TOL);

    // Test intersection
    segment sTEMP;
    ASSERT_TRUE(Intersection(sDE, t1, sTEMP, TOL));
    ASSERT_TRUE(Intersection(sBD, t1, sTEMP, TOL));
    ASSERT_TRUE(Intersection(sFG, t1, sTEMP, TOL));
    ASSERT_TRUE(Intersection(sCF, t1, sTEMP, TOL));
    ASSERT_TRUE(Intersection(sHI, t1, sTEMP, TOL));
    ASSERT_TRUE(Intersection(sFI, t1, sTEMP, TOL));
    ASSERT_FALSE(Intersection(sEJ, t1, sTEMP, TOL));

    // Test 3d case
    point p1(-2.1, -5.3, 0);
    point p1A(-2.1, -5.3 + 1e-6, 1e-6); // Under TOL=1e-4, it should be equal to p1
    point p2(3.57755, -3.51632, 1.2517);
    point p3(1.7, -0.7, 0);
    point p4(4.1, -4.3, 1.6);
    triangle tri(p1, p2, p3);
    triangle triA(p1A, p2, p3);
    segment s(p1, p4);
    ASSERT_TRUE(Intersection(s, tri, sTEMP, TOL));
    ASSERT_TRUE(Intersection(s, triA, sTEMP, TOL));
}

// https://www.geogebra.org/m/zhvmvfp8
TEST(AcmeTest, TriangleIntersection) {
    point pA(3, -1, 0);
    point pB(3, -5, 0);
    point pC(7, -5, 0);
    point pD(-1, -5, 0);
    point pE(2, -3, 0);
    point pF(-1, -1, 0);
    point pG(5, -1, 0);
    point pH(4, -3, 0);
    point pI(3, 0, 0);
    point pJ(-3, -4, 0);
    point pK(1, -3, 0);
    point pL(-3, -2, 0);
    point pM(4, -4, 0);
    point pN(0, -4, 0);
    point pO(2, -6, 0);
    point pP(2, 1, 0);
    point pQ(2, -7, 0);
    point pR(8, -3, 0);
    point pS(-4, -3, -1);
    point pT(0, -3, 0);
    point pU(-1, -3, 3);
    point pV(-2, 0, 0);
    point pW(1, 0, 0);
    point pX(-1, 1, 0);
    point pY(7, -3, 0);
    point pZ(6, -2, 0);
    point pAA(6, -3, 0);

    // Create triangles
    triangle t1(pA, pB, pC);
    triangle t2(pD, pE, pF);
    triangle t3(pG, pH, pI);
    triangle t4(pJ, pK, pL);
    triangle t5(pM, pN, pO);
    triangle t6(pP, pQ, pR);
    triangle t7(pS, pT, pU);
    triangle t8(pV, pW, pX);
    triangle t9(pY, pZ, pAA);

    // Test in/out, and out/in (symmetric)
    ASSERT_TRUE(Intersection(t6, t9, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t9, t6, DUMMY_NONE));

    // Test each triangle intersection. Note intersection is symetric
    ASSERT_TRUE(Intersection(t1, t1, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t1, t2, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t1, t3, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t1, t4, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t1, t5, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t1, t6, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t1, t7, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t1, t8, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t1, t9, DUMMY_NONE));

    ASSERT_TRUE(Intersection(t2, t2, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t2, t3, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t2, t4, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t2, t5, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t2, t6, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t2, t7, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t2, t8, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t2, t9, DUMMY_NONE));

    ASSERT_TRUE(Intersection(t3, t3, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t3, t4, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t3, t5, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t3, t6, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t3, t7, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t3, t8, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t3, t9, DUMMY_NONE));

    ASSERT_TRUE(Intersection(t4, t4, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t4, t5, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t4, t6, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t4, t7, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t4, t8, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t4, t9, DUMMY_NONE));

    ASSERT_TRUE(Intersection(t5, t5, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t5, t6, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t5, t7, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t5, t8, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t5, t9, DUMMY_NONE));

    ASSERT_TRUE(Intersection(t6, t6, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t6, t7, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t6, t8, DUMMY_NONE));
    ASSERT_TRUE(Intersection(t6, t9, DUMMY_NONE));

    ASSERT_TRUE(Intersection(t7, t7, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t7, t8, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t7, t9, DUMMY_NONE));

    ASSERT_TRUE(Intersection(t8, t8, DUMMY_NONE));
    ASSERT_FALSE(Intersection(t8, t9, DUMMY_NONE));

    ASSERT_TRUE(Intersection(t9, t9, DUMMY_NONE));
}
