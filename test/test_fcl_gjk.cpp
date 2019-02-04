/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, CNRS-LAAS.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Florent Lamiraux <florent@laas.fr> */

#define BOOST_TEST_MODULE FCL_GJK
#define BOOST_TEST_DYN_LINK

#include <time.h>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/shape/geometric_shapes.h>

using fcl::GJKSolver_indep;
using fcl::TriangleP;
using fcl::Vec3f;
using fcl::Quaternion3f;
using fcl::Transform3f;
using fcl::Matrix3f;
using fcl::FCL_REAL;

typedef Eigen::Matrix<FCL_REAL, Eigen::Dynamic, 1> vector_t;
typedef Eigen::Matrix<FCL_REAL, 6, 1> vector6_t;
typedef Eigen::Matrix<FCL_REAL, 4, 1> vector4_t;
typedef Eigen::Matrix<FCL_REAL, Eigen::Dynamic, Eigen::Dynamic> matrix_t;



BOOST_AUTO_TEST_CASE(triangleDistanceAssertionFails)
{
    Vec3f a1,b1,c1,a2,b2,c2;

    a1 << -0.8027043342590332 , -0.30276307463645935 , -0.4372950792312622;
    b1 << -0.8027043342590332 , 0.30276307463645935  ,-0.4372950792312622 ;
    c1 << 0.8027043342590332  , 0.30276307463645935  ,-0.4372950792312622 ;

    a2 << -0.224713996052742   , -0.7417119741439819 , 0.19999997317790985  ;
    b2 <<  -0.5247139930725098 , -0.7417119741439819 , 0.19999997317790985  ;
    c2 << -0.224713996052742   , -0.7417119741439819 ,  0.09999997168779373 ;

    fcl::Transform3f tf1, tf2;
    Matrix3f R; Vec3f T;
    R <<  0.9657787025454787,      0.09400415350535746,   0.24173273843919627,
             -0.06713698817647556,    0.9908494114820345,    -0.11709000206805695,
             -0.25052768814676646,    0.09685382227587608,  0.9632524147814993;

    T << -0.13491177905469953, -1,  0.6000449621843792;
    tf1.setRotation(R); tf1.setTranslation(T);

    Vec3f p1, p2, normal;
    FCL_REAL dist;
    TriangleP s1(a1,b1,c1);
    TriangleP s2(a2,b2,c2);

    GJKSolver_indep solver;
    bool succes = solver.shapeDistance<TriangleP, TriangleP>
    (s1,tf1, s2, tf2, dist, p1, p2, normal);
    bool tg;

}


struct Result
{
  bool collision;
  clock_t timeGjk;
  clock_t timeGte;
}; // struct benchmark

typedef std::vector <Result> Results_t;

BOOST_AUTO_TEST_CASE(distance_triangle_triangle_1)
{
  Eigen::IOFormat numpy (Eigen::FullPrecision, Eigen::DontAlignCols, ", ",
                         ", ", "np.array ((", "))", "", "");
  Eigen::IOFormat tuple (Eigen::FullPrecision, Eigen::DontAlignCols, "",
                         ", ", "", "", "(", ")");
  std::size_t N = 10000;
  GJKSolver_indep solver;
  Transform3f tf1, tf2;
  Vec3f p1, p2, a1, a2;
  Matrix3f M;
  FCL_REAL distance (sqrt (-1));
  clock_t start, end;

  std::size_t nCol = 0, nDiff = 0;
  FCL_REAL eps = 1e-7;
  Results_t results (N);
  for (std::size_t i=0; i<N; ++i) {
    Vec3f P1_loc (Vec3f::Random ()), P2_loc (Vec3f::Random ()),
      P3_loc (Vec3f::Random ());
    Vec3f Q1_loc (Vec3f::Random ()), Q2_loc (Vec3f::Random ()),
      Q3_loc (Vec3f::Random ());
    if (i==0) {
      P1_loc = Vec3f (0.063996093749999997, -0.15320971679687501,
                      -0.42799999999999999);
      P2_loc = Vec3f (0.069105957031249998, -0.150722900390625,
                      -0.42999999999999999);
      P3_loc = Vec3f (0.063996093749999997, -0.15320971679687501,
                  -0.42999999999999999);
      Q1_loc = Vec3f (-25.655000000000001, -1.2858199462890625,
                      3.7249809570312502);
      Q2_loc = Vec3f (-10.926, -1.284259033203125, 3.7281499023437501);
      Q3_loc = Vec3f (-10.926, -1.2866180419921875, 3.72335400390625);
      Transform3f tf
        (Quaternion3f (-0.42437287410898855, -0.26862477561450587,
                       -0.46249645019513175, 0.73064726592483387),
         Vec3f (-12.824601270753471, -1.6840516940066426, 3.8914453043793844));
      tf1 = tf;
    } else {
      tf1 = Transform3f ();
    }

    TriangleP tri1 (P1_loc, P2_loc, P3_loc);
    TriangleP tri2 (Q1_loc, Q2_loc, Q3_loc);
    Vec3f normal;

    bool res;
    start = clock ();
    res = solver.shapeDistance (tri1, tf1, tri2, tf2, distance, p1, p2, normal);
    end = clock ();
    results [i].timeGjk = end - start;
    results [i].collision = !res;
    assert (res == (distance > 0));
    if (!res) {
      Vec3f c1, c2, normal2;
      ++nCol;
      // check that moving triangle 2 by the penetration depth in the
      // direction of the normal makes the triangles collision free.
      FCL_REAL penetration_depth (-distance);
      assert (penetration_depth > 0);
      tf2.setTranslation ((penetration_depth + 10-4) * normal);
      res = solver.shapeDistance (tri1, tf1, tri2, tf2, distance, c1, c2,
                                  normal2);
      if (!res) {
        std::cerr << "P1 = " << P1_loc.format (tuple) << std::endl;
        std::cerr << "P2 = " << P2_loc.format (tuple) << std::endl;
        std::cerr << "P3 = " << P3_loc.format (tuple) << std::endl;
        std::cerr << "Q1 = " << Q1_loc.format (tuple) << std::endl;
        std::cerr << "Q2 = " << Q2_loc.format (tuple) << std::endl;
        std::cerr << "Q3 = " << Q3_loc.format (tuple) << std::endl;
        std::cerr << "p1 = " << c1.format (tuple) << std::endl;
        std::cerr << "p2 = " << c2.format (tuple) << std::endl;
        std::cerr << "tf1 = " << tf1.getTranslation ().format (tuple)
                  << " + " << tf1.getQuatRotation ().coeffs ().format (tuple)
                  << std::endl;
        std::cerr << "tf2 = " << tf2.getTranslation ().format (tuple)
                  << " + " << tf2.getQuatRotation ().coeffs ().format (tuple)
                  << std::endl;
        std::cerr << "normal = " << normal.format (tuple) << std::endl;
        abort ();
      }
      distance = 0;
      tf2.setIdentity ();
    }
    // Compute vectors between vertices
    Vec3f P1 (tf1.transform (P1_loc)), P2 (tf1.transform (P2_loc)),
      P3 (tf1.transform (P3_loc)), Q1 (tf2.transform (Q1_loc)),
      Q2 (tf2.transform (Q2_loc)), Q3 (tf2.transform (Q3_loc));
    Vec3f u1 (P2 - P1);
    Vec3f v1 (P3 - P1);
    Vec3f w1 (u1.cross (v1));
    Vec3f u2 (Q2 - Q1);
    Vec3f v2 (Q3 - Q1);
    Vec3f w2 (u2.cross (v2));
    BOOST_CHECK (w1.squaredNorm () > eps*eps);
    M.col (0) = u1; M.col (1) = v1; M.col (2) = w1;
    // Compute a1 such that p1 = P1 + a11 u1 + a12 v1 + a13 u1 x v1
    a1 = M.inverse () * (p1 - P1);
    BOOST_CHECK (w2.squaredNorm () > eps*eps);
    // Compute a2 such that p2 = Q1 + a21 u2 + a22 v2 + a23 u2 x v2
    M.col (0) = u2; M.col (1) = v2; M.col (2) = w2;
    a2 = M.inverse () * (p2 - Q1);

    // minimal distance and closest points can be considered as a constrained
    // optimisation problem:
    //
    // min f (a1[0],a1[1], a2[0],a2[1])
    //   g1 (a1[0],a1[1], a2[0],a2[1]) <=0
    //   ...
    //   g6 (a1[0],a1[1], a2[0],a2[1]) <=0
    // with
    //  f (a1[0],a1[1], a2[0],a2[1]) =
    //         1                                                           2
    //        --- dist (P1 + a1[0] u1 + a1[1] v1, Q1 + a2[0] u2 + a2[1] v2),
    //         2
    //  g1 (a1[0],a1[1], a2[0],a2[1]) = -a1[0]
    //  g2 (a1[0],a1[1], a2[0],a2[1]) = -a1[1]
    //  g3 (a1[0],a1[1], a2[0],a2[1]) =  a1[0] + a1[1] - 1
    //  g4 (a1[0],a1[1], a2[0],a2[1]) = -a2[0]
    //  g5 (a1[0],a1[1], a2[0],a2[1]) = -a2[1]
    //  g6 (a1[0],a1[1], a2[0],a2[1]) =  a2[0] + a2[1] - 1

    // Compute gradient of f
    vector4_t grad_f;
    grad_f [0] = -(p2 - p1).dot (u1);
    grad_f [1] = -(p2 - p1).dot (v1);
    grad_f [2] = (p2 - p1).dot (u2);
    grad_f [3] = (p2 - p1).dot (v2);
    vector6_t g;
    g [0] = -a1 [0];
    g [1] = -a1 [1];
    g [2] = a1 [0] + a1 [1] - 1;
    g [3] = -a2 [0];
    g [4] = -a2 [1];
    g [5] = a2 [0] + a2 [1] - 1;
    matrix_t grad_g (4, 6); grad_g.setZero ();
    grad_g (0,0) = -1.;
    grad_g (1,1) = -1;
    grad_g (0,2) = 1; grad_g (1,2) = 1;
    grad_g (2,3) = -1;
    grad_g (3,4) = -1;
    grad_g (2,5) = 1; grad_g (3,5) = 1;
    // Check that closest points are on triangles planes
    // Projection of [P1p1] on line normal to triangle 1 plane is equal to 0
    BOOST_CHECK (fabs (a1 [2]) < eps);
    // Projection of [Q1p2] on line normal to triangle 2 plane is equal to 0
    BOOST_CHECK (fabs (a2 [2]) < eps);

    /* Check Karush–Kuhn–Tucker conditions
                    6
                   __
                   \
         -grad f = /_  c  grad g
                   i=1  i       i

         where c  >= 0, and
                i
         c  g  = 0 for i between 1 and 6
          i  i
    */

    matrix_t Mkkt (4, 6); matrix_t::Index col = 0;
    // Check that constraints are satisfied
    for (vector6_t::Index j=0; j<6; ++j) {
      BOOST_CHECK (g [j] <= eps);
      // if constraint is saturated, add gradient in matrix
      if (fabs (g [j]) <= eps) {
        Mkkt.col (col) = grad_g.col (j); ++col;
      }
    }
    if (col > 0) {
      Mkkt.conservativeResize (4, col);
      // Compute KKT coefficients ci by inverting
      // Mkkt.c = -grad_f
      Eigen::JacobiSVD <matrix_t> svd
        (Mkkt, Eigen::ComputeThinU | Eigen::ComputeThinV);
      vector_t c (svd.solve (-grad_f));
      for (vector_t::Index j=0; j < c.size (); ++j) {
        BOOST_CHECK (c [j] >= -eps);
      }
    }
  }
  std::cerr << "nCol = " << nCol << std::endl;
  std::cerr << "nDiff = " << nDiff << std::endl;
  // statistics
  clock_t totalTimeGjkColl = 0;
  clock_t totalTimeGjkNoColl = 0;
  for (std::size_t i=0; i<N; ++i) {
    if (results [i].collision) {
      totalTimeGjkColl += results [i].timeGjk;
    } else {
      totalTimeGjkNoColl += results [i].timeGjk;
    }
  }
  std::cerr << "Total time gjk: " << totalTimeGjkNoColl + totalTimeGjkColl
            << std::endl;
  std::cerr << "-- Collisions -------------------------" << std::endl;
  std::cerr << "Total time gjk: " << totalTimeGjkColl << std::endl;
  std::cerr << "-- No collisions -------------------------" << std::endl;
  std::cerr << "Total time gjk: " << totalTimeGjkNoColl << std::endl;
}
