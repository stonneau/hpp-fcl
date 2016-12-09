/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

/** \author Jia Pan */


#ifndef FCL_TRAVERSAL_NODE_SHAPES_H
#define FCL_TRAVERSAL_NODE_SHAPES_H

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/traversal/traversal_node_base.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/shape/geometric_shapes_utility.h>
#include <hpp/fcl/BV/BV.h>
#include <hpp/fcl/shape/geometric_shapes_utility.h>
#include <hpp/fcl/ccd/motion.h>

namespace fcl
{


/// @brief Traversal node for collision between two shapes
template<typename S1, typename S2, typename NarrowPhaseSolver>
class ShapeCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  ShapeCollisionTraversalNode() : CollisionTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;

    nsolver = NULL;
  }

  /// @brief BV culling test in one BVTT node
  bool BVTesting(int, int) const
  {
    return false;
  }

  /// @brief BV culling test in one BVTT node
  bool BVTesting(int, int, FCL_REAL&) const
  {
    throw std::runtime_error ("Not implemented");
  }

  /// @brief Intersection testing between leaves (two shapes)
  void leafTesting(int, int, FCL_REAL&) const
  {
    if(model1->isOccupied() && model2->isOccupied())
    {
      bool is_collision = false;
      if(request.enable_contact)
      {
        Vec3f contact_point, normal;
        FCL_REAL penetration_depth;
        if(nsolver->shapeIntersect(*model1, tf1, *model2, tf2, &contact_point, &penetration_depth, &normal))
        {
          is_collision = true;
          if(request.num_max_contacts > result->numContacts())
          {
            result->addContact(Contact(model1, model2, Contact::NONE, Contact::NONE, contact_point, normal, penetration_depth), request.filter_contact_points);
          }
        }
      }
      else
      {
        if(nsolver->shapeIntersect(*model1, tf1, *model2, tf2, NULL, NULL, NULL))
        {
          is_collision = true;
          if(request.num_max_contacts > result->numContacts())
            result->addContact(Contact(model1, model2, Contact::NONE, Contact::NONE));
        }
      }

      if(is_collision && request.enable_cost)
      {
        AABB aabb1, aabb2;
        computeBV<AABB, S1>(*model1, tf1, aabb1);
        computeBV<AABB, S2>(*model2, tf2, aabb2);
        AABB overlap_part;
        aabb1.overlap(aabb2, overlap_part);
        result->addCostSource(CostSource(overlap_part, cost_density), request.num_max_cost_sources);
      }
    }
    else if((!model1->isFree() && !model2->isFree()) && request.enable_cost)
    {
      if(nsolver->shapeIntersect(*model1, tf1, *model2, tf2, NULL, NULL, NULL))
      {
        AABB aabb1, aabb2;
        computeBV<AABB, S1>(*model1, tf1, aabb1);
        computeBV<AABB, S2>(*model2, tf2, aabb2);
        AABB overlap_part;
        aabb1.overlap(aabb2, overlap_part);
        result->addCostSource(CostSource(overlap_part, cost_density), request.num_max_cost_sources);        
      }      
    }
  }

  const S1* model1;
  const S2* model2;

  FCL_REAL cost_density;

  const NarrowPhaseSolver* nsolver;
};

/// @brief Traversal node for distance between two shapes
template<typename S1, typename S2, typename NarrowPhaseSolver>
class ShapeDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  ShapeDistanceTraversalNode() : DistanceTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;

    nsolver = NULL;
  }

  /// @brief BV culling test in one BVTT node
  FCL_REAL BVTesting(int, int) const
  {
    return -1; // should not be used 
  }

  /// @brief Distance testing between leaves (two shapes)
  void leafTesting(int, int) const
  {
    FCL_REAL distance;
    Vec3f closest_p1, closest_p2;
    nsolver->shapeDistance(*model1, tf1, *model2, tf2, &distance, &closest_p1, &closest_p2);
    result->update(distance, model1, model2, DistanceResult::NONE, DistanceResult::NONE, closest_p1, closest_p2);
  }

  const S1* model1;
  const S2* model2;

  const NarrowPhaseSolver* nsolver;
};

template<typename S1, typename S2, typename NarrowPhaseSolver>
class ShapeConservativeAdvancementTraversalNode : public ShapeDistanceTraversalNode<S1, S2, NarrowPhaseSolver>
{
public:
  ShapeConservativeAdvancementTraversalNode() : ShapeDistanceTraversalNode<S1, S2, NarrowPhaseSolver>()
  {
    delta_t = 1;
    toc = 0;
    t_err = (FCL_REAL)0.0001;

    motion1 = NULL;
    motion2 = NULL;
  }

  void leafTesting(int, int) const
  {
    FCL_REAL distance;
    Vec3f closest_p1, closest_p2;
    this->nsolver->shapeDistance(*(this->model1), this->tf1, *(this->model2), this->tf2, &distance, &closest_p1, &closest_p2);

    Vec3f n = this->tf2.transform(closest_p2) - this->tf1.transform(closest_p1); n.normalize();
    TBVMotionBoundVisitor<RSS> mb_visitor1(model1_bv, n);
    TBVMotionBoundVisitor<RSS> mb_visitor2(model2_bv, -n);
    FCL_REAL bound1 = motion1->computeMotionBound(mb_visitor1);
    FCL_REAL bound2 = motion2->computeMotionBound(mb_visitor2);

    FCL_REAL bound = bound1 + bound2;

    FCL_REAL cur_delta_t;
    if(bound <= distance) cur_delta_t = 1;
    else cur_delta_t = distance / bound;

    if(cur_delta_t < delta_t)
      delta_t  = cur_delta_t;
  }

  mutable FCL_REAL min_distance;

  /// @brief The time from beginning point
  FCL_REAL toc;
  FCL_REAL t_err;

  /// @brief The delta_t each step
  mutable FCL_REAL delta_t;

  /// @brief Motions for the two objects in query
  const MotionBase* motion1;
  const MotionBase* motion2;

  RSS model1_bv, model2_bv; // local bv for the two shapes
};


}

#endif
