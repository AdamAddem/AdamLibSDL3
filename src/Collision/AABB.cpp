/*
  Copyright (c) 2009 Erin Catto http://www.box2d.org
  Copyright (c) 2016-2018 Lester Hedges <lester.hedges+aabbcc@gmail.com>

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.

  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.

  3. This notice may not be removed or altered from any source distribution.

  This code was adapted from parts of the Box2D Physics Engine,
  http://www.box2d.org
*/

/*

    Its called we do a little altering

*/

#include <AdamLib/Nodes/CollisionNode.hpp>
#include <AdamLib/Collision/CollisionShapes.hpp>


#include "AABB.hpp"
#include "cute_c2.hpp"
#include <algorithm>
#include <cassert>
#include <limits>
#include <iostream>


constexpr unsigned int NULL_NODE = std::numeric_limits<unsigned int>::max();

using namespace AdamLib;


constexpr c2AABB aabbConversion(const CollisionRectangle* _rect, const double _xoffset, const double _yoffset)
{
  return 
  {
      {_rect->center_.x - _rect->width_height_.x/2 + _xoffset, _rect->center_.y - _rect->width_height_.y/2 + _yoffset}, 
      {_rect->center_.x + _rect->width_height_.x/2 + _xoffset, _rect->center_.y + _rect->width_height_.y/2 + _yoffset}
  };
}
constexpr c2Circle circleConversion(const CollisionCircle* _circle, const double _xoffset, const double _yoffset)
{
  return 
  {
      {_circle->center_.x + _xoffset, -_circle->center_.y + _yoffset}, 
      _circle->r_
    };
}
constexpr c2Capsule capsuleConversion(const CollisionCapsule* _capsule, const double _xoffset, const double _yoffset)
{
  return 
  {
      {_capsule->a_center_.x + _xoffset, _capsule->a_center_.y + _yoffset}, 
      {_capsule->b_center_.x + _xoffset, _capsule->b_center_.y + _yoffset}, 
      _capsule->r_
    };
}

namespace aabb
{
    AABB::AABB(const AdamLib::AABB& ab) : lowerBound(ab.bottom_left_), upperBound(ab.top_right_), surfaceArea()
    {
      updateSurfaceArea();
      updateCentre();
    }
  
    AABB::AABB(const AdamLib::Vec2& lowerBound_, const AdamLib::Vec2& upperBound_) :
        lowerBound(lowerBound_), upperBound(upperBound_), surfaceArea()
    {
        updateSurfaceArea();
        updateCentre();
    }
    AABB::AABB(const AABB& _aabb1, const AABB& _aabb2)
    {
        lowerBound.x = std::min(_aabb1.lowerBound.x, _aabb2.lowerBound.x);
        upperBound.x = std::max(_aabb1.upperBound.x, _aabb2.upperBound.x);

        lowerBound.y = std::max(_aabb1.lowerBound.y, _aabb2.lowerBound.y);
        upperBound.y = std::min(_aabb1.upperBound.y, _aabb2.upperBound.y);

        surfaceArea = lowerBound.y - upperBound.y;
        updateCentre();
    }
    void AABB::updateSurfaceArea()
    {
        surfaceArea = (lowerBound.y - upperBound.y) * (upperBound.x - lowerBound.x);
    }
    void AABB::merge(const AABB& _aabb1, const AABB& _aabb2)
    {
        lowerBound.x = std::min(_aabb1.lowerBound.x, _aabb2.lowerBound.x);
        upperBound.x = std::max(_aabb1.upperBound.x, _aabb2.upperBound.x);

        lowerBound.y = std::max(_aabb1.lowerBound.y, _aabb2.lowerBound.y);
        upperBound.y = std::min(_aabb1.upperBound.y, _aabb2.upperBound.y);

        updateSurfaceArea();
        updateCentre();
    }
    bool AABB::contains(const AABB& _aabb) const
    {
        if(_aabb.lowerBound.x >= lowerBound.x && _aabb.lowerBound.y <= lowerBound.y
         && _aabb.upperBound.x <= upperBound.x && _aabb.upperBound.y >= upperBound.y)
            return true;

        return false;
    }
    bool AABB::overlaps(const AABB& _aabb) const
    {
        return (_aabb.upperBound.x > lowerBound.x && _aabb.lowerBound.x < upperBound.x &&_aabb.upperBound.y < lowerBound.y && _aabb.lowerBound.y > upperBound.y);
    }
    void AABB::updateCentre()
    {
        centre = {(upperBound.x - lowerBound.x)/2, (lowerBound.y - upperBound.y)/2};
    }

  
    Node::Node(const AdamLib::CollisionNode* _particle) :
    aabb({0,0}, {0,0}), parent(NULL_NODE),
    left(NULL_NODE), right(NULL_NODE), height(-1), particle(_particle)
    {
      if(_particle)
        aabb = {_particle->getAABB()};
    }
    bool Node::isLeaf() const
    {
        return (left == NULL_NODE);
    }

  
    Tree::Tree(double skinThickness_) :
    root(NULL_NODE), skinThickness(skinThickness_) {}

    unsigned int Tree::allocateNode(const CollisionNode* _particle)
    {
        nodes.emplace_back(_particle);
        return nodes.size() - 1;
    }

    void Tree::freeNode(unsigned int node)
    {
      nodes[node].height = -1;
      nodes[node].particle = nullptr;
    }

    void Tree::insertParticle(const CollisionNode* _particle)
    {
        // Allocate a new node for the particle.
        const unsigned int node_index = allocateNode(_particle);

        // AABB size in each dimension.
        const auto& [bottom_left, top_right] = _particle->getAABB();

        // AABB size in each dimension.
        const double sizex = (top_right.x - bottom_left.x) * skinThickness;
        const double sizey = (bottom_left.y - top_right.y) * skinThickness;

        Node& n = nodes[node_index];
        AABB& naabb = n.aabb;

        // Assign and fatten the AABB.
        naabb.lowerBound.x = bottom_left.x - sizex;
        naabb.upperBound.x = top_right.x + sizex;

        naabb.lowerBound.y = bottom_left.y + sizey;
        naabb.upperBound.y = top_right.y - sizey;


        naabb.updateSurfaceArea();
        naabb.updateCentre();

        // Zero the height.
        n.height = 0;

        // Insert a new leaf into the tree.
        insertLeaf(node_index);
      
        n.particle = _particle;
        leaves.insert(_particle);
    }

    void Tree::removeParticle(const CollisionNode* _particle)
    {
      const int node_index = findNode(_particle);
      if(node_index == -1) return;
      
      removeLeaf(node_index);
      freeNode(node_index);
      leaves.erase(_particle);
    }

    void Tree::removeAll()
    {
      nodes.clear();
      leaves.clear();
    }
  
    void Tree::updateParticle(const CollisionNode* _particle, bool _alwaysReinsert)
    {
        // Extract the node.
        const unsigned int node_index = findNode(_particle);
        if(node_index == -1)
          return;
      
        Node& node = nodes[node_index];

        assert(node < nodeCapacity);
        assert(nodes[node].isLeaf());

        // AABB size in each dimension.
        const auto& [bottom_left, top_right] = _particle->getAABB();

        // AABB size in each dimension.
        const double sizex = (top_right.x - bottom_left.x) * skinThickness;
        const double sizey = (bottom_left.y - top_right.y) * skinThickness;

        // Create the new AABB.
        AABB aabb(bottom_left, top_right);

        // No need to update if the particle is still within its fattened AABB.
        if (!_alwaysReinsert && node.aabb.contains(aabb)) return;
      
        // Remove the current leaf.
        removeLeaf(node_index);

        // Assign and fatten the AABB.
        aabb.lowerBound.x = bottom_left.x - sizex;
        aabb.upperBound.x = top_right.x + sizex;

        aabb.lowerBound.y = bottom_left.y + sizey;
        aabb.upperBound.y = top_right.y - sizey;

        aabb.updateSurfaceArea();
        aabb.updateCentre();
      
        // Assign the new AABB.
        node.aabb = aabb;
      
        // Insert a new leaf node.
        insertLeaf(node_index);
    }



    /*
    void Tree::query(const AdamLib::CollisionNode* _particle) const
    {
      const int node_index = findNode(_particle);
      if(node_index == -1)
          return;

      std::vector<unsigned int> stack(256);
      stack.push_back(root);

      std::vector<const AdamLib::CollisionNode*> particles;
    
      const AABB& _aabb = nodes[node_index].aabb;

      while (stack.empty())
      {
          const unsigned int index = stack.back();
          stack.pop_back();
        
          if(index == NULL_NODE) continue;

          if (const Node& node = nodes[index]; _aabb.overlaps(node.aabb))
          {
              // Check that we're at a leaf node.
              if (node.isLeaf())
              {
                  // Can't interact with itself.
                  if (node.particle != _particle)
                      _particle->signalCollisionWith(node.particle);
                
              }
              else
              {
                  stack.push_back(node.left);
                  stack.push_back(node.right);
              }
          }
      }

    }
    */

    void Tree::queryAll() const // this is fucking stupid
    {
      static std::unordered_set<const CollisionNode*> checkedNodes;
      checkedNodes.clear();

      for(const auto particle : leaves)
      {

        const int node_index = findNode(particle);
        if(node_index == -1)
          return;

        static std::vector<unsigned int> stack(256);
        stack.clear();
        stack.push_back(root);
        
        const AABB& _aabb = nodes[node_index].aabb;

        while (!stack.empty())
        {
          const unsigned int index = stack.back();
          stack.pop_back();
        
          if(index == NULL_NODE) continue;

          if (const Node& node = nodes[index]; _aabb.overlaps(node.aabb))
          {
            // Check that we're at a leaf node.
            if (node.isLeaf())
            {
              
              //Don't interact with a particle twice, and not itself
              if (node.particle != particle && !checkedNodes.contains(node.particle))
                determineCollisionBetween(node.particle, particle);
              
            }
            else
            {
              stack.push_back(node.left);
              stack.push_back(node.right);
            }
          }
        }
        
        checkedNodes.insert(particle);
      }
    }

    void Tree::determineCollisionBetween(const CollisionNode* _c1, const CollisionNode* _c2)
      {
        C2_TYPE type1, type2;
        c2AABB rect1, rect2;
        c2Circle circ1, circ2;
        c2Capsule cap1, cap2;
        void *c2_1 = NULL, *c2_2 = NULL;

        //center on the origin for floating point precision purposes.
        double xoffset = std::min(_c1->pos_.x, _c2->pos_.x);
        double yoffset = std::min(_c1->pos_.y, _c2->pos_.y);


    
    

        if(const auto* rect = std::get_if<CollisionRectangle>(&_c1->shape_))
        {
          type1 = C2_TYPE_AABB;
          rect1 = aabbConversion(rect, xoffset + _c1->pos_.x, yoffset + _c1->pos_.y);
          c2_1 = &rect1;
        }
        else if(const auto* circ = std::get_if<CollisionCircle>(&_c1->shape_))
        {
          type1 = C2_TYPE_CIRCLE;
          circ1 = circleConversion(circ, xoffset + _c1->pos_.x, yoffset + _c1->pos_.y);
          c2_1 = &circ1;
        }
        else if(const auto* caps = std::get_if<CollisionCapsule>(&_c1->shape_))
        {
          type1 = C2_TYPE_CAPSULE;
          cap1 = capsuleConversion(caps, xoffset + _c1->pos_.x, yoffset + _c1->pos_.y);
          c2_1 = &cap1;
        }
        else if(auto* ray = std::get_if<CollisionRay>(&_c1->shape_))
        {
          assert(0);
        }
        else
          assert(0);


        if(const auto* rect = std::get_if<CollisionRectangle>(&_c2->shape_))
        {
          type2 = C2_TYPE_AABB;
          rect2 = aabbConversion(rect, xoffset + _c2->pos_.x, yoffset + _c2->pos_.y);
          c2_2 = &rect2;
        }
        else if(const auto* circ = std::get_if<CollisionCircle>(&_c2->shape_))
        {
          type2 = C2_TYPE_CIRCLE;
          circ2 = circleConversion(circ, xoffset + _c2->pos_.x, yoffset + _c2->pos_.y);
          c2_2 = &circ2;
        }
        else if(const auto* caps = std::get_if<CollisionCapsule>(&_c2->shape_))
        {
          type2 = C2_TYPE_CAPSULE;
          cap2 = capsuleConversion(caps, xoffset + _c2->pos_.x, yoffset + _c2->pos_.y);
          c2_2 = &cap2;
        }
        else if(auto* ray = std::get_if<CollisionRay>(&_c2->shape_))
        {
          assert(0);
        }
        else
          assert(0);



        if(c2Collided(c2_1, NULL, type1, c2_2, NULL, type2))
        {
          _c1->signalCollisionWith(_c2);
          _c2->signalCollisionWith(_c1);
        }

      }
  
    void Tree::insertLeaf(unsigned int _index)
    {
        if (root == NULL_NODE)
        {
            root = _index;
            nodes[root].parent = NULL_NODE;
            return;
        }

        // Find the best sibling for the node.

        AABB leafAABB = nodes[_index].aabb;
        unsigned int index = root;

        while (!nodes[index].isLeaf())
        {
            // Extract the children of the node.
            unsigned int left  = nodes[index].left;
            unsigned int right = nodes[index].right;

            double surfaceArea = nodes[index].aabb.getSurfaceArea();

            AABB combinedAABB(nodes[index].aabb, leafAABB);
            double combinedSurfaceArea = combinedAABB.getSurfaceArea();

            // Cost of creating a new parent for this node and the new leaf.
            double cost = 2.0 * combinedSurfaceArea;

            // Minimum cost of pushing the leaf further down the tree.
            double inheritanceCost = 2.0 * (combinedSurfaceArea - surfaceArea);

            // Cost of descending to the left.
            double costLeft;
            if (nodes[left].isLeaf())
            {
                AABB aabb(leafAABB, nodes[left].aabb);
                costLeft = aabb.getSurfaceArea() + inheritanceCost;
            }
            else
            {
                AABB aabb(leafAABB, nodes[left].aabb);
                double oldArea = nodes[left].aabb.getSurfaceArea();
                double newArea = aabb.getSurfaceArea();
                costLeft = (newArea - oldArea) + inheritanceCost;
            }

            // Cost of descending to the right.
            double costRight;
            if (nodes[right].isLeaf())
            {
                AABB aabb(leafAABB, nodes[right].aabb);
                costRight = aabb.getSurfaceArea() + inheritanceCost;
            }
            else
            {
                AABB aabb(leafAABB, nodes[right].aabb);
                double oldArea = nodes[right].aabb.getSurfaceArea();
                double newArea = aabb.getSurfaceArea();
                costRight = (newArea - oldArea) + inheritanceCost;
            }

            // Descend according to the minimum cost.
            if ((cost < costLeft) && (cost < costRight)) break;

            // Descend.
            if (costLeft < costRight) index = left;
            else                      index = right;
        }

        unsigned int sibling = index;

        // Create a new parent.
        unsigned int oldParent = nodes[sibling].parent;
        unsigned int newParent = allocateNode();
        nodes[newParent].parent = oldParent;
        nodes[newParent].aabb.merge(leafAABB, nodes[sibling].aabb);
        nodes[newParent].height = nodes[sibling].height + 1;

        // The sibling was not the root.
        if (oldParent != NULL_NODE)
        {
            if (nodes[oldParent].left == sibling) nodes[oldParent].left = newParent;
            else                                  nodes[oldParent].right = newParent;

            nodes[newParent].left = sibling;
            nodes[newParent].right = _index;
            nodes[sibling].parent = newParent;
            nodes[_index].parent = newParent;
        }
        // The sibling was the root.
        else
        {
            nodes[newParent].left = sibling;
            nodes[newParent].right = _index;
            nodes[sibling].parent = newParent;
            nodes[_index].parent = newParent;
            root = newParent;
        }

        // Walk back up the tree fixing heights and AABBs.
        index = nodes[_index].parent;
        while (index != NULL_NODE)
        {
            index = balance(index);

            unsigned int left = nodes[index].left;
            unsigned int right = nodes[index].right;

            assert(left != NULL_NODE);
            assert(right != NULL_NODE);

            nodes[index].height = 1 + std::max(nodes[left].height, nodes[right].height);
            nodes[index].aabb.merge(nodes[left].aabb, nodes[right].aabb);

            index = nodes[index].parent;
        }
    }

    void Tree::removeLeaf(unsigned int _index)
    {
        if (_index == root)
        {
            root = NULL_NODE;
            return;
        }

        const unsigned int parentIndex = nodes[_index].parent;
        const Node& parentNode = nodes[parentIndex];

        const unsigned int grandParentIndex = nodes[parentIndex].parent;
        const unsigned int siblingIndex = (parentNode.left == _index) ? parentNode.right : parentNode.left;
        Node& siblingNode = nodes[siblingIndex];

        // Destroy the parent and connect the sibling to the grandparent.
        if (grandParentIndex != NULL_NODE)
        {
          Node& grandParentNode = nodes[grandParentIndex];
          
            if (grandParentNode.left == parentIndex) grandParentNode.left = siblingIndex;
            else                                     grandParentNode.right = siblingIndex;

            siblingNode.parent = grandParentIndex;
            freeNode(parentIndex);

            // Adjust ancestor bounds.
            unsigned int index = grandParentIndex;
            while (index != NULL_NODE)
            {
                index = balance(index);

                unsigned int left = nodes[index].left;
                unsigned int right = nodes[index].right;

                nodes[index].aabb.merge(nodes[left].aabb, nodes[right].aabb);
                nodes[index].height = 1 + std::max(nodes[left].height, nodes[right].height);

                index = nodes[index].parent;
            }
        }
        else
        {
            root = siblingIndex;
            siblingNode.parent = NULL_NODE;
            freeNode(parentIndex);
        }
    }

    int Tree::findNode(const AdamLib::CollisionNode *_particle) const
    {
      int node_index = 0;
      for(auto& f : nodes)
      {
        if(f.particle == _particle)
          return node_index;

        ++node_index;
      }

      return -1;

    }
  
    unsigned int Tree::balance(unsigned int node)
    {
        assert(node != NULL_NODE);

        if (nodes[node].isLeaf() || (nodes[node].height < 2))
            return node;

        unsigned int left = nodes[node].left;
        unsigned int right = nodes[node].right;

        assert(left < nodeCapacity);
        assert(right < nodeCapacity);

        int currentBalance = nodes[right].height - nodes[left].height;

        // Rotate right branch up.
        if (currentBalance > 1)
        {
            unsigned int rightLeft = nodes[right].left;
            unsigned int rightRight = nodes[right].right;

            assert(rightLeft < nodeCapacity);
            assert(rightRight < nodeCapacity);

            // Swap node and its right-hand child.
            nodes[right].left = node;
            nodes[right].parent = nodes[node].parent;
            nodes[node].parent = right;

            // The node's old parent should now point to its right-hand child.
            if (nodes[right].parent != NULL_NODE)
            {
                if (nodes[nodes[right].parent].left == node) nodes[nodes[right].parent].left = right;
                else
                {
                    assert(nodes[nodes[right].parent].right == node);
                    nodes[nodes[right].parent].right = right;
                }
            }
            else root = right;

            // Rotate.
            if (nodes[rightLeft].height > nodes[rightRight].height)
            {
                nodes[right].right = rightLeft;
                nodes[node].right = rightRight;
                nodes[rightRight].parent = node;
                nodes[node].aabb.merge(nodes[left].aabb, nodes[rightRight].aabb);
                nodes[right].aabb.merge(nodes[node].aabb, nodes[rightLeft].aabb);

                nodes[node].height = 1 + std::max(nodes[left].height, nodes[rightRight].height);
                nodes[right].height = 1 + std::max(nodes[node].height, nodes[rightLeft].height);
            }
            else
            {
                nodes[right].right = rightRight;
                nodes[node].right = rightLeft;
                nodes[rightLeft].parent = node;
                nodes[node].aabb.merge(nodes[left].aabb, nodes[rightLeft].aabb);
                nodes[right].aabb.merge(nodes[node].aabb, nodes[rightRight].aabb);

                nodes[node].height = 1 + std::max(nodes[left].height, nodes[rightLeft].height);
                nodes[right].height = 1 + std::max(nodes[node].height, nodes[rightRight].height);
            }

            return right;
        }

        // Rotate left branch up.
        if (currentBalance < -1)
        {
            unsigned int leftLeft = nodes[left].left;
            unsigned int leftRight = nodes[left].right;

            assert(leftLeft < nodeCapacity);
            assert(leftRight < nodeCapacity);

            // Swap node and its left-hand child.
            nodes[left].left = node;
            nodes[left].parent = nodes[node].parent;
            nodes[node].parent = left;

            // The node's old parent should now point to its left-hand child.
            if (nodes[left].parent != NULL_NODE)
            {
                if (nodes[nodes[left].parent].left == node) nodes[nodes[left].parent].left = left;
                else
                {
                    assert(nodes[nodes[left].parent].right == node);
                    nodes[nodes[left].parent].right = left;
                }
            }
            else root = left;

            // Rotate.
            if (nodes[leftLeft].height > nodes[leftRight].height)
            {
                nodes[left].right = leftLeft;
                nodes[node].left = leftRight;
                nodes[leftRight].parent = node;
                nodes[node].aabb.merge(nodes[right].aabb, nodes[leftRight].aabb);
                nodes[left].aabb.merge(nodes[node].aabb, nodes[leftLeft].aabb);

                nodes[node].height = 1 + std::max(nodes[right].height, nodes[leftRight].height);
                nodes[left].height = 1 + std::max(nodes[node].height, nodes[leftLeft].height);
            }
            else
            {
                nodes[left].right = leftRight;
                nodes[node].left = leftLeft;
                nodes[leftLeft].parent = node;
                nodes[node].aabb.merge(nodes[right].aabb, nodes[leftLeft].aabb);
                nodes[left].aabb.merge(nodes[node].aabb, nodes[leftRight].aabb);

                nodes[node].height = 1 + std::max(nodes[right].height, nodes[leftLeft].height);
                nodes[left].height = 1 + std::max(nodes[node].height, nodes[leftRight].height);
            }

            return left;
        }

        return node;
    }

    unsigned int Tree::computeHeight() const
    {
        return computeHeight(root);
    }

    unsigned int Tree::computeHeight(unsigned int node) const
    {
        assert(node < nodeCapacity);

        if (nodes[node].isLeaf()) return 0;

        unsigned int height1 = computeHeight(nodes[node].left);
        unsigned int height2 = computeHeight(nodes[node].right);

        return 1 + std::max(height1, height2);
    }

    unsigned int Tree::getHeight() const
    {
        if (root == NULL_NODE) return 0;
        return nodes[root].height;
    }
  
    void Tree::validate() const
    {
#ifndef NDEBUG
        validateStructure(root);
        validateMetrics(root);

        unsigned int freeCount = 0;
        unsigned int freeIndex = freeList;

        while (freeIndex != NULL_NODE)
        {
            assert(freeIndex < nodeCapacity);
            freeIndex = nodes[freeIndex].next;
            freeCount++;
        }

        assert(getHeight() == computeHeight());
        assert((nodeCount + freeCount) == nodeCapacity);
#endif
    }

    /*
    void Tree::rebuild()
    {
        std::vector<unsigned int> nodeIndices(nodes.size());
        unsigned int count = 0;

        for (unsigned int i=0;i<nodeCapacity;i++)
        {
            // Free node.
            if (nodes[i].height < 0) continue;

            if (nodes[i].isLeaf())
            {
                nodes[i].parent = NULL_NODE;
                nodeIndices[count] = i;
                count++;
              nodes.erase(4);
            }
            else nodes.erase(nodes.begin() + i);
        }

        while (count > 1)
        {
            double minCost = std::numeric_limits<double>::max();
            unsigned int iMin = -1, jMin = -1;

            for (unsigned int i=0;i<count;i++)
            {
                AABB aabbi = nodes[nodeIndices[i]].aabb;

                for (unsigned int j=i+1;j<count;j++)
                {
                    AABB aabbj = nodes[nodeIndices[j]].aabb;
                    AABB aabb(aabbi, aabbj);
                    double cost = aabb.getSurfaceArea();

                    if (cost < minCost)
                    {
                        iMin = i;
                        jMin = j;
                        minCost = cost;
                    }
                }
            }

            unsigned int index1 = nodeIndices[iMin];
            unsigned int index2 = nodeIndices[jMin];

            unsigned int parent = allocateNode();
            nodes[parent].left = index1;
            nodes[parent].right = index2;
            nodes[parent].height = 1 + std::max(nodes[index1].height, nodes[index2].height);
            nodes[parent].aabb.merge(nodes[index1].aabb, nodes[index2].aabb);
            nodes[parent].parent = NULL_NODE;

            nodes[index1].parent = parent;
            nodes[index2].parent = parent;

            nodeIndices[jMin] = nodeIndices[count-1];
            nodeIndices[iMin] = parent;
            count--;
        }

        root = nodeIndices[0];

        validate();
    }
    */
  
    void Tree::validateStructure(unsigned int node) const
    {
        if (node == NULL_NODE) return;

        if (node == root) assert(nodes[node].parent == NULL_NODE);

        unsigned int left = nodes[node].left;
        unsigned int right = nodes[node].right;

        if (nodes[node].isLeaf())
        {
            assert(left == NULL_NODE);
            assert(right == NULL_NODE);
            assert(nodes[node].height == 0);
            return;
        }

        assert(left < nodeCapacity);
        assert(right < nodeCapacity);

        assert(nodes[left].parent == node);
        assert(nodes[right].parent == node);

        validateStructure(left);
        validateStructure(right);
    }

    void Tree::validateMetrics(unsigned int node) const
    {
        if (node == NULL_NODE) return;

        const unsigned int left = nodes[node].left;
        const unsigned int right = nodes[node].right;

        if (nodes[node].isLeaf())
        {
            assert(left == NULL_NODE);
            assert(right == NULL_NODE);
            assert(nodes[node].height == 0);
            return;
        }

        assert(left < nodeCapacity);
        assert(right < nodeCapacity);

        const int height1 = nodes[left].height;
        const int height2 = nodes[right].height;
        const int height = 1 + std::max(height1, height2);
        (void)height; // Unused variable in Release build
        assert(nodes[node].height == height);

        AABB  aabb(nodes[left].aabb, nodes[right].aabb);


        assert(aabb.lowerBound.x == nodes[node].aabb.lowerBound.x);
        assert(aabb.upperBound.x == nodes[node].aabb.upperBound.x);

        assert(aabb.lowerBound.y == nodes[node].aabb.lowerBound.y);
        assert(aabb.upperBound.y == nodes[node].aabb.upperBound.y);


        validateMetrics(left);
        validateMetrics(right);
    }



}
