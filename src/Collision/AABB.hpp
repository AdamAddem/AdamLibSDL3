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

#ifndef _aabb_h
#define _aabb_h

#include <AdamLib/Nodes/CollisionNode.hpp>
#include <AdamLib/Utilities/Math.hpp>
#include <vector>
#include <unordered_map>

/// Null node flag.
constexpr unsigned int NULL_NODE = 0xffffffff;

namespace aabb
{
    /*! \brief The axis-aligned bounding box object.

        Axis-aligned bounding boxes (AABBs) store information for the minimum
        orthorhombic bounding-box for an object. Support is provided for
        dimensions >= 2. (In 2D the bounding box is either a rectangle,
        in 3D it is a rectangular prism.)

        Class member functions provide functionality for merging AABB objects
        and testing overlap with other AABBs.
     */
    class AABB
    {
    public:
        //! Constructor.
        /*! \param _lowerBound
                The lower bound in each dimension.

            \param _upperBound
                The upper bound in each dimension.
         */
        AABB(const AdamLib::Vec2& _lowerBound, const AdamLib::Vec2& _upperBound);

        //! Merge Constructor
        /*! \param _aabb1
                The first aabb to merge.

            \param _aabb2
                The second aabb to merge.
        */
        AABB(const AABB& _aabb1, const AABB& _aabb2);



        /// Compute the surface area of the box.
        void updateSurfaceArea();

        /// Get the surface area of the box.
        [[nodiscard]] double getSurfaceArea() const {return surfaceArea;}

        //! Merge two AABBs into this one.
        /*! \param _aabb1
                A reference to the first AABB.

            \param _aabb2
                A reference to the second AABB.
         */
        void merge(const AABB& _aabb1, const AABB& _aabb2);

        //! Test whether the AABB is contained within this one.
        /*! \param _aabb
                A reference to the AABB.

            \return
                Whether the AABB is fully contained.
         */
        [[nodiscard]] bool contains(const AABB& _aabb) const;

        //! Test whether the AABB overlaps this one.
        /*! \param _aabb
                A reference to the AABB.

            \return
                Whether the AABB overlaps.
         */
        [[nodiscard]] bool overlaps(const AABB& _aabb) const;



        //! Update the center of the AABB.
        /*! \returns
                The position vector of the AABB center.
         */
        void updateCentre();

        //! Get the center of the AABB.
        /*! \returns
                The position vector of the AABB center.
         */
        [[nodiscard]] AdamLib::Vec2 getCentre() const {return centre;}


        /// Lower bound of AABB
        AdamLib::Vec2 lowerBound;

        /// Upper bound of AABB
        AdamLib::Vec2 upperBound;

        /// The position of the AABB center.
        AdamLib::Vec2 centre;

        /// The AABB's surface area.
        double surfaceArea;
    };

    /*! \brief A node of the AABB tree.

        Each node of the tree contains an AABB object which corresponds to a
        particle, or a group of particles, in the simulation box. The AABB
        objects of individual particles are "fattened" before they are stored
        to avoid having to continually update and rebalance the tree when
        displacements are small.

        Nodes are aware of their position within in the tree. The isLeaf member
        function allows the tree to query whether the node is a leaf, i.e. to
        determine whether it holds a single particle.
     */
    struct Node
    {
        /// Constructor.
        Node();

        /// The fattened axis-aligned bounding box.
        AABB aabb;

        /// Index of the parent node.
        unsigned int parent;

        /// Index of the next node.
        unsigned int next;

        /// Index of the left-hand child.
        unsigned int left;

        /// Index of the right-hand child.
        unsigned int right;

        /// Height of the node. This is 0 for a leaf and -1 for a free node.
        int height;

        /// The particle that the node contains (leaf nodes only).
        AdamLib::CollisionNode* particle;

        //! Test whether the node is a leaf.
        /*! \return
                Whether the node is a leaf node.
         */
        [[nodiscard]] bool isLeaf() const;
    };

    /*! \brief The dynamic AABB tree.

        The dynamic AABB tree is a hierarchical data structure that can be used
        to efficiently query overlaps between objects of arbitrary shape and size.
     */
    class Tree
    {
    public:
        //! Constructor (non-periodic).
        /*!
            \param skinThickness_
                The skin thickness for fattened AABBs, as a fraction
                of the AABB base length.

            \param nParticles
                The number of particles (for fixed particle number systems).

         */
        explicit Tree(double skinThickness_ = 0.05,
            unsigned int nParticles = 16);


        //! Insert a particle into the tree (arbitrary shape with bounding box).
        /*! \param _particle
                The pointer to the particle.
         */
        void insertParticle(AdamLib::CollisionNode* _particle);

        /// Return the number of particles in the tree.
        unsigned int nParticles() const;

        //! Remove a particle from the tree.
        /*! \param _particle
                The particle pointer (particleMap will be used to map the node).
         */
        void removeParticle(const AdamLib::CollisionNode* _particle);

        //! Remove all particles from the tree.
        void removeAll();

        //! Update the tree if a particle moves outside its fattened AABB.
        /*! \param _particle
                The particle pointer (particleMap will be used to map the node).

            \param _alwaysReinsert
                Always reinsert the particle, even if it's within its old AABB (default: false)
         */
        bool updateParticle(const AdamLib::CollisionNode *_particle, bool _alwaysReinsert = false);

        //! Query the tree to find candidate interactions for a particle.
        /*! \param particle

            \return particles
                A vector of particle indices.
         */
        std::vector<AdamLib::CollisionNode*> query(const AdamLib::CollisionNode *particle);

        //! Query the tree to find candidate interactions for an AABB.
        /*! \param _particle
                The particle pointer.

            \param _aabb
                The AABB.

            \return particles
                A vector of particle pointers.
         */
        std::vector<AdamLib::CollisionNode*> query(const AdamLib::CollisionNode *_particle, const AABB &_aabb) const;

        //! Get a particle AABB.
        /*! \param particle
         */
        const AABB& getAABB(const AdamLib::CollisionNode *particle);

        //! Get the height of the tree.
        /*! \return
                The height of the binary tree.
         */
        unsigned int getHeight() const;

        //! Get the number of nodes in the tree.
        /*! \return
                The number of nodes in the tree.
         */
        unsigned int getNodeCount() const;

        //! Compute the maximum balance of the tree.
        /*! \return
                The maximum difference between the height of two
                children of a node.
         */
        unsigned int computeMaximumBalance() const;

        //! Compute the surface area ratio of the tree.
        /*! \return
                The ratio of the sum of the node surface area to the surface
                area of the root node.
         */
        double computeSurfaceAreaRatio() const;

        /// Validate the tree.
        void validate() const;

        /// Rebuild an optimal tree.
        void rebuild();

    private:
        /// The index of the root node.
        unsigned int root;

        /// The dynamic tree.
        std::vector<Node> nodes;

        /// The current number of nodes in the tree.
        unsigned int nodeCount;

        /// The current node capacity.
        unsigned int nodeCapacity;

        /// The position of node at the top of the free list.
        unsigned int freeList;

        /// The skin thickness of the fattened AABBs, as a fraction of the AABB base length.
        double skinThickness;

        /// A map between particle and node indices.
        std::unordered_map<const AdamLib::CollisionNode*, unsigned int> particleMap;

        //! Allocate a new node.
        /*! \return
                The index of the allocated node.
         */
        unsigned int allocateNode();

        //! Free an existing node.
        /*! \param _index
                The index of the node to be freed.
         */
        void freeNode(unsigned int _index);

        //! Insert a leaf into the tree.
        /*! \param _index
                The index of the leaf node.
         */
        void insertLeaf(unsigned int _index);

        //! Remove a leaf from the tree.
        /*! \param _index
                The index of the leaf node.
         */
        void removeLeaf(unsigned int _index);

        //! Balance the tree.
        /*! \param _node_index
                The index of the node.
         */
        unsigned int balance(unsigned int _node_index);

        //! Compute the height of the tree.
        /*! \return
                The height of the entire tree.
         */
        unsigned int computeHeight() const;

        //! Compute the height of a subtree.
        /*! \param _root_index
                The index of the root node.

            \return
                The height of the subtree.
         */
        unsigned int computeHeight(unsigned int _root_index) const;

        //! Assert that the subtree has a valid structure.
        /*! \param _root_index
                The index of the root node.
         */
        void validateStructure(unsigned int _root_index) const;

        //! Assert that the subtree has valid metrics.
        /*! \param _root_index
                The index of the root node.
         */
        void validateMetrics(unsigned int _root_index) const;

    };
}

namespace AdamLib
{
    struct CollisionTree
    {
        aabb::Tree internal_tree;
        inline CollisionTree(const float _skinThickness, const Vec2& _dimensions) : internal_tree(_skinThickness)
        {

        }

        inline void insertParticle(CollisionNode* _insert)
        {
            internal_tree.insertParticle(_insert);
        }

        inline void removeParticle(CollisionNode* _removal)
        {
            internal_tree.removeParticle(_removal);
        }

        inline std::vector<CollisionNode*> query(CollisionNode* _subject)
        {
            return internal_tree.query(TODO);
        }

        inline void updateParticle(CollisionNode* _updateee)
        {
            internal_tree.updateParticle(_updateee);
        }

    };
}

#endif /* _aabb_h */
