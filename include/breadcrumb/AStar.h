/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>

namespace AStar
{
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_) const;
        uint distance(const Vec2i& other_) const;
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        const Node *parent;

        Node(Vec2i coord_, const Node *parent_ = nullptr);
        uint getScore() const;
    };


    struct NodeCost
    {
        const Node* parent;
        const uint G;
    };

    using NodeSet = std::set<Node*>;
    using ConstNodeSet = std::set<const Node*>;

    class Generator
    {
        const Node* findNodeOnList(ConstNodeSet& nodes_, Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);
        void releaseNodes(ConstNodeSet& nodes_);
        bool lineOfSight(const Node* current, const Node* successor) const;

    public:
        Generator();
        bool detectCollision(Vec2i coordinates_) const;
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Vec2i source_, Vec2i target_);
        void addCollision(Vec2i coordinates_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();
        void setThetaStar(bool enable_);

        NodeCost computeCost(const Node* current, Node* successor, const uint c) const;
        uint getStraightLineCost(uint step) const;

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize;
        uint directions;
        bool use_theta_star;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
