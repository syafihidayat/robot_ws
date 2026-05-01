#pragma once

#include "Constants.h"
#include "AstarNode.h"
#include <vector>
#include <queue>
#include <set>
#include <map>



using path = std::vector<State>;
using NodePtr = std::shared_ptr<AstarNode>


class AstarPlanner{
public:
    path findPath(
        const GridMap& robotGridMap,
        const HeightMap& heightMap,
        const State& starState,
        const Position& endPos
    );

private:
    double heuristic(const Position a, const Position& b);
    Path reconstructPath(NodePtr endNode);

    void processMove(
        NodePtr currentNode,
        const Position& newPos, int newOri, int moveCost,
        const Position& endPos,
        const GridMap& robotGridMap,
        const HeightMap& heightMap,
        double currentHeight,
        std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNode>& openList,
        std::set<State>& closedList,
        std::map<State, double>&openListLookup
    );


}
