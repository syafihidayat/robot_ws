#pragma once


#include <vector>
#include <map>
#include <set>
#include <string>
#include <cmath>
#include <algorithm>
#include <memory>

constexpr int ROWS = 4;
constexpr int COLS = 3;

enum class kfsType{
    EMPTY = 0,
    R2_KFS = 1,
    OBSTACLE_R1 = 2,
    OBSTACLE_FAKE = 3,
    UNKNOWN = -1
};

constexpr double HEIGHT_200 = 0.2;
constexpr double HEIGHT_400 = 0.4;
constexpr double HEIGHT_600 = 0.6;
constexpr double ALLOWED_STEP_HEIGHT = HEIGHT_200;
constexpr double EPSILON = 0.01;

constexpr int TURN_COST = 1;
constexpr int MOVE_COST = 1;

struct Position{
    int r = 0;
    int c = 0;

    bool operator<(const Position& other) const{
        if(r != other.r) return r < other.r;
        return c < other.c;
    }

    bool operator==(const Position& other) const{
        return r == other.r && c == other.c;
    }

    bool operator!=(const Position& other)const{
        return !(*this == other);
    }
};

struct State{
    Position pos;
    int orientation = 0;

    bool operator<(const State& other) const{
        if (pos.r != other.pos.r) return pos.r < other.pos.r;
        if (pos.c != other.pos.c) return pos.c < other.pos.c;
    }
    bool operator<(const State& other) const{
        return pos == other.pos && orientation == other.orientation;
    }
};

using GridMap = std::vector<std::vector<kfsType>>;
using HeightMap = std::vector<std::vector<double>>;



namespace FieldData{
    const std::vector<std::pair<int, int>> DIRECTIONS = {
        {1, 0},  // 0: 下
        {0, 1},  // 1: 右
        {-1, 0}, // 2: 上
        {0, -1}  // 3: 左
    };

    inline std::map<int, Position> getBlockCoords() {
        return {
            {1, {0, 0}}, {2, {0, 1}}, {3, {0, 2}},
            {4, {1, 0}}, {5, {1, 1}}, {6, {1, 2}},
            {7, {2, 0}}, {8, {2, 1}}, {9, {2, 2}},
            {10, {3, 0}}, {11, {3, 1}}, {12, {3, 2}},
        };
    }

    inline std::map<Position, int> getCoordsToBlock() {
        std::map<Position, int> m;
        for (const auto& pair : getBlockCoords()) {
            m[pair.second] = pair.first;
        }
        return m;
    }

    inline std::set<Position> getEntranceBlocksCoords(){
        return {{0, 0}, {0, 1}, {0, 2}}; // 1, 2, 3
    }
    inline std::set<Position> getExitBlocksCoords() {
        return {{3, 0}, {3, 1}, {3, 2}}; // 10, 11, 12
    }
    
    inline std::set<int> getBoundaryBlocksNums() {
        return {1, 2, 3, 4, 6, 7, 9, 10, 11, 12};
    }

    inline std::vector<Position> getAdjacentNodes(Position pos) {
        std::vector<Position> nodes;
        const std::vector<std::pair<int, int>> adjDirections = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1}
        };
        for (const auto& dir : adjDirections) {
            Position nextPos = {pos.r + dir.first, pos.c + dir.second};
            if (nextPos.r >= 0 && nextPos.r < ROWS && 
                nextPos.c >= 0 && nextPos.c < COLS) {
                nodes.push_back(nextPos);
            }
        }
        return nodes;
    }

     inline std::string getBlockNum(Position pos) {
        static const auto coordsMap = getCoordsToBlock();
        auto it = coordsMap.find(pos);
        if (it != coordsMap.end()) {
            return std::to_string(it->second);
        }
        return "不明";
    }



    inline bool isPositionValid(Position pos) {
        return pos.r >= 0 && pos.r < ROWS && pos.c >= 0 && pos.c < COLS;
    }

}