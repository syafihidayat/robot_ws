#ifndef GRID_PLANNER_HPP
#define GRID_PLANNER_HPP

#include <vector>


constexpr int GRID_W = 3;
constexpr int GRID_H = 4;
constexpr int CELL_COUNT = 12;

enum Stage2State
{
    ST2_PLAN,
    ST2_ROTATE,
    ST2_MOVE,
    ST2_REACHED
};

struct GridPlanner
{
    bool obstacle[CELL_COUNT] = {false};

    int last_cell = -1;
    int next_cell = -1;
    Stage2State state = ST2_PLAN;

    int worldToGrid(double x, double y)const;
    double cellToYaw(int from, int to)const;

    std::vector<int>getNeightbors(int cell)const;
    int chooseNextCell(int current, int target);


    void reset();


};

#endif
