#include "grid_planner.hpp"
#include <string>
#include <cmath>

static constexpr int height_map[GRID_H][GRID_W]=
{
    {20, 40, 20},
    {40, 60, 40},
    {60, 40, 20},
    {40, 20, 40}
};

ElevTransition getTransition(int from, int to)
{
    int fr = from / GRID_W;
    int fc = from % GRID_W;
    int tr = to / GRID_W;
    int tc = to % GRID_W;

    int dh = height_map[tr][tc] - height_map[fr][fc];

    if(dh > 0) return UP;
    if(dh < 0) return DOWN;
    return LEVEL;

}

int GridPlanner::worldToGrid(double x, double y)const
{
    const double CELL_SIZE = 1.0;
    const double ORIGIN_X = 2.0;
    const double ORIGIN_Y = 0.5;


    int col = (int)((x - ORIGIN_X) / CELL_SIZE);
    int row = (int)((y - ORIGIN_Y) / CELL_SIZE);

    if(col < 0 || col >= GRID_W || row < 0 || row >= GRID_H)
        return -1;

    return row * GRID_W + col;
}

std::vector<int>GridPlanner::getNeightbors(int cell)const
{
    std::vector<int> n;
    int r = cell / GRID_W;
    int c = cell % GRID_W;

    if(c > 0)               n.push_back(cell - 1);
    if(c < GRID_W - 1)      n.push_back(cell + 1);
    if(r > 0)               n.push_back(cell - GRID_W);
    if(r < GRID_H - 1)      n.push_back(cell + GRID_W);


    return n;
}

int GridPlanner::chooseNextCell(int current, int target)
{

    if(current < 0 || target < 0)
        return current;


    auto neighbors = getNeightbors(current);

    double best_cost = 1e9;
    int best = -1;

    int tr = target / GRID_W; 
    int tc = target % GRID_W;

    int cr = current / GRID_W;
    int cc = current % GRID_W;

    for(int n : neighbors)
    {
        if(obstacle[n])continue;
        if(n == last_cell)continue;

        int nr = n / GRID_W;
        int nc = n % GRID_W;

        double cost = std::abs(tr - nr) + std::abs(tc - nc);

        int dh = height_map[nr][nc] - height_map[cr][cc];

        if(dh > 0)
            cost += dh * 0.2;
        else if(dh < 0)
            cost += std::abs(dh) * 0.05;

        if(cost < best_cost)
        {
            best_cost = cost;
            best = n;
        }
    }

    if(best == -1)
        return current;

    return best;

}

double GridPlanner::cellToYaw(int from, int to)const
{
    int fr = from / GRID_W;
    int fc = from % GRID_W;
    int tr = to / GRID_W;
    int tc = to % GRID_W;

    if(tc > fc) return 0.0;
    if(tc < fc) return M_PI;
    if(tr > fr) return M_PI / 2.0;
    return -M_PI / 2.0;
}

void GridPlanner::reset()
{
    last_cell = -1;
    next_cell = -1;
    state = ST2_PLAN;
}