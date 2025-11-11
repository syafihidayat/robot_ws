#ifndef FLOOD_H_
#define FLOOD_H_


#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3
#define UNKNOWN 4
#define FLOOD_ONE_CELL 9050

struct cell_info{

    bool walls[4];
    bool visited;
};

struct wall_maze{
    struct cell_info cells[16][16];
};

struct dist_maze{
    int distance[16][16];
};

struct coor
{
    int x;
    int y;
};

struct stack{
    struct coor array[256];
    int index;
};

void init_distance_maze(struct dist_maze * dm, struct coor * c, int center);

void init_wall_maze(struct wall_maze * wm);

void init_coor(struct coor * c, int x, int y);

struct coor pop_stack(struct stack * s);

void push_stack(struct * s, sturct coor c);

void advanceTicksFlood(uint32_t ticks, int d, struct coor * c, struct wall_maze * wm);

int floodfill(struct dist_maze * dm, struct coor* c,int a, int direction, struct stack* upst);

void checkForWalls(struct wall_maze * wm,struct coor * c,int direction, int n, int e, int s, int w);

int minusOneNeighbor(struct dist_maze* dm, struct wall_maze * wm,struct coor * c, struct stack * s,int a);

void showCoor(int x, int y);

void turnOnCenterLEDS(void);
void turnOffcenterLEDS(void);

void advanceOneCell(int direction, struct coor* c, struct wall_maze * wm);

void advanceOnecellVisited(void);

int centerMovement(struct wall_maze * wm, struct coor * c, int direction);

int logicalFlood(struct dist_maze* dm, struct coor * c,struct wall_maze * wm, int direction, struct stack * upst);

void shortestPath(struct dist_maze * dm, struct coor * c, struct wall_maze * wm,int direction, struct stack * upst);

#endif
