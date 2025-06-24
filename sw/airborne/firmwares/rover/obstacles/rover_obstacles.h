
#ifndef ROVER_OBSTACLES_H
#define ROVER_OBSTACLES_H


#ifndef N_COL_GRID
#define N_COL_GRID 100
#endif

#ifndef N_ROW_GRID
#define N_ROW_GRID 100
#endif

#include "firmwares/rover/navigation.h"
#include "std.h"



typedef struct{
	int8_t world[N_ROW_GRID][N_COL_GRID];
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	float home[2];
	float dx;
	float dy;
	uint16_t now_row;
	int is_ready;
} world_grid;

extern world_grid obstacle_grid;


extern void init_grid(uint8_t pa, uint8_t pb);
extern void init_grid_4(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4);
extern void obtain_cell_xy(float px, float py, int *cell_x, int *cell_y);

extern void fill_cell(float px, float py);
extern void fill_bayesian_cell(float px, float py);
extern void fill_free_cells();


#endif // ROVER_OBSTACLES_H
