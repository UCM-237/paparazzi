
#include "./rover_obstacles.h"
#include "modules/datalink/telemetry.h"


PRINT_CONFIG_VAR(N_ROW_GRID)
PRINT_CONFIG_VAR(N_COL_GRID)

world_grid obstacle_grid;


#if PERIODIC_TELEMETRY
static void send_obstacle_grid(struct transport_tx *trans, struct link_device *dev)
{


  // Send all cols from obstacle_grid.now_row in a cyclic pattern
  pprz_msg_send_OBSTACLE_GRID(trans, dev, AC_ID,
  				&obstacle_grid.dx,
  				&obstacle_grid.dy,
  				&obstacle_grid.xmin,
  				&obstacle_grid.xmax,
  				&obstacle_grid.ymin,
  				&obstacle_grid.ymax,
  				&obstacle_grid.now_row,
  				N_COL_GRID, obstacle_grid.world[obstacle_grid.now_row]);
  				
  obstacle_grid.now_row = (obstacle_grid.now_row + 1) % N_ROW_GRID;
}
#endif

void init_grid(uint8_t pa, uint8_t pb){

	int i,j;
	for(i = 0; i < N_ROW_GRID; i++){
		for(j = 0; j < N_COL_GRID; j++){
			obstacle_grid.world[i][j] = 0;
		}
	}
	// Rows in X, cols in Y
	obstacle_grid.xmin = WaypointX(pa);
	obstacle_grid.xmax = WaypointX(pb);
	obstacle_grid.ymin = WaypointY(pa);
	obstacle_grid.ymax = WaypointY(pb);
	obstacle_grid.dx   = (obstacle_grid.xmax-obstacle_grid.xmin)/((float)N_COL_GRID); 
	obstacle_grid.dy   = (obstacle_grid.ymax-obstacle_grid.ymin)/((float)N_ROW_GRID);
	obstacle_grid.now_row = 0;
	
	#if PERIODIC_TELEMETRY
  	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OBSTACLE_GRID, send_obstacle_grid);
  	#endif	
  	obstacle_grid.is_ready = 1;

}

void obtain_cell_xy(float px, float py, int *cell_x, int *cell_y){
	
	// Must be >= 0, xmin <= px <= xmax
	//               ymin <= py <= ymax
	*cell_x = (int)((px - obstacle_grid.xmin)/obstacle_grid.dx);	// Like floor
	*cell_y = (int)((py - obstacle_grid.ymin)/obstacle_grid.dy);	
	
	*cell_x = (*cell_x >= 0) ? *cell_x : 0;
	*cell_y = (*cell_y >= 0) ? *cell_y : 0;		
}

void fill_cell(float px, float py){
	
	int cx, cy;
	obtain_cell_xy(px,py,&cx,&cy);
	obstacle_grid.world[cy][cx] = 1; // Row, col
	
}

