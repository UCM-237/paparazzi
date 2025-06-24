
#include "./rover_obstacles.h"
#include "math/pprz_geodetic_float.h"

#include "modules/datalink/telemetry.h"
#include "state.h"


PRINT_CONFIG_VAR(N_ROW_GRID)
PRINT_CONFIG_VAR(N_COL_GRID)

// Mapa de Probabilidades
#define L_FREE    -10  // celda observada libre (log-odds negativo)
#define L_OCC     20   // celda observada ocupada (log-odds positivo)
#define L_MIN    -127  // saturación mínima
#define L_MAX     127  // saturación máxima
#define L0         0   // valor inicial (desconocido)
#define L_T				100  // Threeshold para considerar una celda ocupada/libres

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
static void send_grid_init(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GRID_INIT(trans, dev, AC_ID,
  				&obstacle_grid.dx,
  				&obstacle_grid.dy,
  				&obstacle_grid.xmin,
  				&obstacle_grid.xmax,
  				&obstacle_grid.ymin,
  				&obstacle_grid.ymax);
}
#endif

void init_grid(uint8_t pa, uint8_t pb){

	int i,j;
	for(i = 0; i < N_ROW_GRID; i++){
		for(j = 0; j < N_COL_GRID; j++){
			obstacle_grid.world[i][j] = L0;
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
		register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GRID_INIT, send_grid_init);
	#endif	
	obstacle_grid.is_ready = 1;

}

// Same as init_grid but with 4 waypoints (you can give the 4 wp in any order).
void init_grid_4(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4) {
  float xmin = fminf(fminf(WaypointX(wp1), WaypointX(wp2)), fminf(WaypointX(wp3), WaypointX(wp4)));
  float xmax = fmaxf(fmaxf(WaypointX(wp1), WaypointX(wp2)), fmaxf(WaypointX(wp3), WaypointX(wp4)));
  float ymin = fminf(fminf(WaypointY(wp1), WaypointY(wp2)), fminf(WaypointY(wp3), WaypointY(wp4)));
  float ymax = fmaxf(fmaxf(WaypointY(wp1), WaypointY(wp2)), fmaxf(WaypointY(wp3), WaypointY(wp4)));

  obstacle_grid.xmin = xmin;
  obstacle_grid.xmax = xmax;
  obstacle_grid.ymin = ymin;
  obstacle_grid.ymax = ymax;

  obstacle_grid.dx = (xmax - xmin) / ((float)N_COL_GRID);
  obstacle_grid.dy = (ymax - ymin) / ((float)N_ROW_GRID);

  obstacle_grid.now_row = 0;
  obstacle_grid.is_ready = 1;

  memset(obstacle_grid.world, 0, sizeof(obstacle_grid.world));

	#if PERIODIC_TELEMETRY
  	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OBSTACLE_GRID, send_obstacle_grid);
		register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GRID_INIT, send_grid_init);
	#endif

	// Manda el mensaje una vez para la estacion de tierra
	DOWNLINK_SEND_GRID_INIT(
		DefaultChannel,
		DefaultDevice,
		&obstacle_grid.dx,
		&obstacle_grid.dy,
		&obstacle_grid.xmin,
		&obstacle_grid.xmax,
		&obstacle_grid.ymin,
		&obstacle_grid.ymax
	);
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

void fill_bayesian_cell(float px, float py){

		// DOWNLINK_SEND_GRID_CHANGES(DefaultChannel, DefaultDevice, 0, 0, 120);
		if (!obstacle_grid.is_ready) return;

    int cx, cy;
    obtain_cell_xy(px, py, &cx, &cy);

		// Obtiene la celda en la que esta el rover
		int rx, ry;
		struct EnuCoor_f rover_pos;
		rover_pos = *stateGetPositionEnu_f();
		obtain_cell_xy(rover_pos.x, rover_pos.y, &rx, &ry);

		update_line_bayes(rx, ry, cx, cy);   // Libre entre rover y obstáculo
		update_cell_bayes(cx, cy, true);     // Ocupado en el punto final
}

// Bresenham algorithm
void update_line_bayes(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (1) {
        if (x0 == x1 && y0 == y1) break;
        update_cell_bayes(x0, y0, false); // Libre
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void update_cell_bayes(int x, int y, bool is_occupied) {
    int8_t *cell = &obstacle_grid.world[y][x];
    int delta = is_occupied ? L_OCC : L_FREE;
    int updated = *cell + delta;
    if (updated > L_MAX) updated = L_MAX;
    if (updated < L_MIN) updated = L_MIN;

		// Decide si enviar esta celda (0 unknown, 1 ocupado, 2 libre)
		int8_t old_value = *cell;
    uint8_t old_state = (old_value > L_T) ? 1 : (old_value < -L_T) ? 2 : 0;
    uint8_t new_state = (updated > L_T) ? 1 : (updated < -L_T) ? 2 : 0;

		if(old_state != new_state){
			DOWNLINK_SEND_GRID_CHANGES(DefaultChannel, DefaultDevice, &y, &x, &updated);
		}

		*cell = (int8_t)updated;
}




