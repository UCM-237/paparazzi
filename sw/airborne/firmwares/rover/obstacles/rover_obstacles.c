// Depends of tfmini lidar and INS SLAM EKF (could be replaced by other lidar and ins slam)

#include "./rover_obstacles.h"
#include "modules/lidar/tfmini.h"
#include "modules/ins/ins_slam_ekf.h"
#include "math/pprz_geodetic_float.h"

#include "modules/datalink/telemetry.h"
#include "state.h"


PRINT_CONFIG_VAR(N_ROW_GRID)
PRINT_CONFIG_VAR(N_COL_GRID)

// Mapa de Probabilidades
#define P_FREE    0.4   // celda observada libre (log-odds negativo)
#define P_OCC     0.7   // celda observada ocupada (log-odds positivo)
#define L_MIN    -127   // saturación mínima
#define L_MAX     127   // saturación máxima
#define L0         0    // valor inicial (desconocido)
#define P_T				0.95   // Threeshold para considerar una celda ocupada/libres

#define SCALE    20.0f   // escalado de log-odds float a int8_t --> Con 20, max prob 0.995

#define DECAY_INTERVAL 5000	// ms para que decaiga la probabilidad de los obstaculos
#define DECAY 5	// Decaimiento por segundo
static uint32_t last_s = 0;

// Variables globales (para evitar estar calculando todo el rato log)
float POCC = 0;
float PFREE = 0;
float PT = 0;

int8_t LT, LOCC, LFREE;

world_grid obstacle_grid;
uint8_t grid_block_size = GRID_BLOCK_SIZE;


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
  				&obstacle_grid.ymax,
					&obstacle_grid.map.LT
					);
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

// Same as init_grid but with 4 waypoints (you can give the 4 wp in any order, 
// but you need to have them in the correct order in the flightplan for painting the border in the GCS).
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

	obstacle_grid.map.threshold = (float) P_T; 
	obstacle_grid.map.occ = (float) P_OCC; 
	obstacle_grid.map.free = (float) P_FREE; 
	obstacle_grid.map.decay = (uint8_t) DECAY;

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
		&obstacle_grid.ymax,
		&obstacle_grid.map.LT
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

		if (!obstacle_grid.is_ready) return;

    int cx, cy;
		obtain_cell_xy(px, py, &cx, &cy);

		// Obtiene la celda en la que esta el rover
		int rx, ry;
		struct EnuCoor_f rover_pos;
		rover_pos = *stateGetPositionEnu_f();
		obtain_cell_xy(rover_pos.x, rover_pos.y, &rx, &ry);

		update_line_bayes(rx, ry, cx, cy);   // Libre entre rover y obstáculo
		compute_cell_bayes(cx, cy, true);     // Ocupado en el punto final
}

// Rellena las celda libres cuando no hay medida del lidar
void fill_free_cells() {
		if (!obstacle_grid.is_ready) return;

		// Obtiene la medida actual del lidar
		float lidar = tfmini.distance;
		float angle = tf_servo.ang;

		// Obtiene la celda en la que esta el rover
		int rx, ry;
		struct EnuCoor_f rover_pos;
		rover_pos = *stateGetPositionEnu_f();
		obtain_cell_xy(rover_pos.x, rover_pos.y, &rx, &ry);
		if (rx < 0 || rx >= N_COL_GRID || ry < 0 || ry >= N_ROW_GRID) {
			return;
		}

		// Si no hay medida del lidar, se marcan las celdas como libres
		if(lidar == 0.0f){
			// Calcula el obstaculo ficticio
			int cx, cy;
			float theta = stateGetNedToBodyEulers_f()->psi;
			float corrected_angle = M_PI / 2 - angle*M_PI/180 - theta;

			float px = rover_pos.x + (ins_slam.max_distance * cosf(corrected_angle));
			float py = rover_pos.y + (ins_slam.max_distance * sinf(corrected_angle));

			obtain_cell_xy(px, py, &cx, &cy);

			update_line_bayes(rx, ry, cx, cy);   // Libre entre rover y obstáculo
			return;
		}
		else{
			compute_cell_bayes(rx, ry, false);    // Libre en el punto final
		}
}


// Funcion para que el mapa olvide los obstaculos
void decay_map()
{   
	uint32_t now_s = get_sys_time_msec();

	if (now_s > (last_s + DECAY_INTERVAL)) { 
		last_s = now_s;
		for (int y = 0; y < N_ROW_GRID; y++) {
			for (int x = 0; x < N_COL_GRID; x++) {
				int8_t *cell = &obstacle_grid.world[y][x];
				if (*cell == 0){
					continue;
				}
				else if (*cell > 0) {
					int updated = (*cell > obstacle_grid.map.decay) ? *cell - obstacle_grid.map.decay : 0;
					update_cell(x, y, updated);
				} else if (*cell < 0) {
					int updated = (*cell < - obstacle_grid.map.decay) ? *cell + obstacle_grid.map.decay : 0;
					update_cell(x, y, updated);
				}
			}
  	}
	}
}



/*******************************************************************************
 *                                                                             *
 *  Aux functions                                                           *
 *                                                                             *
 ******************************************************************************/


// Bresenham algorithm
void update_line_bayes(int x0, int y0, int x1, int y1) {
	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = dx + dy;

	while (1) {
			if (x0 == x1 && y0 == y1) break;
			compute_cell_bayes(x0, y0, false); // Libre
			int e2 = 2 * err;
			if (e2 >= dy) { err += dy; x0 += sx; }
			if (e2 <= dx) { err += dx; y0 += sy; }
	}
}


// Decide si enviar esta celda (0 unknown, 1 ocupado, 2 libre)
void update_cell(int x, int y, int new_value){
	
	int8_t *cell = &obstacle_grid.world[y][x];
	int8_t old_value = *cell;
	
	uint8_t old_state = (old_value > LT) ? 1 : (old_value < -LT) ? 2 : 0;
	uint8_t new_state = (new_value > LT) ? 1 : (new_value < -LT) ? 2 : 0;
	obstacle_grid.map.LT = LT; // For the GCS

	*cell = (int8_t)new_value;

	if(old_state != new_state){
		DOWNLINK_SEND_GRID_CHANGES(DefaultChannel, DefaultDevice, &y, &x, &new_value);
		// printf("Cell (%d, %d) updated from %d to %d (delta: %d)\n", x, y, old_value, *cell, delta);
	}
}


void compute_cell_bayes(int x, int y, bool is_occupied) {
		if (x < 0 || x >= N_COL_GRID || y < 0 || y >= N_ROW_GRID) {
			return;
		}
    int8_t *cell = &obstacle_grid.world[y][x];

		// obstacle_grid.map.free = 1 - obstacle_grid.map.occ;
		check_probs(&LOCC, &LFREE, &LT);
		
    int delta = is_occupied ? LOCC : LFREE;
    int updated = *cell + delta;
    if (updated > L_MAX) updated = L_MAX;
    if (updated < L_MIN) updated = L_MIN;

		update_cell(x, y, updated);
}


void check_probs(int8_t *LOCC, int8_t *LFREE, int8_t *LT){

	if ((obstacle_grid.map.occ != POCC) || (obstacle_grid.map.free != PFREE)){
		*LOCC = (int8_t) (SCALE*logf(obstacle_grid.map.occ / (1.0f - obstacle_grid.map.occ)));
		*LFREE = (int8_t) (SCALE*logf(obstacle_grid.map.free / (1.0f - obstacle_grid.map.free)));
		POCC = obstacle_grid.map.occ;
		PFREE = obstacle_grid.map.free;
		printf("LOCC: %d, LFREE: %d\n", *LOCC, *LFREE);
	}

	if (obstacle_grid.map.threshold != PT){
		*LT = (int8_t) (SCALE*logf(obstacle_grid.map.threshold / (1.0f - obstacle_grid.map.threshold)));
		PT = obstacle_grid.map.threshold;
	}
}


/*******************************************************************************
 *                                                                             *
 *  CBF functions                                                           *
 *                                                                             *
 ******************************************************************************/


#ifdef GVF_CBF_H

#include "guidance/gvf/gvf_cbf.h"


void write_cbf_static_obstacle(uint16_t i, float x_utm, float y_utm, uint16_t fake_id)
{
  if (i >= CBF_MAX_NEIGHBORS) return;

  cbf_obs_tables[i].state.x = x_utm;
  cbf_obs_tables[i].state.y = y_utm;
  cbf_obs_tables[i].state.vx = 0.0f;
  cbf_obs_tables[i].state.vy = 0.0f;
  cbf_obs_tables[i].state.speed = 0.0f;
  cbf_obs_tables[i].state.course = 0.0f;
  cbf_obs_tables[i].state.uref = 0.0f;

  cbf_obs_tables[i].ac_id = fake_id;
  cbf_obs_tables[i].available = (uint8_t) 1;
  cbf_obs_tables[i].t_last_msg = get_sys_time_msec();
  cbf_obs_tables[i].omega_safe = 0.0f;

  cbf_telemetry.acs_id[i] = fake_id;
  cbf_telemetry.acs_available[i] = 1;

	cbf_ac_state.d[i] = sqrt(pow(cbf_obs_tables[i].state.x - cbf_ac_state.x, 2) + 
                        pow(cbf_obs_tables[i].state.y - cbf_ac_state.y, 2));

	// printf("Sender %u, Table pos %u",cbf_obs_tables[i].ac_id,i);

	// Hay que añadir al contador para que lo tenga en cuenta ??
	// cbf_control.n_neighborns = cbf_control.n_neighborns + 1;

}

void clean_cbf_static_obstacle(uint16_t i)
{
  if (i >= CBF_MAX_NEIGHBORS) return;

  cbf_obs_tables[i].state.vx = 0.0f;
  cbf_obs_tables[i].state.vy = 0.0f;
  cbf_obs_tables[i].state.speed = 0.0f;
  cbf_obs_tables[i].state.course = 0.0f;
  cbf_obs_tables[i].state.uref = 0.0f;

  cbf_obs_tables[i].available = (uint8_t) 0;
  cbf_obs_tables[i].omega_safe = 0.0f;
  cbf_telemetry.acs_available[i] = 0;

	cbf_ac_state.d[i] = 0;

	// printf("Sender %u, Table pos %u",cbf_obs_tables[i].ac_id,i);

}


void get_occupied_cells(uint8_t BLOCK_SIZE) {

	if (!obstacle_grid.is_ready) return 0;

	struct EnuCoor_f pos = *stateGetPositionEnu_f();
  int cx, cy;
  obtain_cell_xy(pos.x, pos.y, &cx, &cy);

	// Índices de desplazamiento de los 4 bloques
	const int8_t dxs[4] = { -1, 1, -1, 1};
	const int8_t dys[4] = { -1, -1, 1, 1};

	for (int b = 0; b < 4; b++) {

		if ((b+MAX_ROVERS+1) > CBF_MAX_NEIGHBORS) return 0;

    int block_x0 = cx + dxs[b];
    int block_y0 = cy + dys[b];

    bool block_occupied = false;

    // Recorre las 9 celdas del bloque 3x3
    for (int i = 0; i < BLOCK_SIZE && !block_occupied; i++) {
      for (int j = 0; j < BLOCK_SIZE && !block_occupied; j++) {
        int x = cx + i*dxs[b];
        int y = cy + j*dys[b];

        if (x < 0 || x >= N_COL_GRID || y < 0 || y >= N_ROW_GRID) continue;
				if (x == cx && y == cy) continue;

        if (obstacle_grid.world[y][x] > LT) {
					// printf("CELDA (%d, %d) OCUPADA. Bloque %d \n", y, x, b);
          block_occupied = true;
        }
				else{
					// printf("CELDA (%d, %d) LIBRE. Bloque %d \n", y, x, b);
				}
      }
    }

    if (block_occupied) {
			
      struct EnuCoor_f enu;
      enu.x = pos.x + dxs[b]*BLOCK_SIZE/2*obstacle_grid.dx;
      enu.y = pos.y + dys[b]*BLOCK_SIZE/2*obstacle_grid.dy;
      enu.z = 0.0f;

      struct UtmCoor_f utm;
      utm_of_enu_f(&utm, &enu);
			
			// printf("Obstacle in block %d at ENU: (%f, %f)\n", b, enu.x, enu.y);
			write_cbf_static_obstacle(b+MAX_ROVERS+1, utm.north, utm.east, 200+b);
    }
		else{
			clean_cbf_static_obstacle(b+MAX_ROVERS+1);
		}
  }
}



#endif




