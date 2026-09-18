#include "hppm_control.h"
#include "state.h" 
#include "modules/guidance/gvf/gvf.h" 
#include "modules/nav/waypoints.h"
#include "generated/flight_plan.h"
#include "firmwares/rover/guidance/rover_guidance.h" 
#include "modules/datalink/datalink.h"
#include "generated/modules.h"
#include "modules/datalink/downlink.h"
#include <math.h> 

EntornoNavegacion hppm_entorno;
EstadoRobot hppm_robot;
AdministradorMision hppm_mision; 
TrayectoriaSegura hppm_buffer_trayectoria; 

double historial_ruta_buffer[VENTANA_ESTANCAMIENTO + 1][3];
double ultimos_centros[3][3];

void hppm_init(void) {
    memset(&hppm_entorno, 0, sizeof(EntornoNavegacion));
    memset(&hppm_robot, 0, sizeof(EstadoRobot));
    memset(&hppm_buffer_trayectoria, 0, sizeof(TrayectoriaSegura));
    
    hppm_entorno.num_obstaculos = 1;
    hppm_entorno.matriz_obstaculos[0][0] = 9999.0;
    hppm_entorno.matriz_obstaculos[0][1] = 9999.0;
    hppm_entorno.matriz_obstaculos[0][2] = 0.1;
    hppm_entorno.matriz_obstaculos[0][3] = 0.001;
}

static void invertir_matriz_jacobiana(double J[3][3], double J_inv[3][3]) {
    double cofactor_00 = J[1][1]*J[2][2] - J[1][2]*J[2][1];
    double cofactor_01 = J[1][0]*J[2][2] - J[1][2]*J[2][0];
    double cofactor_02 = J[1][0]*J[2][1] - J[1][1]*J[2][0];

    double determinante = J[0][0]*cofactor_00 - J[0][1]*cofactor_01 + J[0][2]*cofactor_02;
    double inv_determinante = (fabs(determinante) < LIMITE_CERO_MATRIZ) ? 0.0 : 1.0 / determinante;

    J_inv[0][0] =  cofactor_00 * inv_determinante;
    J_inv[1][0] = -cofactor_01 * inv_determinante;
    J_inv[2][0] =  cofactor_02 * inv_determinante;
    J_inv[0][1] = -(J[0][1]*J[2][2] - J[0][2]*J[2][1]) * inv_determinante;
    J_inv[1][1] =  (J[0][0]*J[2][2] - J[0][2]*J[2][0]) * inv_determinante;
    J_inv[2][1] = -(J[0][0]*J[2][1] - J[0][1]*J[2][0]) * inv_determinante;
    J_inv[0][2] =  (J[0][1]*J[1][2] - J[0][2]*J[1][1]) * inv_determinante;
    J_inv[1][2] = -(J[0][0]*J[1][2] - J[0][2]*J[1][0]) * inv_determinante;
    J_inv[2][2] =  (J[0][0]*J[1][1] - J[0][1]*J[1][0]) * inv_determinante;
}

static void evaluar_entorno_y_errores(EntornoNavegacion *entorno, EstadoRobot *robot, int calcular_errores_f) {
    double sumatoria_repulsion_x = 0.0, sumatoria_repulsion_y = 0.0, sumatoria_energia_W = 0.0;
    
    for(int k = 0; k < entorno->num_obstaculos; k++) {
        double delta_x = robot->x - entorno->matriz_obstaculos[k][0];
        double delta_y = robot->y - entorno->matriz_obstaculos[k][1];
        
        double distancia_cuadrada = (delta_x * delta_x) + (delta_y * delta_y);
    
        double radio_cuadrado = entorno->matriz_obstaculos[k][2]; 
        if (radio_cuadrado < 0.01) radio_cuadrado = 0.01; 
        
        double peso_neto = entorno->matriz_obstaculos[k][3]; 
        
        double energia_gaussiana = peso_neto * exp(-distancia_cuadrada / radio_cuadrado);
        
        double factor_fuerza = -energia_gaussiana / radio_cuadrado;

        sumatoria_repulsion_x += (factor_fuerza * (2.0 * delta_x)); 
        sumatoria_repulsion_y += (factor_fuerza * (2.0 * delta_y));
        
        if(calcular_errores_f) {
            sumatoria_energia_W += energia_gaussiana; 
        }
    }

    robot->jacobiano[0][0] = -entorno->pendiente_inicial; 
    robot->jacobiano[0][1] = -1.0; 
    robot->jacobiano[0][2] = -entorno->inicio_y - entorno->pendiente_inicial * entorno->inicio_x + entorno->pendiente_inicial * entorno->meta_x + entorno->meta_y;

    robot->jacobiano[1][0] = -entorno->pendiente_final + sumatoria_repulsion_x; 
    robot->jacobiano[1][1] = -1.0 + sumatoria_repulsion_y;
    robot->jacobiano[1][2] = -entorno->inicio_y - entorno->pendiente_final*entorno->inicio_x + entorno->meta_y + entorno->pendiente_final*entorno->meta_x + robot->energia_acumulada_inicio - robot->energia_acumulada_meta;
    
    robot->jacobiano[2][0] = 2.0 * (robot->x - robot->centro_x); 
    robot->jacobiano[2][1] = 2.0 * (robot->y - robot->centro_y);
    robot->jacobiano[2][2] = 2.0 * (robot->L - robot->centro_L);

    if (calcular_errores_f) {
        double constante_H1 = -entorno->inicio_y - entorno->pendiente_inicial*entorno->inicio_x + entorno->pendiente_inicial*entorno->meta_x + entorno->meta_y;
        double constante_H2 = -entorno->inicio_y - entorno->pendiente_final*entorno->inicio_x + (entorno->meta_y + entorno->pendiente_final*entorno->meta_x) + sumatoria_energia_W - robot->energia_acumulada_meta;
        
        robot->error_funcion[0][0] = -robot->y - entorno->pendiente_inicial*robot->x + entorno->pendiente_inicial*entorno->meta_x + entorno->meta_y - (1.0 - robot->L)*constante_H1;
        robot->error_funcion[1][0] = (-robot->y - entorno->pendiente_final*robot->x + (entorno->meta_y + entorno->pendiente_final*entorno->meta_x) + sumatoria_energia_W - robot->energia_acumulada_meta) - (1.0 - robot->L)*constante_H2;
        
        double distancia_centro_x = robot->x - robot->centro_x;
        double distancia_centro_y = robot->y - robot->centro_y;
        double distancia_centro_L = robot->L - robot->centro_L;
        robot->error_funcion[2][0] = (distancia_centro_x * distancia_centro_x) + (distancia_centro_y * distancia_centro_y) + (distancia_centro_L * distancia_centro_L) - (robot->radio_actual * robot->radio_actual);
    }
}

static void hppm_actualizar_energias(void) {
    hppm_robot.energia_acumulada_inicio = 0.0;
    hppm_robot.energia_acumulada_meta = 0.0;
    
    for(int k = 0; k < hppm_entorno.num_obstaculos; k++) {
        double radio_cuadrado = hppm_entorno.matriz_obstaculos[k][2];
        if (radio_cuadrado < 0.01) radio_cuadrado = 0.01;
        double peso_neto = hppm_entorno.matriz_obstaculos[k][3];

        double delta_x_ini = hppm_entorno.inicio_x - hppm_entorno.matriz_obstaculos[k][0];
        double delta_y_ini = hppm_entorno.inicio_y - hppm_entorno.matriz_obstaculos[k][1];
        double dist_cuadrada_ini = (delta_x_ini * delta_x_ini) + (delta_y_ini * delta_y_ini);
        
        hppm_robot.energia_acumulada_inicio += peso_neto * exp(-dist_cuadrada_ini / radio_cuadrado);

        double delta_x_meta = hppm_entorno.meta_x - hppm_entorno.matriz_obstaculos[k][0];
        double delta_y_meta = hppm_entorno.meta_y - hppm_entorno.matriz_obstaculos[k][1];
        double dist_cuadrada_meta = (delta_x_meta * delta_x_meta) + (delta_y_meta * delta_y_meta);
        
        hppm_robot.energia_acumulada_meta += peso_neto * exp(-dist_cuadrada_meta / radio_cuadrado);
    }
}

static void hppm_periodic_step(void) {
    hppm_actualizar_energias();

    if (hppm_robot.contador_pasos <= 1) {
        hppm_robot.L = 0.0;
        hppm_robot.L_anterior = 0.0;
    } else {
        hppm_robot.x_anterior = hppm_robot.paso_corregido[0][0];
        hppm_robot.y_anterior = hppm_robot.paso_corregido[1][0];
    }

    hppm_robot.centro_x = hppm_robot.x_anterior;
    hppm_robot.centro_y = hppm_robot.y_anterior;
    hppm_robot.centro_L = hppm_robot.L_anterior;

    evaluar_entorno_y_errores(&hppm_entorno, &hppm_robot, FALSO); 
    
    double vtx = hppm_robot.jacobiano[0][2]*hppm_robot.jacobiano[1][1] - hppm_robot.jacobiano[1][2]*hppm_robot.jacobiano[0][1];
    double vty = hppm_robot.jacobiano[1][2]*hppm_robot.jacobiano[0][0] - hppm_robot.jacobiano[0][2]*hppm_robot.jacobiano[1][0];
    double vtL = -(hppm_robot.jacobiano[0][0]*hppm_robot.jacobiano[1][1] - hppm_robot.jacobiano[0][1]*hppm_robot.jacobiano[1][0]);
    
    double norma = sqrt((vtx*vtx) + (vty*vty) + (vtL*vtL));
    if (norma < LIMITE_CERO_NORMA) norma = LIMITE_CERO_NORMA;
    
    hppm_robot.direccion_x = vtx / norma; 
    hppm_robot.direccion_y = vty / norma; 
    hppm_robot.direccion_L = vtL / norma;

    int signo = (hppm_entorno.pendiente_inicial > hppm_entorno.pendiente_final) ? -1 : 1; 

    hppm_robot.paso_predicho[0][0] = hppm_robot.x_anterior + signo*(hppm_robot.radio_actual * hppm_robot.direccion_x);
    hppm_robot.paso_predicho[1][0] = hppm_robot.y_anterior + signo*(hppm_robot.radio_actual * hppm_robot.direccion_y);
    hppm_robot.paso_predicho[2][0] = hppm_robot.L_anterior + signo*(hppm_robot.radio_actual * hppm_robot.direccion_L);
    hppm_robot.determinante_anterior = vtL;

    hppm_robot.x = hppm_robot.paso_predicho[0][0]; 
    hppm_robot.y = hppm_robot.paso_predicho[1][0]; 
    hppm_robot.L = hppm_robot.paso_predicho[2][0];
    
    memcpy(&ultimos_centros[0][0], &ultimos_centros[1][0], 3 * sizeof(double));
    memcpy(&ultimos_centros[1][0], &ultimos_centros[2][0], 3 * sizeof(double));
    ultimos_centros[2][0] = hppm_robot.centro_x; 
    ultimos_centros[2][1] = hppm_robot.centro_y; 
    ultimos_centros[2][2] = hppm_robot.centro_L;

    int iteraciones = 0; 
    double error_cuadrado = 1.0;
    double res_x = hppm_robot.x_anterior, res_y = hppm_robot.y_anterior, res_L = hppm_robot.L_anterior;
    
    while (iteraciones < MAX_ITERACIONES_NEWTON) {
        evaluar_entorno_y_errores(&hppm_entorno, &hppm_robot, VERDADERO); 
        error_cuadrado = (hppm_robot.error_funcion[0][0]*hppm_robot.error_funcion[0][0]) + 
                         (hppm_robot.error_funcion[1][0]*hppm_robot.error_funcion[1][0]) + 
                         (hppm_robot.error_funcion[2][0]*hppm_robot.error_funcion[2][0]);
                         
        if (error_cuadrado <= TOLERANCIA_ERROR_F_CUADRADO) break;

        hppm_robot.jacobiano[0][0] += FACTOR_AMORTIGUACION;
        hppm_robot.jacobiano[1][1] += FACTOR_AMORTIGUACION;
        hppm_robot.jacobiano[2][2] += FACTOR_AMORTIGUACION;

        invertir_matriz_jacobiana(hppm_robot.jacobiano, hppm_robot.jacobiano_inverso);
        
        double aj_x = (hppm_robot.jacobiano_inverso[0][0]*hppm_robot.error_funcion[0][0] + hppm_robot.jacobiano_inverso[0][1]*hppm_robot.error_funcion[1][0] + hppm_robot.jacobiano_inverso[0][2]*hppm_robot.error_funcion[2][0]);
        double aj_y = (hppm_robot.jacobiano_inverso[1][0]*hppm_robot.error_funcion[0][0] + hppm_robot.jacobiano_inverso[1][1]*hppm_robot.error_funcion[1][0] + hppm_robot.jacobiano_inverso[1][2]*hppm_robot.error_funcion[2][0]);
        double aj_L = (hppm_robot.jacobiano_inverso[2][0]*hppm_robot.error_funcion[0][0] + hppm_robot.jacobiano_inverso[2][1]*hppm_robot.error_funcion[1][0] + hppm_robot.jacobiano_inverso[2][2]*hppm_robot.error_funcion[2][0]);
        
        hppm_robot.x -= aj_x; hppm_robot.y -= aj_y; hppm_robot.L -= aj_L;
        iteraciones++;
    }

    if (error_cuadrado > TOLERANCIA_ERROR_F_CUADRADO) {
        hppm_robot.radio_actual /= 2.0;
        
        if (hppm_robot.radio_actual < 0.005) {
            hppm_robot.L += 0.02; 
            hppm_robot.radio_actual = hppm_entorno.radio_esfera_paso;
            
            hppm_robot.x = hppm_robot.paso_predicho[0][0];
            hppm_robot.y = hppm_robot.paso_predicho[1][0];
            
            hppm_robot.paso_corregido[0][0] = hppm_robot.x;
            hppm_robot.paso_corregido[1][0] = hppm_robot.y;
            hppm_robot.x_anterior = hppm_robot.x;
            hppm_robot.y_anterior = hppm_robot.y;
            hppm_robot.L_anterior = hppm_robot.L;
            hppm_robot.contador_pasos++;
        } else {
            hppm_robot.x_anterior = res_x; hppm_robot.y_anterior = res_y; hppm_robot.L_anterior = res_L;
        }
        return; 
    }

    hppm_robot.paso_corregido[0][0] = hppm_robot.x;
    hppm_robot.paso_corregido[1][0] = hppm_robot.y;
    hppm_robot.L_anterior = hppm_robot.L; 
    hppm_robot.contador_pasos++;
}

void hppm_calcular_ruta_completa(void) {
    hppm_buffer_trayectoria.cantidad_puntos = 0;
    hppm_buffer_trayectoria.indice_actual = 0;
    hppm_buffer_trayectoria.recalculando = FALSO;
    
    int max_intentos = MAX_PUNTOS_TRAYECTORIA * 5; 
    
    while (hppm_robot.L < 1.0 && max_intentos > 0) {
        hppm_periodic_step();
        
        if (isnan(hppm_robot.paso_corregido[0][0]) || isnan(hppm_robot.paso_corregido[1][0])) break; 
        
        double dist_prev = 999.0;
        if (hppm_buffer_trayectoria.cantidad_puntos > 0) {
            int last_idx = hppm_buffer_trayectoria.cantidad_puntos - 1;
            double dx = hppm_robot.paso_corregido[0][0] - hppm_buffer_trayectoria.x[last_idx];
            double dy = hppm_robot.paso_corregido[1][0] - hppm_buffer_trayectoria.y[last_idx];
            dist_prev = sqrt(dx*dx + dy*dy);
        }
        
        if (dist_prev > 0.001) { 
            int idx = hppm_buffer_trayectoria.cantidad_puntos;
            hppm_buffer_trayectoria.x[idx] = hppm_robot.paso_corregido[0][0];
            hppm_buffer_trayectoria.y[idx] = hppm_robot.paso_corregido[1][0];
            hppm_buffer_trayectoria.L_anterior = hppm_robot.L;
            hppm_buffer_trayectoria.cantidad_puntos++;
            if (hppm_buffer_trayectoria.cantidad_puntos >= MAX_PUNTOS_TRAYECTORIA) break;
        }
        max_intentos--;
    }
}

void hppm_clear_obstacles(void) { 
    hppm_entorno.num_obstaculos = 0; 
    waypoint_move_xy_i(WP_OBS0, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_OBS1, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_OBS2, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_OBS3, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_OBS4, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
}

void hppm_ruta_limpiar(void) {
    hppm_mision.total_puntos = 0;
    hppm_mision.punto_actual = 0;
    
    hppm_clear_obstacles(); 
    waypoint_move_xy_i(WP_L0, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_L1, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_L2, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_L3, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_L4, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_L5, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_L6, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_L7, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
    waypoint_move_xy_i(WP_L8, (int32_t)(999.0f * 256.0f), (int32_t)(999.0f * 256.0f));
} 

void hppm_mover_wp(uint8_t wp_id, float x, float y) {
    waypoint_move_xy_i(wp_id, (int32_t)(x * 256.0f), (int32_t)(y * 256.0f));
}

void hppm_ruta_add_wp(uint8_t wp_id) {
    if (hppm_mision.total_puntos < 20) {
        hppm_mision.waypoints[hppm_mision.total_puntos] = wp_id;
        hppm_mision.total_puntos++;
    }
}

void hppm_ruta_iniciar(void) {
    if (hppm_mision.total_puntos > 0) {
        hppm_mision.punto_actual = 0;
        hppm_route_start(hppm_mision.waypoints[hppm_mision.punto_actual]);
    }
}

// =========================================================================

void hppm_route_start(uint8_t wp_meta) {
    memset(&hppm_robot, 0, sizeof(EstadoRobot));
    
    hppm_entorno.radio_esfera_paso = 0.15; 
    
    struct EnuCoor_f *pos_actual = stateGetPositionEnu_f();
    hppm_entorno.inicio_x = pos_actual->x; hppm_entorno.inicio_y = pos_actual->y;
    hppm_entorno.meta_x = WaypointX(wp_meta); hppm_entorno.meta_y = WaypointY(wp_meta);

    double dx = hppm_entorno.meta_x - hppm_entorno.inicio_x;
    double dy = hppm_entorno.meta_y - hppm_entorno.inicio_y;
    if (fabs(dx) < 1e-6) dx = 1e-6;
    
    double angulo = atan2(dy, dx);
    hppm_entorno.pendiente_inicial = tan(angulo + (M_PI / 4.0));
    hppm_entorno.pendiente_final = tan(angulo - (M_PI / 4.0));
    
    hppm_robot.x = hppm_entorno.inicio_x; hppm_robot.y = hppm_entorno.inicio_y;
    hppm_robot.x_anterior = hppm_entorno.inicio_x; hppm_robot.y_anterior = hppm_entorno.inicio_y;
    hppm_robot.L = 0.0; hppm_robot.L_anterior = 0.0;
    hppm_robot.contador_pasos = 1;
    hppm_robot.radio_actual = hppm_entorno.radio_esfera_paso;

    hppm_robot.centro_x = hppm_robot.x_anterior; 
    hppm_robot.centro_y = hppm_robot.y_anterior; 
    hppm_robot.centro_L = hppm_robot.L_anterior; 
    
    hppm_buffer_trayectoria.L_anterior = -1.0;
    hppm_buffer_trayectoria.recalculando = VERDADERO; 
}

bool nav_hppm_run(void) {
    struct EnuCoor_f *pos_gps = stateGetPositionEnu_f();
    
    if (hppm_buffer_trayectoria.recalculando) {
        // printf(">> GENERANDO RUTA MATEMÁTICA COMPLETA DESDE L0 A L1 <<\n");

        hppm_robot.x = hppm_entorno.inicio_x;
        hppm_robot.y = hppm_entorno.inicio_y;
        hppm_robot.L = 0.0;
        
        hppm_robot.x_anterior = hppm_entorno.inicio_x;
        hppm_robot.y_anterior = hppm_entorno.inicio_y;
        hppm_robot.L_anterior = 0.0;
        
        hppm_robot.centro_x = hppm_entorno.inicio_x;
        hppm_robot.centro_y = hppm_entorno.inicio_y;
        hppm_robot.centro_L = 0.0;
        
        hppm_robot.contador_pasos = 1;
        hppm_robot.radio_actual = hppm_entorno.radio_esfera_paso;
        
        hppm_buffer_trayectoria.L_anterior = -1.0;
        
        hppm_calcular_ruta_completa(); 
        
        hppm_buffer_trayectoria.indice_actual = 0;
        hppm_buffer_trayectoria.recalculando = FALSO;
        
        // printf(">> BÚFER GENERADO EXITOSAMENTE: %d PUNTOS <<\n", hppm_buffer_trayectoria.cantidad_puntos);
    }

    int closest_idx = 0;
    float min_dist = 999999.0f;
    for (int i = 0; i < hppm_buffer_trayectoria.cantidad_puntos; i++) {
        float dx_c = (float)hppm_buffer_trayectoria.x[i] - pos_gps->x;
        float dy_c = (float)hppm_buffer_trayectoria.y[i] - pos_gps->y;
        float d = (dx_c * dx_c) + (dy_c * dy_c);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }
    
    int anchor_idx = closest_idx;
    int target_idx = closest_idx + 25; // Modificar distancia aqui ////////////////////////////////////////////////////////////////
    
    if (target_idx >= hppm_buffer_trayectoria.cantidad_puntos) {
        target_idx = hppm_buffer_trayectoria.cantidad_puntos - 1;
    }

    float p1_x = (float)hppm_buffer_trayectoria.x[anchor_idx];
    float p1_y = (float)hppm_buffer_trayectoria.y[anchor_idx];
    float p2_x = (float)hppm_buffer_trayectoria.x[target_idx];
    float p2_y = (float)hppm_buffer_trayectoria.y[target_idx];
    
    float d_sq = (p2_x - p1_x)*(p2_x - p1_x) + (p2_y - p1_y)*(p2_y - p1_y);
    
    if (d_sq < 0.01f || isnan(p2_x) || hppm_buffer_trayectoria.cantidad_puntos == 0) {
        p1_x = (float)hppm_entorno.inicio_x;
        p1_y = (float)hppm_entorno.inicio_y;
        p2_x = WaypointX(hppm_mision.waypoints[hppm_mision.punto_actual]);
        p2_y = WaypointY(hppm_mision.waypoints[hppm_mision.punto_actual]);
    }

    if (isnan(p1_x) || isnan(p1_y) || isnan(p2_x) || isnan(p2_y)) {
        p1_x = 0.0f; p1_y = 0.0f; p2_x = 1.0f; p2_y = 0.0f;
    }

    gvf_segment_XY1_XY2(p1_x, p1_y, p2_x, p2_y);
    
    float hppm_telemetry_array[2] = { (float)hppm_robot.L, (float)hppm_buffer_trayectoria.cantidad_puntos };
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 2, hppm_telemetry_array);

    float dx_meta = hppm_entorno.meta_x - pos_gps->x;
    float dy_meta = hppm_entorno.meta_y - pos_gps->y;
    float dist_fisica_al_punto = sqrt((dx_meta * dx_meta) + (dy_meta * dy_meta));

    float VELOCIDAD_MINIMA = 0.5f; 
    static float gcs_max_memoria = 2.0f; 
    static bool rover_frenando = false;

    if (dist_fisica_al_punto < 3.0f) {
        if (!rover_frenando) {
            gcs_max_memoria = guidance_control.cmd.max_speed;
            rover_frenando = true;
        }
        guidance_control.cmd.max_speed = VELOCIDAD_MINIMA + ((gcs_max_memoria - VELOCIDAD_MINIMA) * 0.5f);
    } else {
        if (rover_frenando) {
            guidance_control.cmd.max_speed = gcs_max_memoria;
            rover_frenando = false;
        }
    }

    if (dist_fisica_al_punto < 2.0f) { 
        if (rover_frenando) { guidance_control.cmd.max_speed = gcs_max_memoria; rover_frenando = false; }
        hppm_mision.punto_actual++; 
        if (hppm_mision.punto_actual < hppm_mision.total_puntos) {
            hppm_route_start(hppm_mision.waypoints[hppm_mision.punto_actual]);
            return true; 
        } else { return false; }
    }
    
    return true; 
}

void hppm_update_obstacle(uint8_t id, float x, float y, float radio, float peso) {
    if (id < MAX_OBSTACULOS) { 
        hppm_entorno.matriz_obstaculos[id][0] = x;
        hppm_entorno.matriz_obstaculos[id][1] = y;
        
        hppm_entorno.matriz_obstaculos[id][2] = radio * radio; 
        hppm_entorno.matriz_obstaculos[id][3] = peso;
        
        if (id >= hppm_entorno.num_obstaculos) hppm_entorno.num_obstaculos = id + 1;
        
        if (id == 0) {
            waypoint_move_xy_i(WP_OBS0, (int32_t)(x * 256.0f), (int32_t)(y * 256.0f));
        } else if (id == 1) {
            waypoint_move_xy_i(WP_OBS1, (int32_t)(x * 256.0f), (int32_t)(y * 256.0f));
        } else if (id == 2) {
            waypoint_move_xy_i(WP_OBS2, (int32_t)(x * 256.0f), (int32_t)(y * 256.0f));
        } else if (id == 3) {
            waypoint_move_xy_i(WP_OBS3, (int32_t)(x * 256.0f), (int32_t)(y * 256.0f));
        } else if (id == 4) {
            waypoint_move_xy_i(WP_OBS4, (int32_t)(x * 256.0f), (int32_t)(y * 256.0f));
        }
        
        hppm_buffer_trayectoria.recalculando = VERDADERO;
    }
}

void hppm_parse_obstacle(uint8_t *buf) {
    // Extraemos para quién iba dirigido este mensaje desde el bus Ivy
    uint8_t id_destino = DL_HPPM_OBSTACLE_ac_id(buf);
    
    // FILTRO DE JERARQUÍA: AC_ID es la macro global de Paparazzi para este rover.
    // Si el mensaje no es para mí, lo descarto silenciosamente.
    if (id_destino != AC_ID) {
        return; 
    }

    // Si llegó hasta aquí, el robot sabe que él debe esquivar
    // printf("\n>> ALERTA (Rover %d): OBSTACULO RECIBIDO EN X:%.1f Y:%.1f <<\n", 
    //        AC_ID, DL_HPPM_OBSTACLE_x(buf), DL_HPPM_OBSTACLE_y(buf));
           
    hppm_update_obstacle(
        DL_HPPM_OBSTACLE_obs_id(buf), 
        DL_HPPM_OBSTACLE_x(buf), 
        DL_HPPM_OBSTACLE_y(buf), 
        DL_HPPM_OBSTACLE_radio(buf), 
        DL_HPPM_OBSTACLE_peso(buf)
    );
} 