#ifndef HPPM_CONTROL_H
#define HPPM_CONTROL_H

#include "std.h"
#include <math.h>
#include <string.h>
#include <stdbool.h> 

#define VERDADERO 1
#define FALSO 0

#define MAX_OBSTACULOS 10
#define VENTANA_ESTANCAMIENTO 15       

#define MAX_PUNTOS_TRAYECTORIA 5000

#define PI 3.14159265358979323846
#define EPSILON_SEGURIDAD 0.001
#define EPSILON_CUADRADO (EPSILON_SEGURIDAD * EPSILON_SEGURIDAD) 
#define LIMITE_CERO_MATRIZ 1e-15
#define LIMITE_CERO_NORMA 1e-12
#define FACTOR_AMORTIGUACION 1e-7 

#define MAX_ITERACIONES_NEWTON 40
#define TOLERANCIA_ERROR_F 1e-3
#define TOLERANCIA_ERROR_F_CUADRADO (TOLERANCIA_ERROR_F * TOLERANCIA_ERROR_F) 

#define UMBRAL_RETROCESO_ANGULAR 0.999
#define UMBRAL_SALIDA_RETROCESO 0.9
#define MIN_DISTANCIA_PROGRESO 0.005   

#define MAX_WAYPOINTS_MISION 20 

#define LIMITAR_ACOS(x) ((x) > 1.0 ? 1.0 : ((x) < -1.0 ? -1.0 : (x)))

typedef struct {
    double radio_esfera_paso;
    double pendiente_inicial;
    double pendiente_final;
    double inicio_x;
    double inicio_y;
    double meta_x;
    double meta_y;
    
    int num_obstaculos;
    double matriz_obstaculos[MAX_OBSTACULOS][4]; 
} EntornoNavegacion;

typedef struct {
    double x, y, L;                         
    double x_anterior, y_anterior, L_anterior; 
    double centro_x, centro_y, centro_L;    
    double radio_actual, radio_anterior;    
    
    double direccion_x, direccion_y, direccion_L; 
    
    double jacobiano[3][3];      
    double jacobiano_inverso[3][3];       
    double error_funcion[3][1];          
    double paso_predicho[3][1];          
    double paso_corregido[3][1];        
    
    double energia_acumulada_meta;                
    double energia_acumulada_inicio;              
    int contador_pasos;                 
    int total_iteraciones_newton;                
    double determinante_anterior;             
} EstadoRobot;

typedef struct {
    double x[MAX_PUNTOS_TRAYECTORIA];
    double y[MAX_PUNTOS_TRAYECTORIA];
    int cantidad_puntos;
    int indice_actual;
    bool recalculando; 
    double L_anterior; 
} TrayectoriaSegura;

typedef struct {
    uint8_t waypoints[MAX_WAYPOINTS_MISION]; 
    int total_puntos;                        
    int punto_actual;                        
} AdministradorMision;

extern EntornoNavegacion hppm_entorno;
extern EstadoRobot hppm_robot;
extern AdministradorMision hppm_mision;
extern TrayectoriaSegura hppm_buffer_trayectoria; 

extern void hppm_init(void);

extern void hppm_ruta_limpiar(void);
extern void hppm_ruta_add_wp(uint8_t wp_id);
extern void hppm_ruta_iniciar(void);
extern void hppm_mover_wp(uint8_t wp_id, float x, float y);

extern void hppm_route_start(uint8_t wp_meta);
extern bool nav_hppm_run(void);

extern void hppm_update_obstacle(uint8_t id, float x, float y, float radio, float peso);
extern void hppm_clear_obstacles(void);
extern void hppm_parse_obstacle(uint8_t *buf);

#endif 