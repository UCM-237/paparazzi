/*
 * Copyright (C) 2025 Lia Garcia Pérez
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include <math.h>
#include "std.h"


#include "gvf.h"
#include "gvf_low_level_control.h"
#include "gvf_cbf.h"
#include "autopilot.h"
#include "../gvf_common.h"
#include "modules/multi/traffic_info.h"
#include "modules/datalink/telemetry.h"

// CBF parameters
struct cbf_parameters cbf_param;

// Robots state

cbf_state_t cbf_ac_state = {0};
cbf_tab_entrie_t cbf_obs_tables[CBF_MAX_NEIGHBORS];
// State

gvf_common_field gvf_c_field;
struct cbf_con cbf_control;
struct cbf_tel cbf_telemetry;




#if PERIODIC_TELEMETRY


//TODO Create the message

static void send_cbf(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t n = 1;
  //uint16_t cbf_nei_ac_ids[CBF_MAX_NEIGHBORS] = CBF_NEI_AC_IDS;
  if (cbf_control.n_neighborns > 0) { // n = 0 doesn't make sense
    n = cbf_control.n_neighborns;
  }
  
  for (int i = 0; i < n; i++) {
    cbf_telemetry.acs_available[i] = cbf_obs_tables[i].available;
  }
  
 pprz_msg_send_CBF(trans, dev, AC_ID, &cbf_ac_state.xi_x,&cbf_ac_state.xi_y,
  				&cbf_ac_state.xicbf_x,&cbf_ac_state.xicbf_y,
  				&cbf_control.n_neighborns,
          &cbf_ac_state.active_conds,
  				&cbf_ac_state.r,&cbf_ac_state.alpha);
}

static void send_cbf_rec(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t n = 1;
  // This is the array of the ac_ids of the neighbors
  uint16_t cbf_nei_ac_ids[CBF_MAX_NEIGHBORS] = CBF_NEI_AC_IDS;
  if (cbf_control.n_neighborns > 0) { // n = 0 doesn't make sense
    n = cbf_control.n_neighborns;
  }
  
  for (int i = 0; i < cbf_control.n_neighborns ; i++) {
    cbf_telemetry.acs_available[i] = cbf_obs_tables[i].available;    
    if (cbf_obs_tables[i].ac_id>0 && cbf_obs_tables[i].ac_id<255){
       for (int j = 0; j < CBF_MAX_NEIGHBORS; j++) {
        if (cbf_nei_ac_ids[j]== cbf_obs_tables[i].ac_id) {
          pprz_msg_send_CBF_REC(trans, dev, AC_ID, &cbf_obs_tables[i].ac_id,&(cbf_obs_tables[i].available),
    				&cbf_obs_tables[i].t_last_msg,
  				&cbf_obs_tables[i].state.x,&cbf_obs_tables[i].state.y,
  				&cbf_obs_tables[i].state.vx,&cbf_obs_tables[i].state.vy);
          break;
  	}
	}
}
}
}
#endif // PERIODIC TELEMETRY
 


void cbf_init(void)
{

  cbf_param.r = 3.0;
  cbf_param.alpha = 0.1;
  cbf_ac_state.r=3.0;
  cbf_ac_state.alpha=0.1;
  cbf_ac_state.nei=0;
  cbf_ac_state.active_conds=(uint8_t)0;
  cbf_ac_state.xicbf_y=0;
  cbf_ac_state.xicbf_x=0;
  cbf_ac_state.xi_y=0;
  cbf_ac_state.xi_x=0;
  cbf_control.n_neighborns= (uint8_t) 0;
  cbf_param.timeout = 2000.0; // valor en milisegundos
 // Initilize the obstacles tables with the ac_id of the neighborns
  uint16_t cbf_nei_ac_ids[CBF_MAX_NEIGHBORS] = CBF_NEI_AC_IDS;
  for (int i = 0; i < CBF_MAX_NEIGHBORS; i++) {
    cbf_obs_tables[i].available = 0; // Initialize as not available
    cbf_obs_tables[i].omega_safe = 0.0;
  }
  // Initialize the obstacles tables with the ac_id of the neighborns
  for (int i = 0; i < CBF_MAX_NEIGHBORS; i++) {
    if (cbf_nei_ac_ids[i] > 0 && cbf_nei_ac_ids[i] < 255) {
      cbf_obs_tables[i].ac_id = cbf_nei_ac_ids[i];
      cbf_telemetry.acs_id[i] = cbf_nei_ac_ids[i]; // for telemetry
      cbf_obs_tables[i].available = 1;
      cbf_control.n_neighborns = cbf_control.n_neighborns + 1;
    } 
    else {
      cbf_obs_tables[i].ac_id = 0;
      cbf_telemetry.acs_id[i] =0;
    }
    // Initialize the state related to the obstacles   
    cbf_obs_tables[i].state.x=0.;
    cbf_obs_tables[i].state.y=0.;
    cbf_obs_tables[i].state.vx=0.;
    cbf_obs_tables[i].state.vy=0.;
    cbf_obs_tables[i].t_last_msg=0;
    // Initialize the state related to telemetry
    cbf_telemetry.acs_timeslost[i] = 0; // for telemetry
    cbf_telemetry.acs_available[i]=0;
    cbf_telemetry.uref[i]=0;
    cbf_telemetry.prel_norm[i]=0;
    cbf_telemetry.px_rel[i]=0;
    cbf_telemetry.py_rel[i]=0;
    cbf_telemetry.vx_rel[i]=0;
    cbf_telemetry.vy_rel[i]=0;
    cbf_telemetry.h_ref[i]=0;
    cbf_telemetry.h_dot_ref[i]=0;
    cbf_telemetry.psi_ref[i]=0;
     }

  cbf_ac_state.nei=cbf_control.n_neighborns;
  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CBF, send_cbf);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CBF_REC, send_cbf_rec);
  #endif // PERIODIC_TELEMETRY 

}

//TODO Create the functions to fill in the table and to send the status to other robots
// Thanks to Jesús Bautista
/*struct UtmCoor_f {
  float north; ///< in meters
  float east; ///< in meters
  float alt; ///< in meters (above WGS84 reference ellipsoid or above MSL)
  uint8_t zone; ///< UTM zone number
};*/

static void cbf_low_level_getState(void)
{
  

    // Verifica si el origen UTM está configurado
    if (state.utm_origin_f.zone == 0) {
        struct UtmCoor_f utm_origin = { .north = 500000.0f, .east = 0.0f, .alt = 0.0f, .zone = 30 };
        stateSetLocalUtmOrigin_f(MODULE_GVF_CBF_ID, &utm_origin);
       // DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen("UTM origin set manually"), "UTM origin set manually");
    }

    // Forzar el cálculo de las coordenadas LLA si no están disponibles
    if (!bit_is_set(state.pos_status, POS_LLA_F)) {
        stateCalcPositionLla_f();
        //DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen("LLA position calculated"), "LLA position calculated");
    }
  // Forzar el cálculo de las coordenadas UTM
    stateCalcPositionUtm_f();
   
    // Verificar si las coordenadas UTM están inicializadas
    
    if (!state.utm_initialized_f) {
        //DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen("UTM not initialized"), "UTM not initialized");
        return;
    }

    // Obtener las coordenadas UTM
    struct UtmCoor_f *utm_pos = stateGetPositionUtm_f();
    if (utm_pos->north == 0.0f && utm_pos->east == 0.0f) {
    //DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen("UTM position is zero"), "UTM position is zero");
    }
    cbf_ac_state.x = utm_pos->north;
    cbf_ac_state.y = utm_pos->east;
    cbf_ac_state.speed = stateGetHorizontalSpeedNorm_f();
    //DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen("UTM OK"), "UTM OK");

  
    // Obtener otros estados
    cbf_ac_state.course = 90 - stateGetNedToBodyEulers_f()->psi; // ENU course
    cbf_ac_state.vx = stateGetSpeedEnu_f()->x;
    cbf_ac_state.vy = stateGetSpeedEnu_f()->y;
}

// Fill the i'th obstacle table with the info contained in the buffer  
static void write_cbf_table(uint16_t i, uint8_t *buf) 
{
  char msg[35];
  if (i<CBF_MAX_NEIGHBORS){
  cbf_obs_tables[i].state.x = DL_CBF_STATE_x_enu(buf);
  cbf_obs_tables[i].state.y = DL_CBF_STATE_y_enu(buf);
  cbf_obs_tables[i].state.vx = DL_CBF_STATE_vx_enu(buf);
  cbf_obs_tables[i].state.vy = DL_CBF_STATE_vy_enu(buf);
  cbf_obs_tables[i].state.speed  = DL_CBF_STATE_speed(buf);
  cbf_obs_tables[i].state.course = DL_CBF_STATE_course(buf);
  cbf_obs_tables[i].state.uref = DL_CBF_STATE_uref(buf);
  
  cbf_obs_tables[i].available = (uint8_t) 1;
  cbf_obs_tables[i].t_last_msg = get_sys_time_msec();
  sprintf(msg,"Sender %u, Table pos %u",cbf_obs_tables[i].ac_id,i);
  //DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(msg), msg);
  }  
}

 void parseCBFTable(uint8_t *buf)
{
  uint8_t ac_id = DL_CBF_REG_TABLE_ac_id(buf);
  if (ac_id == AC_ID) {
      uint8_t nei_id = DL_CBF_REG_TABLE_nei_id(buf);
    
      if (nei_id == 0) {
        for (int i = 0; i < CBF_MAX_NEIGHBORS; i++) {
          //cbf_obs_tables[i].available = 0;
        }
      } 
      else {
        for (int i = 0; i < CBF_MAX_NEIGHBORS; i++){
          if (cbf_obs_tables[i].ac_id == nei_id) {
              cbf_obs_tables[i].state.x = DL_CBF_REG_TABLE_x_enu(buf);
              cbf_obs_tables[i].state.y = DL_CBF_REG_TABLE_y_enu(buf);
              cbf_obs_tables[i].state.vx = DL_CBF_REG_TABLE_vx_enu(buf);
              cbf_obs_tables[i].state.vy = DL_CBF_REG_TABLE_vy_enu(buf);
              cbf_obs_tables[i].state.speed  = DL_CBF_REG_TABLE_speed(buf);
              cbf_obs_tables[i].state.course = DL_CBF_REG_TABLE_course(buf);
              cbf_obs_tables[i].state.uref = DL_CBF_REG_TABLE_uref(buf);
              cbf_obs_tables[i].available = (uint8_t) 1;
              cbf_obs_tables[i].t_last_msg = get_sys_time_msec();
              return;
           
        }
      } 
      for (int i = 0; i < CBF_MAX_NEIGHBORS; i++){
        if (cbf_obs_tables[i].available == 0) {
            cbf_obs_tables[i].ac_id = nei_id;
          	cbf_obs_tables[i].state.x = DL_CBF_REG_TABLE_x_enu(buf);
            cbf_obs_tables[i].state.y = DL_CBF_REG_TABLE_y_enu(buf);
            cbf_obs_tables[i].state.vx = DL_CBF_REG_TABLE_vx_enu(buf);
            cbf_obs_tables[i].state.vy = DL_CBF_REG_TABLE_vy_enu(buf);
            cbf_obs_tables[i].state.speed  = DL_CBF_REG_TABLE_speed(buf);
            cbf_obs_tables[i].state.course = DL_CBF_REG_TABLE_course(buf);
            cbf_obs_tables[i].state.uref = DL_CBF_REG_TABLE_uref(buf);
            cbf_obs_tables[i].available = 1;
            cbf_obs_tables[i].t_last_msg = get_sys_time_msec();
            return;
        }
    }
 
  }
}
}
// Send the AC CBF_STATE to the neighborns network
static void send_cbf_state_to_nei(void)
{
  struct pprzlink_msg msg;
  if (cbf_control.n_neighborns > CBF_MAX_NEIGHBORS) {
   cbf_control.n_neighborns = CBF_MAX_NEIGHBORS;
  } // Limit the number of neighbors to CBF_MAX_NEIGHBORS 
  for (int i = 0; i < cbf_control.n_neighborns; i++){
    if (cbf_obs_tables[i].ac_id>0 && cbf_obs_tables[i].ac_id<255) {
        if(cbf_obs_tables[i].available==1){// send state to the ACs in CBF_NEI_AC_IDS
        msg.trans = &(DefaultChannel).trans_tx;
        msg.dev = &(DefaultDevice).device;
        msg.sender_id = AC_ID;
        msg.receiver_id = cbf_obs_tables[i].ac_id;
        msg.component_id = 0;
        
        // The information sended is redundant
      pprzlink_msg_send_CBF_STATE(&msg, &cbf_ac_state.x, &cbf_ac_state.y, 
                                          &cbf_ac_state.vx, &cbf_ac_state.vy, 
                                          &cbf_ac_state.speed, &cbf_ac_state.course,
                                          &cbf_ac_state.uref);
        /*char m[30]; 
        sprintf(m,"Enviado a %u",msg.receiver_id);
        DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(m), m);*/
      }
    }
    }
}

//---------------------------------------------------------------------------------
/* External functions --------------------------- */

// GVF modified by the CBFs generated by other agents
bool gvf_cbf(void){

float eta[CBF_MAX_NEIGHBORS];
float Aa[CBF_MAX_NEIGHBORS][2];
float b[CBF_MAX_NEIGHBORS];
int j=0; 
uint32_t now = get_sys_time_msec();
cbf_param.alpha = 0.1; 
 cbf_low_level_getState();
 send_cbf_state_to_nei();
 cbf_ac_state.xi_x=gvf_c_field.xi_x;
 cbf_ac_state.xi_y=gvf_c_field.xi_y;

 for (uint8_t i = 0; i < cbf_control.n_neighborns; ++i)  {
	
    if (cbf_obs_tables[i].available == 0) { 
    	continue; 
    	} // ignore unavailable neighborns

    uint32_t timeout = now - cbf_obs_tables[i].t_last_msg;

    if (timeout > cbf_param.timeout) { // ignore died neighborns 
      cbf_obs_tables[i].available = 0;
      cbf_telemetry.acs_timeslost[i] = cbf_telemetry.acs_timeslost[i] + 1;
      cbf_obs_tables[i].omega_safe = 0;
      
      continue;
      } 
     // Build the eta[j] (safe function)
     float dx=cbf_ac_state.x-cbf_obs_tables[i].state.x;
     float dy=cbf_ac_state.y-cbf_obs_tables[i].state.y;
   
     eta[i]=(dx)*(dx)+(dy)*(dy)-cbf_param.r*cbf_param.r;
     //  Test the condition for each neighbour. If the condition does not hold the requierement is active
     float bb;
     // To protect against division by zero
     if(eta[i]<1e-12){
        bb=0.0;
        eta[i]=0.0;
     }
     else{ 
      //bb=0.25*cbf_param.alpha*pow(eta[i],3);
      bb=0.25*cbf_param.alpha*eta[i]*eta[i]*eta[i];
     }
     
     printf("Condition%f\t",-dx*gvf_c_field.xi_x-dy*gvf_c_field.xi_y);
     printf("GT %f\t",bb);
     //TODO: Revisar cálculo y signos
     if ((-dx*gvf_c_field.xi_x-dy*gvf_c_field.xi_y) >= bb){ // Condition is active
        Aa[j][0]=-dx;
        Aa[j][1]=-dy;
        b[j]=bb;
        j++;
      }
   }  
  int active_conds=j;
  cbf_ac_state.active_conds=(uint8_t)active_conds;
 
  // Lagrange multiplier
  if (active_conds > 0 && active_conds <= CBF_MAX_NEIGHBORS) {
    if (active_conds==1){
      cbf_ac_state.alpha=19;
      double Aact[2], lambda_A[2];
      Aact[0]=Aa[0][0];
      Aact[1]=Aa[0][1];
      double norm_Aact=Aact[0]*Aact[0]+Aact[1]*Aact[1];
      if (norm_Aact<1e-6){
        norm_Aact=1e-6;
      }
      /*printf("A[0]=%f\t",Aact[0]);
      printf("A[1]=%f\t",Aact[1]);
      printf("norm_Aact=%f",norm_Aact);*/
      lambda_A[0]=(Aact[0]*gvf_c_field.xi_x-b[0])/ norm_Aact;
      lambda_A[1]=(Aact[1]*gvf_c_field.xi_y-b[0])/ norm_Aact;
      float cx=0.0,cy=0.0;
      /*printf("lambda_A[0]=%f\t",lambda_A[0]);
      printf("lambda_A[1]=%f\t",lambda_A[1]);*/
      cx=Aact[0]*lambda_A[0];
      cy=Aact[1]*lambda_A[1];

      cbf_ac_state.xicbf_x=gvf_c_field.xi_x-cx;
      cbf_ac_state.xicbf_y=gvf_c_field.xi_x-cy;
      /*printf("cx=%f\t",cx);
      printf("cy=%f\t",cy);*/

    }
    else{
      
      double Aact[active_conds][active_conds];
      for (uint8_t i=0;i<active_conds;i++){
        for   (uint8_t l=0;l<active_conds;l++){
          Aact[i][l]=Aa[i][0]*Aa[l][0]+Aa[i][1]*Aa[l][1];
        }
      }
      
      double L[active_conds][active_conds],D[active_conds],invA[active_conds][active_conds];
      cbf_ac_state.alpha=20;
    
      ldltDecomposition(Aact,L, D);
      cbf_ac_state.alpha=15;
      inverseUsingLDLT(L,D, invA);
      cbf_ac_state.alpha=16;
      double lambda_A[active_conds];
      for (uint8_t i=0;i<active_conds;i++){
        int su=0;
        for   (int l=0;l<active_conds;l++){
          su=invA[i][l]*(Aact[l][0]*gvf_c_field.xi_x+Aact[l][1]*gvf_c_field.xi_y-b[l]);
        }
        lambda_A[i]=su;
      }  
      float cx=0,cy=0;
      for (uint8_t i=0;i<active_conds;i++){
        cx=cx+Aact[i][0]*lambda_A[i];
        cy=cy+Aact[i][1]*lambda_A[i];
    
      }   
    }
  // Modified field (only if there are active conditions
    cbf_ac_state.active_conds=(uint8_t)active_conds;
    gvf_c_field.xi_x=cbf_ac_state.xicbf_x;
    gvf_c_field.xi_y=cbf_ac_state.xicbf_y;
 
 }    
 return true;
}
// Helpers

// Parse telemetry messages from air
void parse_CBF_STATE(uint8_t *buf)
{
  int16_t sender_id = (int16_t) pprzlink_get_msg_sender_id(buf);
  char msg[15];
  sprintf(msg,"Sender %u",sender_id);
  //DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(msg), msg);
  for (uint16_t i = 0; i < CBF_MAX_NEIGHBORS; i++){
    if (cbf_obs_tables[i].ac_id == sender_id) {
      write_cbf_table(i, buf);
      break;
    }
    }
}

void ldltDecomposition(double A[N1][N1], double L[N1][N1], double D[N1]) {
   cbf_ac_state.alpha=30;
    for (int i = 0; i < N1; i++) {
       cbf_ac_state.alpha=31;
        for (int j = 0; j <= i; j++) {
            double sum = 0;

            if (j == i) {
                for (int k = 0; k < j; k++) {
                    sum += L[j][k] * L[j][k] * D[k];
                }
                D[j] = A[j][j] - sum;
                L[j][j] = 1.0;
            } 
            else {
                for (int k = 0; k < j; k++) {
                    sum += L[i][k] * L[j][k] * D[k];
                }
            if (D[j]< 1e-6) {
                // Handle the case where D[j] is too small to avoid division by zero
                L[i][j] = 0.0; // or some other handling
               
            } else {
            
                L[i][j] = (A[i][j] - sum) / D[j];
                          
            }
        }
    }
}
}
void forwardSubstitution(double L[N1][N1], double b[N1], double y[N1]) {
    for (int i = 0; i < N1; i++) {
        double sum = 0.0;
        for (int j = 0; j < i; j++) {
            sum += L[i][j] * y[j];
        }
        y[i] = b[i] - sum;
    }
}

void diagonalSolve(double D[N1], double y[N1], double z[N1]) {
    for (int i = 0; i < N1; i++) {
        z[i] = y[i] / D[i];
    }
}

void backwardSubstitution(double L[N1][N1], double z[N1], double x[N1]) {
    for (int i = N1 - 1; i >= 0; i--) {
        double sum = 0.0;
        for (int j = i + 1; j < N1; j++) {
            sum += L[j][i] * x[j];
        }
        x[i] = z[i] - sum;
    }
}

void inverseUsingLDLT(double L[N1][N1], double D[N1], double invA[N1][N1]) {
    double y[N1], z[N1], x[N1], e[N1];

    for (int i = 0; i < N1; i++) {
        for (int j = 0; j < N1; j++) {
            e[j] = (i == j) ? 1.0 : 0.0; // Vector unitario
        }

        forwardSubstitution(L, e, y);
        diagonalSolve(D, y, z);
        backwardSubstitution(L, z, x);

        for (int j = 0; j < N1; j++) {
            invA[j][i] = x[j];
        }
    }
}
