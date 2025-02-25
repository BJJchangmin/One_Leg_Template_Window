#ifndef DATALOGGING_H_
#define DATALOGGING_H_

#include "globVariable.h"

/***************** Data Logging *****************/
FILE* fid;
int loop_index = 0;
const int data_frequency = 1; // frequency at which data is written to a file

char datapath[] = "../myProject/slip_realization_kws/data/data.csv";
char filename[] = "../myProject/slip_realization_kws/data/scene.xml";
char datafile[] = "data.csv";

void init_save_data()
{
    // This function is called once and is used to get the headers
    // Write name of the variable here (header)
    // comma(,) should be omitted in the last line.
    
    fprintf(fid, "t, ");
    fprintf(fid, "r_ref, qr_ref, r_act, qr_act, ");
    fprintf(fid, "out_acc, out_acc_old, out_acc_old2, ydR_act,");
    fprintf(fid, "fxR_hat, fyR_hat, ");
    fprintf(fid, "grf_x, grf_y, grf_z, ");
    fprintf(fid, "touch, ");
    fprintf(fid, "trunk_vel_y ");
    
    // Don't remove the newline
    fprintf(fid, "\n");
}

void save_data(const mjModel* m, mjData* d, StateModel* state_model)
{
    // This function is called at a set frequency,put data here.
    // Data here should correspond to headers in init_save_data()
    // Seperate data by a space %f followed by space
    // comma(,) should be omitted in the last line.
    double touch = d->sensordata[8];
    double grf_x = d->sensordata[9];
    double grf_y = d->sensordata[10];
    double grf_z = d->sensordata[11];
    double trunk_vel_y = d->sensordata[15];

    double cartesian_grf_x = grf_x * cos(pi - state_model->q_bi[1]) - grf_y * sin(pi - state_model->q_bi[1]);
    double cartesian_grf_y = grf_x * sin(pi - state_model->q_bi[1]) + grf_y * cos(pi - state_model->q_bi[1]); 

    double grf_r = cartesian_grf_y * cos(state_model->posRW[1] - pi / 2) + cartesian_grf_x * sin(state_model->posRW[1] - pi / 2);
    double grf_thetar = cartesian_grf_y * sin(state_model->posRW[1] - pi / 2) - cartesian_grf_x * cos(state_model->posRW[1] - pi / 2);

    fprintf(fid, "%f, ", d->time);
    
    fprintf(fid, "%f, %f, %f, %f, ", state_model->posRW_ref[0], state_model->posRW_ref[1], state_model->posRW[0], state_model->posRW[1]);
    fprintf(fid, "%f, %f, %f, %f, ", state_model->out_acc, state_model->out_acc_old, state_model->out_acc_old2, d->sensordata[7]);

    
    
    fprintf(fid, "%f, %f, ", state_model->forceExt_hat[0], state_model->forceExt_hat[1]); // forceEXT[0]�� r���� , ī�׽þƳ߼� y����
    
    fprintf(fid, "%f, %f, %f, ", grf_r, grf_thetar, grf_z);
    
    fprintf(fid, "%f, ", touch);

    fprintf(fid, "%f ", state_model->velRW[0]);
    
    
    // Don't remove the newline
    fprintf(fid, "\n");


}

#endif // DATALOGGING_H_