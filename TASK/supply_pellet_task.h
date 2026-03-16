#ifndef __SUPPLY_PELLET_TASK_H
#define __SUPPLY_PELLET_TASK_H

#include "rm_algorithm.h"
#include "dji_protocol.h"
#include "rm_types_my.h"
#include "math.h"


void SupplyPelletControl(void);
void SupplyMotor_RC_Mode(void);
void SupplyMotor_KeyMouse_Mode(void);
void PelletSupply_Loop(void);
void SupplyPelletRevise(void);
void SupplyMotor_Speed_Mode(void);
void SupplyMotor_Single_Mode(void);
void HappyShooting(void);
void Shoot_freq_calculate(void);
void SupplyImage_Speed_Mode(void);
bool Is_Heat_Safe(void);

extern float Bullet_prespeed  ;
extern int Bullet_num_actaul ;
extern int Bullet_num_fb;
extern int shoot_num_buchang;

#endif
