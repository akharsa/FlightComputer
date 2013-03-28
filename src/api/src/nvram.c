
#include "nvram.h"
#include "eeprom.h"
#include "quadrotor.h"
#include "board.h"

nvram_t nvramBuffer;

Status qNVRAM_Load(nvram_t * p){
	if (eeprom_read(EEPROM_ADDRESS,(uint8_t*)p,0x0000,sizeof(nvram_t))!=SUCCESS){
		return ERROR;
	}else{
		quadrotor.rateController[ROLL].K = p->rateController[ROLL].K;
		quadrotor.rateController[ROLL].Ti = p->rateController[ROLL].Ti;
		quadrotor.rateController[ROLL].Td = p->rateController[ROLL].Td;
		quadrotor.rateController[ROLL].Nd = p->rateController[ROLL].Nd;

		quadrotor.rateController[PITCH].K = p->rateController[PITCH].K;
		quadrotor.rateController[PITCH].Ti = p->rateController[PITCH].Ti;
		quadrotor.rateController[PITCH].Td = p->rateController[PITCH].Td;
		quadrotor.rateController[PITCH].Nd = p->rateController[PITCH].Nd;

		quadrotor.rateController[YAW].K = p->rateController[YAW].K;
		quadrotor.rateController[YAW].Ti = p->rateController[YAW].Ti;
		quadrotor.rateController[YAW].Td = p->rateController[YAW].Td;
		quadrotor.rateController[YAW].Nd = p->rateController[YAW].Nd;

		quadrotor.attiController[ROLL].K = p->attiController[ROLL].K;
		quadrotor.attiController[ROLL].Ti = p->attiController[ROLL].Ti;
		quadrotor.attiController[ROLL].Td = p->attiController[ROLL].Td;
		quadrotor.attiController[ROLL].Nd = p->attiController[ROLL].Nd;

		quadrotor.attiController[PITCH].K = p->attiController[PITCH].K;
		quadrotor.attiController[PITCH].Ti = p->attiController[PITCH].Ti;
		quadrotor.attiController[PITCH].Td = p->attiController[PITCH].Td;
		quadrotor.attiController[PITCH].Nd = p->attiController[PITCH].Nd;

		quadrotor.K = p->attiController[YAW].K;
		quadrotor.attiController[YAW].Ti = p->attiController[YAW].Ti;
		quadrotor.attiController[YAW].Td = p->attiController[YAW].Td;
		quadrotor.attiController[YAW].Nd = p->attiController[YAW].Nd;

		quadrotor.altitudeController.K = p->altitudeController.K;
		quadrotor.altitudeController.Ti = p->altitudeController.Ti;
		quadrotor.altitudeController.Td = p->altitudeController.Td;
		quadrotor.altitudeController.Nd = p->altitudeController.Nd;
		return SUCCESS;
	}
}

Status qNVRAM_setDefaults(nvram_t * p){
	p->rateController[ROLL].K = 0.02;
	p->rateController[ROLL].Ti = 1/0.03;
	p->rateController[ROLL].Td = 0.000;
	p->rateController[ROLL].Nd = 5;

	p->rateController[PITCH].K = 0.02;
	p->rateController[PITCH].Ti = 1/0.03;
	p->rateController[PITCH].Td = 0.000;
	p->rateController[PITCH].Nd = 5;

	p->rateController[YAW].K = 0.1;
	p->rateController[YAW].Ti = 1/0.2;
	p->rateController[YAW].Td = 0.000;
	p->rateController[YAW].Nd = 5;
	return eeprom_write(EEPROM_ADDRESS,(uint8_t*)p,0x0000,sizeof(nvram_t));
}

Status qNVRAM_Save(nvram_t * p){
	return eeprom_write(EEPROM_ADDRESS,(uint8_t*)p,0x0000,sizeof(nvram_t));
}
