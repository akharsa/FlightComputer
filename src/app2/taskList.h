
/*
 * taskList.h
 *
 *  Created on: 03/01/2013
 *      Author: alan
 */

#ifndef TASKLIST_H_
#define TASKLIST_H_


void Communications(void * pvParams);
void system(void * pvParams);
void hardwareInit(void);

#define COMMS_PRIORITY		2
#define AUTOPILOT_PRIORITY	3
#define TLM_PRIORITY	1


#endif /* TASKLIST_H_ */
