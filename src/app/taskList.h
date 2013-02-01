
/*
 * taskList.h
 *
 *  Created on: 03/01/2013
 *      Author: alan
 */

#ifndef TASKLIST_H_
#define TASKLIST_H_

void SystemController(void * pvParams);
void Communications(void * pvParams);



#define SYSCON_PRIORITY	3
#define COMMS_PRIORITY	4
#define INIT_PRIORITY	2
#define IDLE_PRIORITY	2
#define FLIGHT_PRIORITY	5



#endif /* TASKLIST_H_ */
