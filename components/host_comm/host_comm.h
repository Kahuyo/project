#pragma once
#include "components/needle/needle_motor.h"
#include "components/screw/screw_motor.h"

void host_comm_init(int uart_num, needle_motor_t *nm, screw_motor_t *sm);
void host_comm_start(void);
