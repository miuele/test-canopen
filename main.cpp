/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

#include "CANopen.h"
#include "OD.h"

#include <mstd_atomic>

/* default values for CO_CANopenInit() */
#define NMT_CONTROL \
            (CO_NMT_ERR_ON_ERR_REG \
          | CO_ERR_REG_GENERIC_ERR \
          | CO_ERR_REG_COMMUNICATION)

#ifndef NMT_CONTROL
#define NMT_CONTROL \
            (CO_NMT_STARTUP_TO_OPERATIONAL \
          | CO_NMT_ERR_ON_ERR_REG \
          | CO_ERR_REG_GENERIC_ERR \
          | CO_ERR_REG_COMMUNICATION)
#endif
#ifndef FIRST_HB_TIME
#define FIRST_HB_TIME 500
#endif
#ifndef SDO_SRV_TIMEOUT_TIME
#define SDO_SRV_TIMEOUT_TIME 1000
#endif
#ifndef SDO_CLI_TIMEOUT_TIME
#define SDO_CLI_TIMEOUT_TIME 500
#endif
#ifndef SDO_CLI_BLOCK
#define SDO_CLI_BLOCK false
#endif
#ifndef OD_STATUS_BITS
#define OD_STATUS_BITS NULL
#endif

void log_printf(const char *msg) {
	mbed_event_queue()->call([msg] {
			printf(msg);
		});
}

CAN can(PA_11, PA_12, 500'000);
Ticker co_ticker;
Timer co_timer_irq;
Timer co_timer_com;

DigitalOut led(LED1);

void process_co(CO_t *CO) {
	auto dt = co_timer_irq.elapsed_time().count();

	CO_process_RPDO(CO, false, dt, NULL);
	CO_process_TPDO(CO, false, dt, NULL);

	co_timer_irq.reset();
}

volatile int task_pending = 0;


Timer tim_perf;
int perf = 0;

int main()
{
	OD_extension_t ext{};
	ext.read = OD_readOriginal;
	ext.write =
		[](OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten)
		-> ODR_t {
			unsigned char val = *static_cast<const unsigned char *>(buf);

			if (val == 1) {
				log_printf("received ignite\n");
				task_pending = mbed_event_queue()->call_in(3s, [] {
						led.write(1);
						OD_RAM.x2111_ledint = 1;
						task_pending = mbed_event_queue()->call_in(3s, []{
							led.write(0);
							OD_RAM.x2111_ledint = 0;
							task_pending = 0;
						});
					});
			} else if (val == 0) {
				log_printf("received cancel\n");
				mbed_event_queue()->call([]{
						if (task_pending) {
							led.write(0);
							OD_RAM.x2111_ledint = 0;
							mbed_event_queue()->cancel(task_pending);
						}
					});
			}
			return ODR_OK;
		};
	OD_extension_init(OD_find(OD, 0x2111), &ext);
comm_reset:
	uint32_t size;
	CO_t *CO = CO_new(NULL, &size);
	CO_ReturnError_t err;
	err = CO_CANinit(CO, (void *)&can, 500);
	if (err != CO_ERROR_NO) {
		error("Error: CAN initialization failed: %d\n", err);
	} else {
		printf("Allocated %lu bytes for CANopen objects\n", size);
	}

	uint32_t errInfo = 0;

	err = CO_CANopenInit(CO,                /* CANopen object */
			NULL,              /* alternate NMT */
			NULL,              /* alternate em */
			OD,                /* Object dictionary */
			OD_STATUS_BITS,    /* Optional OD_statusBits */
			(CO_NMT_control_t)NMT_CONTROL,       /* CO_NMT_control_t */
			FIRST_HB_TIME,     /* firstHBTime_ms */
			SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
			SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
			SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
			0x05,
			&errInfo);

	if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
		if (err == CO_ERROR_OD_PARAMETERS) {
			error("Error: Object Dictionary entry 0x%lX\n", errInfo);
		} else {
			error("Error: CANopen initialization failed: %d\n", err);
		}
	}

	err = CO_CANopenInitPDO(CO, CO->em, OD, 0x05, &errInfo);

	if (err != CO_ERROR_NO) {
		if (err == CO_ERROR_OD_PARAMETERS) {
			error("Error: Object Dictionary entry 0x%lX\n", errInfo);
		} else {
			error("Error: PDO initialization failed: %d\n", err);
		}
	}
	
	CO_CANsetNormalMode(CO->CANmodule);

	co_timer_irq.start();
	co_ticker.attach(callback(process_co, CO), 1000us);

	printf("Hello, Mbed!\n");

	co_timer_com.start();

	tim_perf.start();

	mbed_event_queue()->call_every(1ms, [CO]() {
		auto dt = co_timer_com.elapsed_time().count();

		tim_perf.reset();
		CO_NMT_reset_cmd_t reset = CO_process(CO, false, dt, NULL);
		perf = tim_perf.elapsed_time().count();

		if (reset == CO_RESET_COMM) {
			co_ticker.detach();
			printf("CANopenNode Reset Communication request\n");
			CO_CANsetConfigurationMode(CO->CANmodule);
			CO_CANmodule_disable(CO->CANmodule);
			CO_delete(CO);
			mbed_event_queue()->break_dispatch();
		} else if (reset == CO_RESET_APP) {
			HAL_NVIC_SystemReset();
		}


		co_timer_com.reset();
	});
	
	mbed_event_queue()->call_every(1s, []() {
			printf("took %d us to process\n", perf);
		});

	mbed_event_queue()->dispatch_forever();

	printf("out from equeue loop\n");

	goto comm_reset;
}

