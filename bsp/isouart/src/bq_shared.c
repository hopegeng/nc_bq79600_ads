/*
 * bq_shared.c
 *
 *  Created on: Nov 4, 2025
 *      Author: rgeng
 */


#include "bq_shared.h"

/* Put shared data in CPU-shared memory section */
volatile __attribute__((section ("BQ_SHARED"))) BQ_ShareData_t g_bqSharedData;

