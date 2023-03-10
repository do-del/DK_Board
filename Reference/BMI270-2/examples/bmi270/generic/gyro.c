/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi2.h"
#include "bmi270.h"
#include "bmi2_common.h"

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for gyro.
 *
 *  @param[in] dev       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_gyro_config(struct bmi2_dev *dev);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Gravity.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print gyro data. */
    uint8_t limit = 10;

    uint8_t indx;
    float x = 0, y = 0, z = 0;

    /*! Sensor initialization configuration. */
    struct bmi2_dev bmi2_dev;

    /* Create an instance of sensor data structure. */
    struct bmi2_sensor_data sensor_data = { 0 };

    /* Assign gyro sensor to variable. */
    uint8_t sens_list = BMI2_GYRO;

    /* Initialize the interrupt status of gyro. */
    uint16_t int_status = 0;

    /* Initialize the dev structure */
    rslt = bmi2_interface_selection(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270. */
    rslt = bmi270_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {

        /* Enable the selected sensors. */
        rslt = bmi2_sensor_enable(&sens_list, 1, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK)
        {

            /* Gyro configuration settings. */
            rslt = set_gyro_config(&bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            /* Select gyro sensor. */
            sensor_data.type = BMI2_GYRO;
            printf("Gyro and DPS data");

            /* Loop to print gyro data when interrupt occurs. */
            do
            {

                /* To get the data ready interrupt status of gyro. */
                rslt = bmi2_get_int_status(&int_status, &bmi2_dev);

                /* To check the data ready interrupt status and print the status for 10 samples. */
                for (indx = 0; (indx < limit && (int_status & BMI2_GYR_DRDY_INT_MASK)); indx++)
                {

                    /* Get gyro data for x, y and z axis. */
                    rslt = bmi2_get_sensor_data(&sensor_data, 1, &bmi2_dev);
                    printf("\nGyrx= %d\t", sensor_data.sens_data.gyr.x);
                    printf("Gyry= %d\t", sensor_data.sens_data.gyr.y);
                    printf("Gyrz= %d\r\n", sensor_data.sens_data.gyr.z);

                    /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                    x = lsb_to_dps(sensor_data.sens_data.gyr.x, 2000, bmi2_dev.resolution);
                    y = lsb_to_dps(sensor_data.sens_data.gyr.y, 2000, bmi2_dev.resolution);
                    z = lsb_to_dps(sensor_data.sens_data.gyr.z, 2000, bmi2_dev.resolution);

                    /* Print the data in dps. */
                    printf("Dx = %4.2f, Dy = %4.2f, Dz = %4.2f\n", x, y, z);
                }
            } while (int_status != BMI2_GYR_DRDY_INT_MASK);
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to set configurations for gyro.
 */
static int8_t set_gyro_config(struct bmi2_dev *dev)
{

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(&config, 1, dev);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {

        /* The user can change the following configuration parameters according to their requirement. */
        /* Output Data Rate. By default ODR is set as 200Hz for gyro. */
        config.cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config.cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the gyro configurations. */
        rslt = bmi2_set_sensor_config(&config, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI2_GYR_RANGE_2000)) * (val);
}
