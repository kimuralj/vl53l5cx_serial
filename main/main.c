#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l5cx_api.h"
#include "kalman.h"

//------------------- Compilation Flags ----------------------/
//! Flag to indicate if it shall use an envelope of values
#define IS_ENVELOPE_ENABLED     true
//! Flag to indicate if it shall use the Kalman filter
#define IS_KALMAN_ENABLED       true

//----------------------- Constants --------------------------/
//! Constant for the initial values of the Kalmann filter
#define KALMAN_INITIAL_VALUE    300
//! Constant for the amount of pixels in the matrix
#define MATRIX_PIXEL_SIZE       64
//! Envelope filter value. The maximum allowed distance
#define ENVELOPE_FILTER_VALUE   300

//----------------------- Variables --------------------------/
#if IS_KALMAN_ENABLED
//! The Kalman filter structs (one per cell in the 8x8 matrix)
static KALMAN_STRUCT_T kalman[MATRIX_PIXEL_SIZE];
#endif

//------------------ Function Definitions --------------------/
/**
 * Main function, the code starts running from here
 */
void app_main(void)
{
    //Define the i2c bus configuration
    i2c_port_t i2c_port = I2C_NUM_0;
    i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 5,
            .scl_io_num = 4,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = VL53L5CX_MAX_CLK_SPEED,
    };

    i2c_param_config(i2c_port, &i2c_config);
    i2c_driver_install(i2c_port, i2c_config.mode, 0, 0, 0);

    /*********************************/
    /*   VL53L5CX ranging variables  */
    /*********************************/

    uint8_t 				status, isAlive, isReady;
    VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
    VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */


    /*********************************/
    /*      Customer platform        */
    /*********************************/

    /* Fill the platform structure with customer's implementation. For this
    * example, only the I2C address is used.
    */
    Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    Dev.platform.port = i2c_port;

    /* (Optional) Reset sensor toggling PINs (see platform, not in API) */
    //Reset_Sensor(&(Dev.platform));

    /* (Optional) Set a new I2C address if the wanted address is different
    * from the default one (filled with 0x20 for this example).
    */
    //status = vl53l5cx_set_i2c_address(&Dev, 0x20);


    /*********************************/
    /*   Power on sensor and init    */
    /*********************************/

    /* (Optional) Check if there is a VL53L5CX sensor connected */
    status = vl53l5cx_is_alive(&Dev, &isAlive);
    if(!isAlive || status)
    {
        printf("VL53L5CX not detected at requested address\n");
        return;
    }

    /* (Mandatory) Init VL53L5CX sensor */
    status = vl53l5cx_init(&Dev);
    if(status)
    {
        printf("VL53L5CX ULD Loading failed\n");
        return;
    }

    printf("VL53L5CX ULD ready ! (Version : %s)\n",
           VL53L5CX_API_REVISION);

    vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_ranging_frequency_hz(&Dev, 5);
    vl53l5cx_set_ranging_mode(&Dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    vl53l5cx_set_sharpener_percent(&Dev, 20);

    // Starts ranging
    status = vl53l5cx_start_ranging(&Dev);

#if IS_KALMAN_ENABLED
    // Initializes the Kalman filter array
    for(int i = 0; i < MATRIX_PIXEL_SIZE; i++)
    {
        Kalman_Initialize(&kalman[i], KALMAN_INITIAL_VALUE);
    }
#endif

    /***************************************/
    /*   Super loop running indefinitely   */
    /***************************************/
    while(1)
    {
        /* Use polling function to know when a new measurement is ready.
         * Another way can be to wait for HW interrupt raised on PIN A1
         * (INT) when a new measurement is ready */

        status = vl53l5cx_check_data_ready(&Dev, &isReady);

        if(isReady)
        {
            vl53l5cx_get_ranging_data(&Dev, &Results);

            // The sensor is set in 8x8 mode with a total 64 zones to print.
            printf("Print data no : %3u\n", Dev.streamcount);
            for(int i = 0; i < MATRIX_PIXEL_SIZE; i++)
            {
                // Variable for temporary holding the returned value
                float filtered_value = Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i];

                #if IS_ENVELOPE_ENABLED
                // Apply envelope for filtering extreme outliers
                if(Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i] > ENVELOPE_FILTER_VALUE)
                {
                    filtered_value = ENVELOPE_FILTER_VALUE;
                }
                #endif
                
                #if IS_KALMAN_ENABLED
                // Filter all the data from the current iteration
                filtered_value = Kalman_Update(&kalman[i], filtered_value);
                #endif

                // Serial print the filtered value
                printf("%d", (int) filtered_value);

                // If the row is over, print "\n", otherwise print the comma separator
                if(i % 8 == 7) printf("\n");
                else printf(",");
            }
            // printf("\n");
        }

        // Wait a few ms to avoid too high polling (function in platform file, not in API)
        WaitMs(&(Dev.platform), 5);
    }

    // It shall never get here, unless an abort condition is implemented on the loop 

    // Finish execution
    status = vl53l5cx_stop_ranging(&Dev);
    printf("End of execution\n");
}