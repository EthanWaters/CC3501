#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "SensorFusion/LSM9DS1_driver.h"
#include "math.h"
#include "SensorFusion/FusionArhs.h"
#include "SensorFusion/FusionAxes.h"
#include "SensorFusion/FusionCalibration.h"
#include "SensorFusion/FusionCompass.h"
#include "SensorFusion/FusionConvention.h"
#include "SensorFusion/FusionOffset.h"
#include "SensorFusion/FusionMath.h"
#include <time.h>


#define PIN_SA0 4
#define PIN_CS   5
#define PIN_SCK  23
#define PIN_SDA 6

float PI = 3.14159;
float rad_conv = (PI/180);

#define SAMPLE_RATE (100) // replace this with actual sample rate

int main()
{
    stdio_init_all();
    uint64_t start_time = time_us_64();

    // Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    //************************************************* SET UP LSM9DS1*************************************************************//
    LSM9DS1Driver lsm9ds1(PIN_SA0, PIN_CS, PIN_SCK, PIN_SDA);
    lsm9ds1.initialise_I2C(); // Set up I2C
    lsm9ds1.WHO_AM_I_CHECK_AG(); // Check that we're talkin' to accel/gyro//
    lsm9ds1.WHO_AM_I_CHECK_M();   // Check that we're talkin' to mag //
    lsm9ds1.CalibrateAG(); // Run sensor through calibration process //
    lsm9ds1.CalibrateMag();
    sleep_ms(2000); // you've done well, have a rest //

    //*********************************************** GET SENSOR DATA AND PERFORM FUSION *******************************************//
while (true) {
   
    OrientationData orientation = lsm9ds1.getdata();

    float pitch_rad = orientation.pitch*rad_conv;
    float yaw_rad = orientation.yaw*rad_conv;
    float roll_rad = orientation.roll*rad_conv;
    float ax_G = orientation.ax_G;
    float ay_G = orientation.ay_G;
    float az_G = orientation.az_G;
    float gx_dps = orientation.gx_dps;
    float gy_dps = orientation.gy_dps;
    float gz_dps = orientation.gz_dps;
    float mx_g = orientation.mx_g;
    float my_g = orientation.my_g;
    float mz_g = orientation.mz_g;

      
    // printf("Roll %0.01f, Pitch %0.01f, Yaw %0.01f\n",
    //            (roll_rad), (pitch_rad), (yaw_rad));

    // Acquire latest sensor data
        const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
        FusionVector gyroscope = {gx_dps, gy_dps, gz_dps}; // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {az_G, ay_G, az_G}; // replace this with actual accelerometer data in g
        FusionVector magnetometer = {-mx_g, my_g, mz_g}; // replace this with actual magnetometer data in arbitrary units

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        static clock_t previousTimestamp;
        const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
        previousTimestamp = timestamp;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n",
               euler.angle.roll, euler.angle.pitch, euler.angle.yaw);


}
}