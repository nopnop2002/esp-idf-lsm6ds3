#include <cstring>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "cJSON.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// I2Cdev and LSM303DLHC must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "LSM6DS3.h"

LSM6DS3 imu(CONFIG_I2C_ADDR);

// Source: https://github.com/TKJElectronics/KalmanFilter
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

void _getMotion6(double *_ax, double *_ay, double *_az, double *_gx, double *_gy, double *_gz) {
	float ax=0.0, ay=0.0, az=0.0;
	float gx=0.0, gy=0.0, gz=0.0;
#if 0
	if (imu.accelerationAvailable()) {
		imu.readAcceleration(ax, ay, az);
	}
	if (imu.gyroscopeAvailable()) {
		imu.readGyroscope(gx, gy, gz);
	}
#endif
	while(1) {
		if (imu.accelerationAvailable()) break;
		vTaskDelay(1);
	}
	imu.readAcceleration(ax, ay, az);
	while(1) {
		if (imu.gyroscopeAvailable()) break;
		vTaskDelay(1);
	}
	imu.readGyroscope(gx, gy, gz);

	*_ax = ax;
	*_ay = ay;
	*_az = az;
	*_gx = gx;
	*_gy = gy;
	*_gz = gz;
}

void getRollPitch(double accX, double accY, double accZ, double *roll, double *pitch) {
	// atan2 outputs the value of - to	(radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	*roll = atan2(accY, accZ) * RAD_TO_DEG;
	*pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	*roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	*pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void lsm6ds3(void *pvParameters){
	// Initialize device
	if (imu.begin(400000) == 0) {
		ESP_LOGE(TAG, "Connection fail");
		vTaskDelete(NULL);
	}
	
	// Set Kalman and gyro starting angle
	double accX, accY, accZ;
	double gyroX, gyroY, gyroZ;
	double roll, pitch; // Roll and pitch are calculated using the accelerometer
	double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

	_getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
	getRollPitch(accX, accY, accZ, &roll, &pitch);
	kalAngleX = roll;
	kalAngleY = pitch;
	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	uint32_t timer = micros();
	
	int elasped = 0;
	bool initialized = false;
	double initial_kalAngleX = 0.0;
	double initial_kalAngleY = 0.0;

	while(1){
		_getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
		//printf("%f %f %f - %f %f %f\n", accX, accY, accZ, gyroX, gyroY, gyroZ);
		getRollPitch(accX, accY, accZ, &roll, &pitch);

		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		/* Roll and pitch estimation */
		double gyroXrate = gyroX;
		double gyroYrate = gyroY;

#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanX.setAngle(roll);
			kalAngleX = roll;
		} else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	

		if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			kalAngleY = pitch;
		} else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		/* Print Data every 10 times */
		if (elasped > 10) {
			// Set the first data
			if (!initialized) {
				initial_kalAngleX = kalAngleX;
				initial_kalAngleY = kalAngleY;
				initialized = true;
			}

#if 0
			printf("roll:%f", roll); printf(" ");
			printf("kalAngleX:%f", kalAngleX); printf(" ");
			printf("initial_kalAngleX:%f", initial_kalAngleX); printf(" ");
			printf("kalAngleX-initial_kalAngleX:%f", kalAngleX-initial_kalAngleX); printf(" ");
			printf("\n");

			printf("pitch:%f", pitch); printf(" ");
			printf("kalAngleY:%f", kalAngleY); printf(" ");
			printf("initial_kalAngleY:%f", initial_kalAngleY); printf(" ");
			printf("kalAngleY-initial_kalAngleY: %f", kalAngleY-initial_kalAngleY); printf(" ");
			printf("\n");
#endif

			// Send UDP packet
			float _roll = kalAngleX-initial_kalAngleX;
			float _pitch = kalAngleY-initial_kalAngleY;
			ESP_LOGI(TAG, "roll:%f pitch=%f", _roll, _pitch);

			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = 0.0;
			if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
			}

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", _roll);
			cJSON_AddNumberToObject(request, "pitch", _pitch);
			cJSON_AddNumberToObject(request, "yaw", 0.0);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);

			vTaskDelay(1);
			elasped = 0;
		}
		elasped++;
		vTaskDelay(1);
	}

	// Never reach here
	vTaskDelete(NULL);
}
