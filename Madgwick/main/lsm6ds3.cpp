#include <cstring>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
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

// Source: https://github.com/arduino-libraries/MadgwickAHRS
#include "MadgwickAHRS.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

Madgwick madgwick;

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

// Get time in seconds since boot
// Compatible with ROS's time.toSec() function
double TimeToSec() {
	int64_t _time = esp_timer_get_time(); // Get time in microseconds since boot
	double __time = (double)_time / 1000000;
	return __time;
}

void lsm6ds3(void *pvParameters){
	// Initialize device
	if (imu.begin() == 0) {
		ESP_LOGE(TAG, "Connection fail");
		vTaskDelete(NULL);
	}
	
	int elasped = 0;
	double last_time_ = TimeToSec();

	bool initialized = false;
	float initial_roll = 0.0;
	float initial_pitch = 0.0;
	float initial_yaw = 0.0;

	// It takes time for the estimated value to stabilize.
	// It need about 4Sec.
	int initial_period = 400;

	while(1){
		double ax=0.0, ay=0.0, az=0.0;
		double gx=0.0, gy=0.0, gz=0.0;
		_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//printf("%f %f %f - %f %f %f\n", ax, ay, az, gx, gy, gz);

		// Get the elapsed time from the previous
		float dt = (TimeToSec() - last_time_);
		ESP_LOGD(TAG, "dt=%f",dt);
		last_time_ = TimeToSec();

		// Get Euler
		madgwick.updateIMU(gx, gy, gz, ax, ay, az, dt);
		float roll = madgwick.getRoll();
		float pitch = madgwick.getPitch();
		float yaw = madgwick.getYaw();
		ESP_LOGD(TAG, "roll=%f pitch=%f yaw=%f", roll, pitch, yaw);


		/* Print Data every 10 times */
		if (elasped > initial_period) {
			// Set the first data
			if (!initialized) {
				initial_roll = roll;
				initial_pitch = pitch;
				initial_yaw = yaw;
				initialized = true;
				initial_period = 10;
			}

			// Send UDP packet
			float _roll = roll-initial_roll;
			float _pitch = pitch-initial_pitch;
			float _yaw = yaw-initial_yaw;
			ESP_LOGD(TAG, "roll:%f pitch=%f yaw=%f", roll, pitch, yaw);
			ESP_LOGI(TAG, "roll:%f pitch=%f yaw=%f", _roll, _pitch, _yaw);

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
