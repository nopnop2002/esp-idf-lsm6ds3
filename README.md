# esp-idf-lsm6ds3
A demo showing the pose of the lsm6ds3 6DoF IMU sensor in 3D using esp-idf.   

You can use the Kalman filter or the Madgwick filter to estimate the Euler angle.   
Euler angles are roll, pitch and yaw.   
It's very intuitive and easy to understand.   
However, since LSM6DS3 is a 6DoF IMU, YAW estimation is not possible.   
![a-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original](https://user-images.githubusercontent.com/6020549/226072914-a7f923fc-eb6e-4d19-b2ff-8c9f2749ee6f.jpg)

I used [this](https://github.com/arduino-libraries/Arduino_LSM6DS3) library.   

# Software requiment
ESP-IDF V4.4/V5.x.   
ESP-IDF V5.0 is required when using ESP32-C2.   
ESP-IDF V5.1 is required when using ESP32-C6.   

# Hardware requirements
LSM6DS3 Accelerometer Gyroscope module 6 Dof inertial Measurement Sensors.   

# Wireing
|LSM6DS3||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6||
|:-:|:-:|:-:|:-:|:-:|:-:|
|VIN|--|N/C|N/C|N/C||
|3V3|--|3.3V|3.3V|3.3V||
|GND|--|GND|GND|GND||
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|
|CS|--|3.3V|3.3V|3.3V|Use i2c|
|SAO|--|GND/3.3V|GND/3.3V|GND/3.3V|(*2)|

(*1)You can change it to any pin using menuconfig.   

(*2)Choosing an i2c address.   
GND:i2c address is 0x6A.   
3.3V:i2c address is 0x6B.   


# Get Euler angles from lsm6ds3 using Kalman filter
I used [this](https://github.com/TKJElectronics/KalmanFilter).
```
git clone https://github.com/nopnop2002/esp-idf-lsm6ds3
cd esp-idf-lsm6ds3/Kalman
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

### Configuration
![config-top](https://user-images.githubusercontent.com/6020549/227118246-884d39fc-aaf8-4e65-bd7d-1ae233eb6dbb.jpg)
![config-app](https://user-images.githubusercontent.com/6020549/227118248-b2d41954-bb82-4e17-85e0-03e6c5925809.jpg)


# Get Euler angles from lsm6ds3 using Madgwick filter
I used [this](https://github.com/arduino-libraries/MadgwickAHRS).
```
git clone https://github.com/nopnop2002/esp-idf-lsm6ds3
cd esp-idf-lsm6ds3/Madgwick
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

### Configuration
![config-top](https://user-images.githubusercontent.com/6020549/227118246-884d39fc-aaf8-4e65-bd7d-1ae233eb6dbb.jpg)
![config-app](https://user-images.githubusercontent.com/6020549/227118248-b2d41954-bb82-4e17-85e0-03e6c5925809.jpg)

# View Euler angles with built-in web server   
ESP32 acts as a web server.   
I used [this](https://github.com/Molorius/esp32-websocket) component.   
This component can communicate directly with the browser.   
It's a great job.   
Enter the following in the address bar of your web browser.   
```
http:://{IP of ESP32}/
or
http://esp32.local/
```

![lsm6ds3-browser](https://user-images.githubusercontent.com/6020549/232383992-b9cff50e-fc86-4395-b698-3ddbc1ecdd53.jpg)

WEB pages are stored in the html folder.   
I used [this](https://canvas-gauges.com/) for gauge display.   
I used [this](https://threejs.org/) for 3D display.   
You can change the design and color according to your preference.   


# View Euler angles using PyTeapot   
You can view Euler angles using [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation) tool.   
It works as a UDP display server.   
This is a great application.   

```
+-------------+     +-------------+     +-------------+
|     IMU     | i2c |    ESP32    | UDP | pyteapot.py |
|             |---->|             |---->|             |
|             |     |             |     |             |
+-------------+     +-------------+     +-------------+
```

### Installation
```
$ sudo apt install python3-pip python3-setuptools
$ python3 -m pip install -U pip
$ python3 -m pip install pygame
$ python3 -m pip install PyOpenGL PyOpenGL_accelerate
$ git clone https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
$ cd PyTeapot-Quaternion-Euler-cube-rotation
$ python3 pyteapot.py
```
The posture of your sensor is displayed.   
![lsm6ds3_2023-03-23_10-52-26](https://user-images.githubusercontent.com/6020549/227118323-031c5dc6-d3d0-4169-8b0f-27c5948f91cf.png)

