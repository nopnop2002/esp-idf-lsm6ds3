idf_component_register(
	SRCS "LSM6DS3.cpp"
	INCLUDE_DIRS "."
	PRIV_REQUIRES driver esp_driver_i2c esp_timer
)
