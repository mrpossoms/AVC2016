all:
	gcc imu.c imuTest.c
calibrator:
	gcc imu.c calibrator.c -o calibrator.bin
