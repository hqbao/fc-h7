#include <imu/imu.h>
#include <attitude_fusion/attitude_fusion.h>
#include <attitude_fusion_ekf/attitude_fusion_ekf.h>
#include <air_pressure/air_pressure.h>
#include <attitude_control/attitude_control.h>
#include <remote_control/remote_control.h>
#include <imu_calibrator/imu_calibrator.h>
#include <navigation/navigation.h>
#include <logger/logger.h>
#include <test/test.h>

void platform_setup(void) {
	imu_setup();
	attitude_fusion_setup();
	//attitude_fusion_ekf_setup();
	air_pressure_setup();
	attitude_control_setup();
	remote_control_setup();
	navigation_setup();
	logger_setup();
//	test_setup();
//	imu_calibrator_setup();
}
