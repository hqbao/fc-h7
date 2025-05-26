#include <imu/imu.h>
#include <attitude_fusion/attitude_fusion.h>
#include <air_pressure/air_pressure.h>
#include <attitude_control/attitude_control.h>
#include <test/test.h>

void platform_setup(void) {
	imu_setup();
	attitude_fusion_setup();
//	air_pressure_setup();
	attitude_control_setup();
	test_setup();
}
