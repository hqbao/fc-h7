#include "test.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include "main.h"

int g_test_counter = 0;

static void on_test(uint8_t *data, size_t size) {
	//platform_pwm_send(DSHOT_PORT1, g_test_counter);
	//platform_dshot_send(DSHOT_PORT1, g_test_counter);

	g_test_counter++;
	if (g_test_counter > 65000) g_test_counter = 0;
}

void test_setup(void) {
	//platform_pwm_init(PWM_PORT1);
	//platform_dshot_init(DSHOT_PORT1);
	subscribe(SCHEDULER_1KHZ, on_test);
}
