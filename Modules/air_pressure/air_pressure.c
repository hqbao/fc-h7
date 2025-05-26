#include "air_pressure.h"
#include <pubsub.h>
#include <platform.h>
#include "dps310.h"

static float g_pressure_alt = 0;

static void air_pressure_init(void) {
	platform_delay(100);
	dps310_probe();
	platform_delay(100);
}

static void air_pressure_loop(uint8_t *data, size_t size) {
#define ALT_SAMPLES 50
    static int g_alt_counter = -10;
    static float g_alt_off = 0;
    if (g_alt_counter > ALT_SAMPLES) {
    	g_pressure_alt = 1000 * (get_altitude() - g_alt_off);
    } else if (g_alt_counter == ALT_SAMPLES) {
        g_alt_off = g_alt_off / ALT_SAMPLES;
        g_alt_counter += 1;
    } else if (g_alt_counter >= 0) {
        g_alt_off += get_altitude();
        g_alt_counter += 1;
    } else {
        g_alt_counter += 1;
    }
}

void air_pressure_setup(void) {
	air_pressure_init();
	subscribe(LOOP, air_pressure_loop);
}
