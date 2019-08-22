#include <string.h>

#include "vector.h"

//exponential moving average low pass filter
void lpf_ema_vector3d(vector3d_f_t new, vector3d_f_t *last, vector3d_f_t *filtered, float a)
{
	//fuse new data
	filtered->x = (new.x * a) + (last->x * (1 - a));
	filtered->y = (new.y * a) + (last->y * (1 - a));
	filtered->z = (new.z * a) + (last->z * (1 - a));

	//update old data
	memcpy((uint8_t *)last, (uint8_t *)filtered, sizeof(vector3d_f_t));
}
