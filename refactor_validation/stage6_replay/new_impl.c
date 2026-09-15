/* Stage 6 被测实现：直接调用实际迁移后的公开接口，
 * 不复制算法、不做任何等价改写。 */
#include "motor_speed_loop.h"

float Adaptive_Call(float published_speed_rpm)
{
    return Motor_SpeedEstimator_UpdateAdaptive(published_speed_rpm);
}
