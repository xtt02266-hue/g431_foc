#ifndef BOARD_PROFILE_H
#define BOARD_PROFILE_H

#ifndef FOC_PROFILE_SENSORLESS
#define FOC_PROFILE_SENSORLESS 0
#endif

/* Sensorless development has no startup/handover controller yet. */
#define BOARD_SENSORED_CONTROL_ENABLE (!FOC_PROFILE_SENSORLESS)
#define BOARD_AS5600_REFERENCE_ENABLE FOC_PROFILE_SENSORLESS

#endif
