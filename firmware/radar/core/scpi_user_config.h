/* libscpi configuration, identical for the host test build and XC16. */
#ifndef RADAR_SCPI_USER_CONFIG_H
#define RADAR_SCPI_USER_CONFIG_H

#define SCPI_LINE_ENDING "\n"
#define USE_FULL_ERROR_LIST 0
#define USE_USER_ERROR_LIST 1
#define USE_DEVICE_DEPENDENT_ERROR_INFORMATION 0
#define USE_COMMAND_TAGS 0
#define USE_DEPRECATED_FUNCTIONS 0
#define USE_UNITS_IMPERIAL 0
#define USE_UNITS_ANGLE 0
#define USE_UNITS_PARTICLES 0
#define USE_UNITS_DISTANCE 0
#define USE_UNITS_MAGNETIC 0
#define USE_UNITS_LIGHT 0
#define USE_UNITS_ENERGY_FORCE_MASS 0
#define USE_UNITS_TIME 0
#define USE_UNITS_TEMPERATURE 0
#define USE_UNITS_RATIO 0
#define USE_UNITS_POWER 0
#define USE_UNITS_FREQUENCY 0
#define USE_UNITS_ELECTRIC 0
#define USE_UNITS_ELECTRIC_CHARGE_CONDUCTANCE 0

#define LIST_OF_USER_ERRORS                                                    \
  X(RADAR_ERR_RANGE, 201, "Out of range")                                      \
  X(RADAR_ERR_CLOCK, -330, "Self-test failed")                                 \
  X(RADAR_ERR_CONFLICT, -221, "Settings conflict")

#endif
