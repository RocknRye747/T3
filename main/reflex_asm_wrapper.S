// Build wrapper that keeps firmware/reflex/reflex.S untouched while
// avoiding symbol collision with ESP-IDF startup _start.
#define _start reflex_kernel_start
#include "../firmware/reflex/reflex.S"
