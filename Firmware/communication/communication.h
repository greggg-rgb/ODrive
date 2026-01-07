#ifndef COMMANDS_H
#define COMMANDS_H

#ifndef assert
#define assert(expr)
#endif

#ifdef __cplusplus

#include <functional>
#include <limits>

extern "C" {
#endif

#include <cmsis_os.h>

void init_communication(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */
