#include "cputimer.h"

#include <sys/time.h>
#include <sys/resource.h>

namespace util {
double cpu_timer() {
    struct rusage usage;
    (void) getrusage(RUSAGE_SELF, &usage);
    return usage.ru_utime.tv_sec * 1.0 + usage.ru_utime.tv_usec / 1e6;
}
}  // namespace util
