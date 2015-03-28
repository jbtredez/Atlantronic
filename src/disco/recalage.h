#ifndef RECALAGE_H
#define RECALAGE_H

#ifndef WEAK_RECALAGE
#define WEAK_RECALAGE __attribute__((weak, alias("nop_function") ))
#endif

void recalage() WEAK_RECALAGE;

#endif
