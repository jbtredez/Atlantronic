#ifndef SRC_DISCO_POWER_H_
#define SRC_DISCO_POWER_H_

#ifndef WEAK_CUTMOTORSOFF
#define WEAK_CUTMOTORSOFF __attribute__((weak, alias("nop_function") ))
#endif

#ifndef WEAK_CUTSERVOSOFF
#define WEAK_CUTSERVOSOFF __attribute__((weak, alias("nop_function") ))
#endif

#ifndef WEAK_FUNNYACTION
#define WEAK_FUNNYACTION __attribute__((weak, alias("nop_function") ))
#endif

void cutMotorsOff(void) WEAK_CUTMOTORSOFF;
void cutServosOff(void)WEAK_CUTSERVOSOFF;
void funnyAction(void) WEAK_FUNNYACTION;

#endif /* SRC_DISCO_POWER_H_ */
