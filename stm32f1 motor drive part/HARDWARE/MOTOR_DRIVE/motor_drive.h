#ifndef __MOTOR_DRIVE_H
#define __MOTOR_DRIVE_H	 
#include "sys.h"

void motor_drive_Init(void);

#define Left_X1 PEout(12)
#define Left_X2 PEout(11)
#define Left_X3 PEout(10)
#define Left_FR PEout(9)
#define Left_BK PEout(8)

#define Right_X1 PFout(11)
#define Right_X2 PFout(12)
#define Right_X3 PFout(13)
#define Right_FR PFout(14)
#define Right_BK PFout(15)


#endif
