/*
 * Telephone.h
 *
 * Created: 23/07/2025 18:05:14
 *  Author: peter
 */

#include "Compile.h"
#include <stdbool.h>

#ifndef TELEPHONE_H_
#define TELEPHONE_H_

#ifdef CODE_SECTION_TELEPHONE

//_______________________________________________________________________________________
// Function prototypes
//_______________________________________________________________________________________
//

void initTelephone();

//_______________________________________________________________________________________
// Some variables
//_______________________________________________________________________________________
//

short	cadenceTablePtr;
short	cadenceCount;
short	counter_20Hz;
bool	phase_20Hz;

#endif /*CODE_SECTION_TELEPHONE*/

#endif /* TELEPHONE_H_ */