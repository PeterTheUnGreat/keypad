/*
 * Menu.h
 *
 * Created: 29/03/2025 09:44:45
 *  Author: peter
 */ 


#ifndef MENU_H_
#define MENU_H_

#include <avr/pgmspace.h>


typedef struct  {
	const char			name[4];
	int					digits;
	const char			type;
	const unsigned int	address;
	unsigned char		flags;
} menuStruct;

//flags
#define		MENU_FWD	0	// Are we allowed to step forward?
#define		MENU_BK		1	// Are we allowed to step back?
#define		MENU_NUM	2	// Is it possible to enter a number?
#define		MENU_MENU	3	// this is a menu item
#define		MNEU_ENTRY	4	// Are we in the entry phase

int menuItem;									// Keeps track of which menu item we are on

// menu item types
#define		MENU_TYP_TIME	'T'		// Entering a time value
#define		MENU_TYP_BCD	'D'		// Entering a BCD value
#define		MENU_TYP_BYTE	'B'		// Entering a byte value
#define		MENU_TYP_EXIT	'E'		// Leave the settings menu
#define		MENU_TYP_CODE	'C'		// Entering a code to compare
#define		MENU_TYP_TEST	'X'		// Just for debugging
#define		MENU_TYPE_NULL	'N'		// No action

menuStruct menuLocal;

#endif /* MENU_H_ */