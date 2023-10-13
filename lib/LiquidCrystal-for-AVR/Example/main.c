/*
 * LiquidCrystal.c
 *
 * Created: 12/09/99 12:07:04 PM
 * Author : Mobin.byn
 */ 

#include <avr/io.h>
#include "LiquidCrystal_Config.h"
#include "LiquidCrystal.h"

int main(void)
{
	liquidCrystal_init();
	liquidCrystal_setCursor(0, 0);
	liquidCrystal_print("Hello world!");
	
    while (1);
}

