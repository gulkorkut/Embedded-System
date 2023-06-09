#include "stm32f10x.h"
#include<stdio.h>

int main (void) {
	
	//------------------------------
	//      init the micro controller
	//------------------------------
	//enable the GPIO clock for port GPIOA
	RCC->APB2ENR |= 0x0004; // ...000100
	
	
	//Or we could use a more readable way:
	//RCC->APB2ENR |= 1<<2; // This ORs the 2nd bit (Note that bit numbers start from 0)
	
	//Let's set PA03 pin as output
	//We need to modify GPIOA_CRL register. To make it output with max speed 2Mhz:
	//Mode3[1:0] should be 10, CR3[1:0] might be 00, which is Push/Pull output.
	//Therefore, GPIOA_CRL[15:12] bits should be 0b0010=0x2 (Reference manual)
	//Note that we cannot use OR. We exacly want 0b0010.
	//Reset value of GPIOA is 0x4444 4444. Each 4 corresponds a pair of (Mode,CR)
	//So, the value to be loaded into the reg is 0x4444 2444
	GPIOA->CRL = 0x44442444;	//Leave every pin in reset mode except pin1 and 2. It is 
														//set to PushPull output at max 2Mhz speed.
				
														
	//Or for a more readable but slow way we could do:
	//GPIOA->CRL = (GPIOA->CRL & ~(0xFF<<12)) | 2<<12;
	//now we are all set to go


	GPIOA->ODR=0x00000000; // pins are turned off
	//forever do...
	int i;
	int counter = 0;
	for(;;){
		
		
		
		for(i=0;i<0x100000;i++){
		}
		
		
		for(int j=0; j<counter; j++) {
			
			GPIOA->ODR = 0x00000008; // active
			// wait:
			for(i=0;i<0x100000;i++){
			}
			
			GPIOA->ODR=0x00000000; // pins are turned off
			for(i=0;i<0x100000;i++){
			}
			for(i=0;i<0x100000;i++){
			}
		}
		
		
		if (counter < 3) {
			counter ++;
		}else {
			counter = 0;
			GPIOA->ODR=0x00000000;
		}

	}//for	
	
	return 0;
}