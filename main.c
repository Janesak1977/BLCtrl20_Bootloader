/*****************************************************************************
*
* Copyright (C) 1996-1998 Atmel Corporation
*
* File          : main.c
* Created       : 16-jun-99
* Last Modified : 3-dec-1999  (mt: ??)
* Author(s)     : BBrandal, PKastnes, ARodland, LHM
*
* Description   : This Program allows an AVR with bootloader capabilities to 
*                 Read/write its own Flash/EEprom. 
* 
*
****************************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>


// OSCCAL value in EEPROM @ address
#define	OSCCAL_EE_Value_Addr	0x100		 

//Bootloader USART Baudrate											 
#define BAUD_RATE	57600

/* Select Boot Size (select one, comment out the others) */
// #define _B128
//#define _B256  
//#define _B512 
#define _B1024

#define FlashAddr_BLCtrlData	0x37F0

#include "main.h"

unsigned char BLCtrlData[16] PROGMEM = {0x00, 0x05, 0x01, 0x1E, 0x94, 0x06, 0x01, 0x02, 0x33, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDF} ;

unsigned char gBuffer[UART_RX_BUFFER_SIZE+2]; 			// Added 2 bytes for CRC

void sendchar(char);
char recchar(void);

// cbA void USART_Init(unsigned int );
unsigned char BufferLoad(unsigned int , unsigned char ) ;
void SetOSCCAL(void) ;
void Delay(unsigned int) ;
void AppPreSetup(void);
unsigned int CheckIfProgrammed(void);

void write_page (unsigned int adr, unsigned char function);
void write_lock_bits (unsigned char val);
unsigned int read_program_memory (unsigned int,unsigned char);
void fill_temp_buffer (unsigned int data,unsigned int adr);


unsigned int address;
unsigned char device;


int main(void)
{
		unsigned int tempi;
    char val;
    char OK = 0;
		void (*app_start)(void) = 0x0000;
		void (*boot_start) (void) = 0x1C00;
		
				    
		cli();
    MCUCR = (1<<IVCE);       
    MCUCR = (1<<IVSEL);             //move interruptvectors to the Boot sector
    wdt_reset();
    MCUSR = 0;
    
    WDTCSR |= _BV(WDCE) | _BV(WDE);
		WDTCSR = 0;

    SetOSCCAL();   // Set the OSCCAL Register

    // Enable 2x speed
    UCSR0A = (1<<U2X0);
    
		// Set Baudrate
    UBRR0H = (F_CPU/(BAUD_RATE*8L)-1) >> 8;
    UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*8L)-1); 

    // Enable receiver and transmitter
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);

    // Async. mode, 8N1
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
  
            
    TCCR1B = _BV(CS12) | _BV(CS10); // Start Timer1, Divider 1024 (128us)
    
    TIFR1 = _BV(TOV1); 	// Clear TOV1 flag
    TCNT1 = -(15625);		// 2 seconds to overflow (15625*128us)
    
    
    while(!(TIFR1 & _BV(TOV1)) && !OK)
    {
			if (UCSR0A & _BV(RXC0))					// Received char?
			{
				val = UDR0;
				
				if (val==0x1B)              // Is it 0x1B?
					gBuffer[0]=val;
					
				else if (val==0xAA)
				{
					if (gBuffer[0]==0x1B)
					{
						sendchar('M');					// Report signon
						sendchar('K');
						sendchar('B');
						sendchar('L');
						OK = 1;
					}
				  else
				  	gBuffer[0]=0x00;
				}
			}	
		}		//repeat until TOV1 is set
		
		TCCR1B = 0;					// stop Timer1
		TIFR1 = _BV(TOV1); 	// Clear TOV1 flag
		
		if (OK!=1)			//Was Init char received?
		{
			if (CheckIfProgrammed())		//1-App programmed, 0-not programmed
			{
		  	AppPreSetup();
		  	MCUCR = 0x01;       
    		MCUCR = 0x00;             //move interruptvectors to the App sector
				app_start();
  		}
  		else
  		{
  			sendchar(0x0D);          
  			sendchar(0x0A);
  			sendchar('V');
  			sendchar('0');
  			sendchar('.');
  			sendchar('5');
  			sendchar('b');
  			sendchar(':');
  			sendchar('M');
  			sendchar('K');
  			sendchar('B');
  			sendchar('L');
  			boot_start();					// Restart bootloader
			}
		}	
	
	for(;;)
	{
		val=recchar();

		if (val=='a')                        //Autoincrement?
			sendchar('Y');		    						//Autoincrement is quicker

		else if (val=='A')                   //write address 
		{
			address=recchar();                //read address 8 MSB
			address=(address<<8)|recchar();
			sendchar('\r');
		}
		
		else if (val=='b')
		{																									// Buffer load support
			sendchar('Y');																	// Report buffer load supported
			sendchar((UART_RX_BUFFER_SIZE >> 8) & 0xFF);		// Report buffer size in bytes
			sendchar(UART_RX_BUFFER_SIZE & 0xFF);
		}

		else if (val=='B')																	// Start buffer load
		{
			tempi = recchar() << 8;													// Load high byte of buffersize
			tempi |= recchar();															// Load low byte of buffersize
			val = recchar();																// Load memory type ('E' or 'F')
			sendchar (BufferLoad(tempi,val));								// Start downloading of buffer
		}
				
		else if (val=='c')                   // ????
		{       
			sendchar(0xB5);
			sendchar(0xCF);	
		}
        
		else if (val=='e')                   //Chip erase 
		{
			if (device == devtype)
			{
			
				for(address=0;address < APP_END;address += PAGESIZE)    //Application section = 60 pages
				{
					write_page(address,(1<<PGERS) + (1<<SPMEN));    			//Perform page erase
					write_page(address,(1<<RWWSRE) + (1<<SPMEN));   			//Re-enable the RWW section
				}
			}
			write_page(address,(1<<RWWSRE) + (1<<SPMEN));   					//Re-enable the RWW section
			
			for(unsigned int i=0;i<128;i++)
				gBuffer[i] = 0xFF;					//Fillup gBuffer with 0xFF
			for(unsigned int i=0;i<16;i++)
				gBuffer[0x70+i] = pgm_read_byte(&(BLCtrlData[i]));	//Copy 16 bytes of Data to gBuffer
			
			unsigned int adr = 0x3780;
			unsigned int size = 128;
			unsigned int cnt=0;
			unsigned int data;
							
			do
			{
				data = gBuffer[cnt++];
				data |= (gBuffer[cnt++]<<8);
				fill_temp_buffer(data,adr);			//call asm routine. 
				adr=adr+2;  						// Select next word in memory
				size -= 2;									// Reduce number of bytes to write by two
	    
			} while(size);
			
			write_page(0x3780,0x05);						// Program page contents
			write_page(0x3780,(1<<RWWSRE) + (1<<SPMEN));	//Re-enable the RWW section
			 
			
			
			
			sendchar('\r');        
    }

		else if(val=='E')                   //Exit upgrade
		{
			sendchar('\r');
			boot_start();
		}

		else if(val=='t')       // Return programmer type 
		{
			sendchar(devtype);
			sendchar(1);
		}
       
		else if (val=='T')			//set device/programmer type in bootloader ??
		{
			device = recchar();
			sendchar('\r');
		}
        
		else if (val==0xAA)                  // Return software identifier 
		{
			sendchar('M');
			sendchar('K');
			sendchar('B');
			sendchar('L');
		}                
        
		else if (val=='V')                  // Return Software Version
		{
			sendchar('0'); 
			sendchar('5');
			sendchar('b');
		}	               

		else if(val!=0x1b)                  // if not esc
		{
			sendchar('?');
		}
	}
	return 0;
}


void sendchar(char data)
{
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = data;
	while(!(UCSR0A & _BV(TXC0)));
	UCSR0A |= _BV(TXC0);                             //delete TXCflag
}


char recchar(void)
{   
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}


unsigned char BufferLoad(unsigned int size, unsigned char mem)
{
	int data, tempaddress, cnt;
	
	// store values to be programmed in temporary buffer
	for (cnt=0; cnt<UART_RX_BUFFER_SIZE+2; cnt++)			// +2 bytes for CRC
	{
		if (cnt<size+2)										// +2 bytes for CRC
			gBuffer[cnt]=recchar();
		else
			gBuffer[cnt]=0xFF;
	}
	cnt=0;
	
	tempaddress = address;					// Store address in page
	
	if (device == devtype)
	{
		if (mem == 'F')
		{
			do
			{
				data = gBuffer[cnt++];
				data |= (gBuffer[cnt++]<<8);
				fill_temp_buffer(data,address);			//call asm routine. 
				address=address+2;  						// Select next word in memory
				size -= 2;									// Reduce number of bytes to write by two
	    
			} while(size);									// Loop until all bytes written

			tempaddress &= 0xFF80;							//Ensure the address points to the first byte in the page
			write_page(tempaddress,0x05);					// Program page contents

			write_page(tempaddress,(1<<RWWSRE) + (1<<SPMEN));		//Re-enable the RWW section
			if (address != (address & 0xFF80))
			{
				address &= 0xFF80;
				address += PAGESIZE;
			}									// End FLASH
		}
		else
			return 0;
			
    return '\r';						// Report programming OK
	}
	return 0;								// Report programming failed
}


void SetOSCCAL(void)
{
	unsigned char EEOSCCalVal;
	
	EEOSCCalVal = eeprom_read_byte((uint8_t *) OSCCAL_EE_Value_Addr);
	if (EEOSCCalVal>=0x70 && EEOSCCalVal<=0xAF)
  {
    OSCCAL = EEOSCCalVal;
  }
  
}


void Delay(unsigned int millisec)
{
	unsigned char i;
    
	while (millisec--)
	{
		for (i=0; i<125; i++)
		{
			asm volatile (""::);
		}
	}
}

void AppPreSetup(void)
{
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	UCSR0A = 0x22;
	UCSR0B = 0x18;
	UCSR0C = 0x06;
	TIMSK0 = 0x01;
	TCCR0B = 0x02;
	TIFR0 = 0x07;
	SMCR = 0x00;
	ACSR = 0x00;
}

unsigned int CheckIfProgrammed(void)
{
	unsigned int data = pgm_read_byte_near(0x0080);
	
	if (data==0xFF)
		return 0;
	else
		return 1;
}


//
//  Low-level bootloader routines to replace "assembly.s90"
//  from the original ATMEL code. 
// 
//  See avr-libc's boot-module for more information
//  adapted to the BF-"system" (this is based on avr-libc 1.0).
//  Thanks to Eric B. Weddington author of boot.h.
//
//  1/2004 Martin Thomas, Kaiserslautern, Germany
//

#include <avr/io.h>

#define my_eeprom_is_ready() bit_is_clear(EECR, EEPE)

// There are some #ifdefs in avr/boot.h V1.0 that do not "know"
// about the ATmega169. So the functions have been copied here 
// and small modifications have been done to avoid problems.
// Be aware that there are some definitions from boot.h used 
// here which may change in the future (those starting with 
// two underlines). Because of this a copy of avr/boot.h from
// avr-libc 1.0 is included here to avoid conflicts (hopefully)
//#include <avr/boot.h>
#include "avr_libc_1_0_boot.h"

// #define _ATMEGA169
#define _ATMEGA16
#include "main.h"

// from avr/boot.h
// added "func" parameter and spm-busy check at end
#define _boot_page_write_alternate_bf(address,func)    \
({                                               \
    boot_spm_busy_wait();                        \
    while(!my_eeprom_is_ready());                   \
    __asm__ __volatile__                         \
    (                                            \
        "movw r30, %2\n\t"                       \
        "sts %0, %1\n\t"                         \
        "spm\n\t"                                \
        ".word 0xffff\n\t"                       \
        "nop\n\t"                                \
        : "=m" (__SPM_REG)                       \
        : "r" ((unsigned char)func),             \
          "r" ((unsigned short)address)          \
        : "r30", "r31", "r0"                     \
    );                                           \
    boot_spm_busy_wait();                        \
})

// from avr/boot.h 
// added spm-busy check at end
#define _boot_page_fill_alternate_bf(address, data)\
({                                               \
    boot_spm_busy_wait();                        \
    while(!my_eeprom_is_ready());                   \
    __asm__ __volatile__                         \
    (                                            \
        "movw  r0, %3\n\t"                       \
        "movw r30, %2\n\t"                       \
        "sts %0, %1\n\t"                         \
        "spm\n\t"                                \
        ".word 0xffff\n\t"                       \
        "nop\n\t"                                \
        "clr  r1\n\t"                            \
        : "=m" (__SPM_REG)                       \
        : "r" ((unsigned char)__BOOT_PAGE_FILL), \
          "r" ((unsigned short)address),         \
          "r" ((unsigned short)data)             \
        : "r0", "r30", "r31"                     \
    );                                           \
    boot_spm_busy_wait();                        \
})

// from avr/boot.h
// added spm-busy check at end, removed lockbit-masking, 
// removed r30/r31 init
#define _boot_lock_bits_set_alternate_bf(lock_bits)        \
({                                                         \
    boot_spm_busy_wait();                                  \
    while(!my_eeprom_is_ready());                             \
    __asm__ __volatile__                                   \
    (                                                      \
        "mov r0, %2\n\t"                                   \
        "sts %0, %1\n\t"                                   \
        "spm\n\t"                                          \
        ".word 0xffff\n\t"                                 \
        "nop\n\t"                                          \
        : "=m" (__SPM_REG)                                 \
        : "r" ((unsigned char)__BOOT_LOCK_BITS_SET),       \
          "r" ((unsigned char) lock_bits)                  \
        : "r0"                                             \
    );                                                     \
    boot_spm_busy_wait();                                  \
})

void write_page (unsigned int adr, unsigned char function)
{
	_boot_page_write_alternate_bf(adr,function);
}

void fill_temp_buffer (unsigned int data,unsigned int adr)
{
	_boot_page_fill_alternate_bf(adr, data);
}

unsigned int read_program_memory(unsigned int adr ,unsigned char param)
// to read lockbits give param=0x09, if param!=0 it is written to SPMCSR
// this is a "translation" from the original code to gcc-inline-assembler
{
	unsigned int retval;
	
	boot_spm_busy_wait();
	asm volatile
	(
		"movw r30, %3\n\t"
		"sbrc %2,0x00\n\t"
		"sts %0, %2\n\t"
		#ifdef LARGE_MEMORY  // If large memory (>64K) ELPM is needed to use RAMPZ
		"elpm\n\t"           // read LSB
		#else
		"lpm\n\t"            // R0 is now the LSB of the return value
		#endif
		"mov %A1,r0\n\t"
		"inc r30\n\t"
		#ifdef LARGE_MEMORY  // If large memory (>64K) ELPM is needed to use RAMPZ        
		"elpm\n\t"           // read MSB
		#else
		"lpm\n\t"            // R0 is now the MSB of the return value
		#endif
		"mov %B1,r0\n\t"  
		: "=m" (__SPM_REG),
		  "=r" (retval)
		: "r" ((unsigned char)param),
		  "r" ((unsigned short)adr)
		: "r30", "r31", "r0"
	);
	return retval;
}

void write_lock_bits (unsigned char val)
{
	_boot_lock_bits_set_alternate_bf(val);
}

