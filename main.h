  #define sig_byte3 0x1E
  #define sig_byte2 0x94
  #define sig_byte1 0x05
  
  #define devtype 0x78       // Mega 168 device code
  
  #define PAGESIZE 128       // Size in Bytes
  
  #ifdef _B128
    #define APP_PAGES ((2*8192 / PAGESIZE)- (2*128 / PAGESIZE )) 
    #define APP_END APP_PAGES * PAGESIZE 
  #endif
  #ifdef _B256
    #define APP_PAGES ((2*8192 / PAGESIZE)- (2*256 / PAGESIZE )) 
    #define APP_END APP_PAGES * PAGESIZE 
  #endif
  #ifdef _B512
    #define APP_PAGES ((2*8192 / PAGESIZE)- (2*512 / PAGESIZE )) 
    #define APP_END APP_PAGES * PAGESIZE 
  #endif
   #ifdef _B1024
    #define APP_PAGES ((2*8192 / PAGESIZE)- (2*1024 / PAGESIZE )) 
    #define APP_END APP_PAGES * PAGESIZE 
  #endif  

#define UART_RX_BUFFER_SIZE PAGESIZE

#define TRUE    1
#define FALSE   0
