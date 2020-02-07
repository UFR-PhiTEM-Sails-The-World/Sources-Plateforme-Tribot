/******************************************************************************/
/* I2C_STM32.c: STM32 low level I2C routines */
/******************************************************************************/
/* I2C1 simple library for STM32F4 processor. Needs heavy revision specially  */
/* Status register reading order                                                                                            */
/******************************************************************************/

#include "I2C.h"

/************************ Local auxiliary functions ***************************/

/*******************************************************************************
* I2C communication status *
* Parameter: *
* Return: status *
*******************************************************************************/

//static __inline unsigned int I2C_sr (void) {
static unsigned int I2C_sr (void) {
unsigned int sr;

sr = I2C1->SR1;
sr |= (I2C1->SR2 << 16);
return (sr);
}

/************************ Exported functions **********************************/


/*******************************************************************************
* Initialize I2C interface in master mode *
* Parameter: *
* Return: *
*******************************************************************************/

void I2C_Init(void) {
        //SCL sur PB8 et SDA sur PB9 ->I2C1, pas de pull-up externe, à configurer donc

    // Configuration des E/S
    RCC->AHB1ENR |= 1<<1;                       // Activation de l'horloge sur le port B
    GPIOB->MODER |= (1<<17)|(1<<19);    // PB8 et PB9 en mode Alternate function
    GPIOB->MODER &= ~((1<<16)|(1<<18));   // PB8 et PB9 en mode Alternate function (MODER8[1:0]=MODER9[1:0]="10")
    GPIOB->OTYPER |= (1<<8)|(1<<9);     // Sortie en drain ouvert sur PB8 et PB9
    GPIOB->PUPDR |= (1<<16)|(1<<18);
    GPIOB->PUPDR &= ~( (1<<17)|(1<<19) ); // Pull-up sur PB8 et PB9.
    GPIOB->AFR[1] |= (4<<0)|(4<<4);     // Fonction alternative AF4 sur PB8 et PB9
    GPIOB->AFR[1] &= ~((11<<0)|(11<<4));// Fonction alternative AF4 sur PB8 et PB9 (AFRH8[3:0]=AFRH9[3:0]="0100"

    // Configuration de l'I2C
    RCC->APB1ENR |= 1<<21;                      // Activation de l'horloge pour le périphérique I2C1
    I2C1->CR2 = 8;                                      // Horloge de fonctionnement du périphérique I2C = 8MhZ (FREQ[5..0]="001000")(exemple fourni dans la doc au niveau du registre CCR pour SCL à 100kHz)
    I2C1->CCR &= ~(3<<14);                      // Mode standard, rapport cyclique 1/2
    I2C1->CCR |= 40;                                    // Prédiviseur 40 pour la fréquence de communication 100kHz=8Mhz/(2*40) 
    I2C1->CR1|=1<<10;                               // Activation de l'acquittement apres réception d'un octet
    I2C1->CR1|=1;                                       // Activation du périphérique I2C1 une fois la config terminée
    
//  I2C1->OAR1 = 0x4033;  //reserved Bit=1 (see doc) & Own address = 33
}




/*******************************************************************************
* Generate start condition on I2C bus *
* Parameter: *
* Return: *
*******************************************************************************/

void I2C_Start (void) {
     __IO uint32_t i2c1SR1 = 0;
       unsigned int tmp;
    I2C1->CR1 |= (1<<8); //start genneration when bus free
//while (!(I2C1->SR1& 0x0001));
  i2c1SR1 = (uint32_t)I2C1 + 0x14; // I2C1-SR1
  do {
        tmp = *(__IO uint32_t *)i2c1SR1;
    }
    while (!(tmp&0x1));
//  I2C1->DR = 0x94;
    
}

/*******************************************************************************
* Generate stop condition on I2C bus *
* Parameter: *
* Return: *
*******************************************************************************/

void I2C_Stop (void) {

I2C1->CR1 |= 0x0200;
while (I2C_sr() & 0x00020000); /* Wait until BUSY bit reset */
}
/*******************************************************************************
* Write address on I2C interface *
* Parameter: adr: address to be written *
* Return: *
*******************************************************************************/

void I2C_Addr (unsigned char adr) {
    I2C1->DR = adr;
  while (!(I2C_sr() & 0x0002)); //Addr sent
}
/*******************************************************************************
* Write a byte to I2C interface *
* Parameter: c: data to be written *
* Return: *
*******************************************************************************/

void I2C_Write (unsigned char c) {

I2C1->DR = c;
while (!(I2C_sr() & 0x00000004)); /* Wait until BTF bit set */
}
/*******************************************************************************
* Read a byte from I2C interface *
* Parameter: *
* Return: read data *
*******************************************************************************/

unsigned char I2C_Read (int ack) {

/* Enable/disable Master acknowledge */
if (ack) I2C1->CR1 |= 0x0400;
else I2C1->CR1 &= ~0x0400;

while (!(I2C_sr() & 0x00000040)); /* Wait until RxNE bit set */
return (I2C1->DR);
}

/******************************************************************************/

unsigned char I2C_getbyte(unsigned char address, unsigned char cmd) {
unsigned char uc;
I2C_Start(); // Initial Start bit sequence
I2C_Addr(address); // Address I2C Device. (Base address is Write Address)
I2C_Write(cmd); // Transfer Command to I2C Device (Register to be Read)
I2C_Start(); // Repeated start bit sequence
I2C_Addr(address+1); // Address I2C Device. (Base address + 1 is Read Address)
uc = I2C_Read(0); // Read 1 byte without Acknowledge
I2C_Stop(); // Stop I2C transfer
return( uc );
}

unsigned short int I2C_getword(unsigned char address, unsigned char cmd) {
unsigned short int uw;
//unsigned short int uw2;
I2C_Start(); // Initial Start bit sequence
I2C_Addr(address); // Address I2C Device. (Base address is Write Address)
I2C_Write(cmd); // Transfer Command to I2C Device (Register to be Read)
I2C_Start(); // Repeated start bit sequence
I2C_Addr(address+1); // Address I2C Device. (Base address + 1 is Read Address)
uw = I2C_Read(1) << 8; // Read MSB without Acknowledge
uw |= I2C_Read(0); // Read LSB with Acknowledge
I2C_Stop(); // Stop I2C transfer
return( uw );
}
void I2C_putbyte(unsigned char address, unsigned char cmd, unsigned char data) {
I2C_Start(); // Initial Start bit sequence
I2C_Addr(address); // Address I2C Device. (Base address is Write Address)
I2C_Write(cmd); // Transfer Command to I2C Device (Register to be Read)
I2C_Write(data); // Transfer Data to I2C device
I2C_Stop(); // Stop I2C transfer
}

int I2C_getbytearray(unsigned char address, unsigned char cmd, int number, unsigned char *data) {
int count;
I2C_Start(); // Initial Start bit sequence
I2C_Addr(address); // Address I2C Device. (Base address is Write Address)
I2C_Write(cmd); // Transfer Command to I2C Device (Register to be Read)
I2C_Start(); // Repeated start bit sequence
I2C_Addr(address+1); // Address I2C Device. (Base address + 1 is Read Address)
// Read number - 1 bytes with Acknowledge
for ( count=0; count < number - 2; count++ ) {
data[count] = I2C_Read(1); // Read with Acknowledge
}
data[count] = I2C_Read(0); // Last byte without Acknowledge
I2C_Stop(); // Stop I2C transfer
return( count+1 );
}
