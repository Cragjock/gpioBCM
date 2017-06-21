
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <stdint.h>   // for uint32_t - 32-bit unsigned integer

// GPIO_BASE is 0x20000000 on RPi models other than the RPi 2
#define GPIO_BASE       0x3F200000          // on the RPi 2
#define GPFSELn          0x00
#define GPSET0          0x1c                // from Figure 6-X
#define GPCLR0          0x28
#define GPLVL0          0x34
#define GPEDS0          0x40
#define GPHEN0          0X64
#define GPLEN0          0x70
#define LEVEL_MASK      0x10000             /// mask for bit 16
#define L_MASK(x)       (1<<x)              /// level mask for GPIO x

static volatile uint32_t *gpio;         // pointer to the gpio (*int)
static uint32_t led_level;
static uint16_t GPIO_input = 16;        /// hard coded for my test
static uint16_t GPIO_output = 26;       /// hard coded for my test

uint16_t GDFSELn[]={0,1,2,3,4,5};
enum G_direction {Ginput, Goutput} GPIO_direction;
enum G_function_set{input, output, alt_0, alt_1, alt_2, alt_3, alt_4, alt_5} GPIO_Function_set;
enum G_pin_output{Gset, Gclear} GPIO_pin_set;
enum G_Pin_level {HIGH, LOW} Pin_level;
enum G_event {high, low, rising, falling};

int GPIO_set_FSR(uint16_t GNumber, enum G_direction Gdir);           /// for GPFSELn
int GPIO_Pin_set_state(uint16_t GNumber, enum G_pin_output Gset);   /// for GPSETn and GPCLRn
int GPIO_Pin_level(uint16_t GNumber, enum G_Pin_level Glevel);       /// for GPLEVn
uint32_t GPIO_read(uint16_t GNumber, uint8_t g_reg);
uint32_t GPIO_write(uint16_t GNumber, uint8_t g_reg);
int GPIO_toggle(uint16_t GNumber, uint8_t g_reg);

int main()
{
    int fd, x;
    printf("Start of GPIO memory-manipulation test program.\n");

    if(getuid()!=0)
    {
        printf("You must run this program as root. Exiting.\n");
        return -EPERM;
    }

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
    {
        printf("Unable to open /dev/mem: %s\n", strerror(errno));
        return -EBUSY;
    }

   // get a pointer that points to the peripheral base for the GPIOs
    gpio = (uint32_t *) mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);
    printf("gpio after mmap call %x \n", gpio);
    if ((int32_t) gpio < 0)
    {
        printf("Memory mapping failed: %s\n", strerror(errno));
        return -EBUSY;
    }


   //  *(gpio + 1) = (*(gpio + 1) & ~(7 << 21) | (1 << 21));



    /// use bit index for the GPFSELn registers
    uint32_t FS_input_bit_index = 3*(GPIO_input % 10);
    printf("in bit index= %i\n", FS_input_bit_index);

    uint32_t FS_output_bit_index = 3*(GPIO_output % 10);
    printf("out bit index= %i\n", FS_output_bit_index);

    /// get an index to GPFSELn register
    uint32_t FSEL_index = GPIO_input/10;
    printf("FSEL_index input divide= %i\n",FSEL_index);

    FSEL_index = GPIO_output/10;
    printf("FSEL_index output divide= %i\n",FSEL_index);



    /// ====== set GPIO 16 as input
    //printf("before 16+1 %x \n", *(1+gpio));

    *(gpio + 1) = (*(gpio + 1) & ~(7 << 18) | (0 << 18));

    //printf("set 16 %x \n", gpio);
    //printf("after 16+1 %x \n", *(1+gpio));


   // set up the button GPIO FSEL27 mode = 000 at addr GPFSEL2 (0008)
   // both FSEL17 and FSEL27 are 21 bits in, but on different registers
   // *(gpio + 2) = (*(gpio + 2) & ~(7 << 21) | (0 << 21));


    ///  ===== set GPIO 26 as output
    //printf("before 16+2 %x \n", *(2+gpio));
    //*(gpio + 2) = (*(gpio + 2) & ~(7 << 18) | (1 << 18));
    //*(gpio + FSEL_index) = (*(gpio + FSEL_index) & ~(7 << FS_output_bit_index) | (1 << FS_output_bit_index));
    *(gpio + GDFSELn[(GPIO_output/10)]) = (*(gpio + FSEL_index) & ~(7 << FS_output_bit_index) | (1 << FS_output_bit_index));

    //printf("set 26 %x \n", gpio);
    //printf("after 16+2 %x \n", *(2+gpio));


   // writing the 000 is not necessary but is there for clarity

    do
    {
        // turn the LED on using the bit 17 on the GPSET0 register
        //*(gpio + (GPSET0/4)) = 1 << 17;
        /// turn on the LED, set GPIO 26 set n output ON
        //*(gpio + (GPSET0/4)) = 1 << 26;
        *(gpio + (GPSET0/4)) = (1 << GPIO_output);
        //printf("set LED on %x \n", *(gpio + (GPSET0/4)));

        //usleep(10);          // cannot use sleep as it is non-blocking

        for(x=0;x<50;x++){}  // blocking delay hack using a simple loop


        /// turn off the LED, set GPIO clear n to ON

        // *(gpio + (GPCLR0/4)) = 1 << 17;  // turn the LED off
        //*(gpio + (GPCLR0/4)) = 1 << 26;  // turn the LED off

        *(gpio + (GPCLR0/4)) = (1 << GPIO_output);  // turn the LED off
        printf("set LED off %x \n", *(gpio + (GPCLR0/4)));


        *(gpio + (GPHEN0/4)) = (0 << GPIO_output); // set event



        for(x=0;x<49;x++){}  // delay hack -- balanced for while()

        led_level = *(gpio+(GPLVL0/4));
        //printf("switch value = %x\n", (led_level));
        printf("switch value = %x\n", (led_level & L_MASK(GPIO_input)));
        uint32_t dododod= GPIO_read(GPIO_input, GPLVL0);
        dododod= GPIO_read(GPIO_output, GPLVL0);
        //printf("return value= %x\n",dododod);
        dododod =GPIO_read(GPIO_output, GPFSELn);
        dododod =GPIO_read(GPIO_input, GPFSELn);
        dododod =GPIO_read(GPIO_output, GPSET0);
        dododod =GPIO_read(GPIO_output, GPCLR0);
        dododod =GPIO_read(GPIO_input, GPSET0);
        dododod =GPIO_read(GPIO_input, GPCLR0);
        dododod =GPIO_read(GPIO_input, GPEDS0);
        dododod =GPIO_read(GPIO_output, GPEDS0);

       // if((led_level & LEVEL_MASK) == LEVEL_MASK)
       //     printf("in the IF value = %x\n", led_level);

        //led_level = (*(gpio+(GPLVL0/4)) & (1<<16));
        led_level = (*(gpio+(GPLVL0/4)) & (L_MASK(GPIO_input)));
        ///printf("while condition check %x \n", led_level);

    }

    //while((*(gpio+(GPLVL0/4))&(1<<27))==0); // only true if bit 27 high, typo from book == ??
    while((*(gpio+(GPLVL0/4)) & (1<<16)) !=0); // only true if bit 27 high

    *(gpio + (GPSET0/4)) = (1 << GPIO_output); /// turn on LED for test

    printf("Button was pressed - end of example program.\n");
    int mmm= munmap((void*)gpio, getpagesize());
    if(mmm != 0)
            printf("munmap failed");

    close(fd);

    return 0;
}


uint32_t GPIO_read(uint16_t GNumber, uint8_t g_reg)
{
    uint32_t rtn_value = -1;
    switch (g_reg)
    {

        case GPEDS0:
            {
                rtn_value = (*(gpio+(GPEDS0/4)) & (L_MASK(GNumber)));
                printf("at GPEDS0=pin %i value= %x\n",GNumber, rtn_value);
                if(rtn_value == L_MASK(GNumber))
                {
                    printf("event detected\n");
                    *(gpio + (GPEDS0/4)) = (1 << GNumber);
                }
                break;
            }

        case GPFSELn:
            {
                rtn_value = *(gpio + GDFSELn[(GNumber/10)]);
                printf("at GPFSELn=pin %i value= %x\n",GNumber, rtn_value);
                break;
            }

        case GPSET0:
            {
                rtn_value = *(gpio + (GPSET0/4));
                printf("at GPSET0=pin %i value= %x\n",GNumber, rtn_value);
                break;
            }
        case GPCLR0:
            {
                rtn_value = *(gpio + (GPCLR0/4));
                printf("at GPCLR0=pin %i value= %x\n",GNumber, rtn_value);
                break;
            }
        case GPLVL0:
            {
                rtn_value = (*(gpio+(GPLVL0/4)) & (L_MASK(GNumber)));
                printf("at GPLVL0=pin %i value= %x\n",GNumber, rtn_value);
                break;
            }
        default:
            printf("error on switch!\n");
    }

    return rtn_value;
}


