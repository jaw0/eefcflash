/*
  Copyright (c) 2016
  Author: Jeff Weisberg <jaw @ tcp4me.com>
  Created: 2016-Jun-19 16:50 (EDT)
  Function: program atmel sam eefc flash

*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdarg.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <poll.h>
#include <err.h>


// use -p to specify device
#define DEVFILE "/dev/cu.usbmodem2411"

#define TIMEOUT	5000

// addresses of registers on chip
#define CHIPID	0x400E0940	// chip id register
#define FLASH   0x00400000	// start of flash
#define EEFC    0x400e0c00	// eefc device registers
#define AIRCR 	0xE000ED0C	// arm cortex scb register (for reset)

// eefc register offsets
#define FMR	0
#define FCR	4
#define FSR	8
#define FRR	12

int devfd;
int pagesize;
int debugp = 0;

void
debug(const char *fmt, ...){
    va_list ap;

    if( !debugp ) return;

    fprintf(stderr, ">> ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
}

// read from device
int
ser_read(char* buffer, int len, int timeout){
    int totread = 0;

    while( totread < len ){
        struct pollfd pf[1];

        pf[0].fd = devfd;
        pf[0].events = POLLIN;
        pf[0].revents = 0;

        // timeout is msec
        int r = poll( pf, 1, timeout );
        // 0 => TO, -1 => error, else # fds

        if( r < 0 )
            return -1;
        if( r == 0 )
            return totread;

        if( pf[0].revents & POLLIN ){

            r = read(devfd, buffer + totread, len - totread);

            if (r < 0)
                return -1;
            totread += r;
        }
    }

    return totread;
}

// write to device
int
ser_write(const char* buffer, int len){

    int res = write(devfd, buffer, len);
    usleep(1000);
    return res;
}



// open device
void
open_dev(const char *file, int speed){
    struct termios options;

    devfd = open(file, O_RDWR);

    if( devfd < 0 ){
        err(-1, "cannot open '%s'", file);
    }

    if (tcgetattr(devfd, &options) == -1) {
        err(-1, "tcgetattr failed");
    }

    if( cfsetispeed(&options, speed) || cfsetospeed(&options, speed)){
        err(-1, "setspeed failed");
    }

    options.c_cflag |= (CLOCAL | CREAD);
    // No hardware flow control
    options.c_cflag &= ~CRTSCTS;

    // No software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Raw input
    options.c_iflag &= ~(BRKINT | ICRNL);
    options.c_lflag &= ~(ICANON | IEXTEN | ECHO | ECHOE | ISIG);

    // Raw output
    options.c_oflag &= ~OPOST;

    // No wait time
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(devfd, TCSANOW, &options)){
        err(-1, "tcsetattr failed");
    }

    ser_write("N#", 2);
    char buf[8];
    ser_read(buf, sizeof(buf), 100);
}

/****************************************************************/

// have SAMBA read a 32bit word
uint32_t
readw(uint32_t addr){
    char buf[32];

    snprintf(buf, sizeof(buf), "w%08X,4#", addr);
    int w = ser_write(buf, strlen(buf));

    uint32_t res;
    int r = ser_read((char*)&res, 4, TIMEOUT);
    if( r != 4 ) err(-1, "readw read failed");
    debug("readw [%x] = %x", addr, res);

    return res;
}

// have SAMBA write a 32bit word
void
writew(uint32_t addr, uint32_t val){
    char buf[32];

    snprintf(buf, sizeof(buf), "W%08X,%08X#", addr, val);
    int w = ser_write(buf, strlen(buf));
    debug("writew [%x] = %x", addr, val);
}

// there are all sorts of bugs/quirks with sam-ba read/write file
// instead, we just do everything word-by word. slow and steady.

void
readbuf(uint32_t addr, uint32_t *buf, int size){


    while( size > 0 ){
        uint32_t r = readw(addr);
        *buf = r;

        addr += 4;
        buf  ++;
        size -= 4;
    }
}

void
writebuf(uint32_t addr, const uint32_t *buf, int size){

    while( size > 0 ){
        writew(addr, *buf);
        addr += 4;
        buf  ++;
        size -= 4;
    }
}


/****************************************************************/

// wait for flash to become ready
void
wait_fsr(void){
    int n = 0;

    while(1){
        int sr = readw( EEFC + FSR );
        if( sr & 0xE ){
            fprintf(stderr, "FSR %x\n", sr);
            exit(1);
        }
        if( sr & 1 ) break;

        fprintf(stderr, "%c", "/-\\|"[n&3]);
        usleep(10000);
        fprintf(stderr, "\b");

        if( ++n > 10000000) err(-1, "timeout");
    }
}

// execute a flash command
void
send_cmd(int cmd, int arg){
    writew( EEFC + FCR, (0x5A << 24) | (arg << 8) | cmd );
}

/****************************************************************/

// I like to think there is a historical compatibility story here,
// and not something in the water at the atmel factory.

// these are from the datasheet

const int sramsize[] = {
    48, 192, 384, 6, 24, 4, 80, 160, 8, 16, 32, 64, 128, 256, 96, 512
};
const int nvpsize[] = {
    0, 8, 16, 32, 0, 64, 0, 128, 160, 256, 512, 0, 1024, 0, 2048, 0
};
const char *const archname[] = {
    "Cortex-M7", "ARM946ES", "ARM7TDMI", "Cortex-M3", "ARM920T", "ARM926EJS", "Cortex-A5", "Cortex-M4"
};

void
get_info(int show){

    int chip  = readw( CHIPID );

    int eproc = (chip>>5) & 7;
    int nvpsz = nvpsize[ (chip>>8) & 0xF ];
    int srsz  = sramsize[ (chip>>16) & 0xF ];

    send_cmd( 0, 0 );
    wait_fsr();

    int id   = readw( EEFC + FRR );
    int size = readw( EEFC + FRR );
    int page = readw( EEFC + FRR );
    int plan = readw( EEFC + FRR );
    int plsz = readw( EEFC + FRR );
    int lock = readw( EEFC + FRR );
    int lksz = readw( EEFC + FRR );

    pagesize = page;

    if( show ){
        printf("%-10s: %X\n",  "Chip ID",	chip);
        printf("%-10s: %s\n",  "Arch", 		archname[eproc] );
        printf("%-10s: %dk\n", "SRAM", 		srsz);
        printf("%-10s: %X\n",  "Flash ID", 	id);
        printf("%-10s: %dk\n", "Size", 		size/1024);
        printf("%-10s: %d\n",  "Page size", 	page);
        printf("%-10s: %d\n",  "Planes", 	plan);
        printf("%-10s: %d\n",  "Plane Size", 	plsz);
        printf("%-10s: %d\n",  "Locks", 	lock);
        printf("%-10s: %d\n",  "Lock Size", 	lksz);
        printf("\n");
    }

    // simple sanity check
    if( (page != 512) || (nvpsz*1024 != size) )
        err(-1, "unsupported device");

}

void
dump(uint32_t addr, int size){
    int i;
    char buf[16];
    size /= 4;

    while(size > 0){
        readbuf(addr, (void*)buf, 16);

        printf("%08X:", addr);
        for(i=0; i<16; i++){
            if( i && i%4 == 0 ) printf(" ");
            printf(" %02X", buf[i] & 0xFF );
        }
        printf("\n");

        addr += 16;
        size -= 4;
    }
}

// erase entire device
void
erase_all(void){
    fprintf(stderr, "erasing ");
    send_cmd( 5, 0 );
    wait_fsr();
    fprintf(stderr, " done\n");
}

// set the 'boot from flash' bit
void
set_boot(void){

    send_cmd( 0xB, 1 );
    wait_fsr();
}

void
reboot_device(void){

    // set reboot request bit in scb->aircr
    writew( AIRCR, 0x05FA0004 );
}


// program one page of flash
void
program(uint32_t addr, uint32_t *buf, int erased){

    // RSN - erase by sector/page if not already erased?
    debug("prog: %x", addr);
    wait_fsr();
    writebuf( addr, buf, pagesize );
    int page = (addr - FLASH) / pagesize;
    send_cmd( 1, page );
    wait_fsr();
}

// fill flash with test pattern
void
write_pattern(uint32_t addr, int size, int erased){
    char buf[512];
    uint32_t *lb = (void*)buf;
    int i;

    if( !size ) size = 512;
    for(i=0; i<512; i++) buf[i] = i;

    fprintf(stderr, "writing");

    while(size > 0){
        for(i=0; i<512/16; i++){
            lb[4 * i] = addr + 16 * i;
        }

        program(addr, (void*)buf, erased);
        addr += pagesize;
        size -= pagesize;
        fprintf(stderr, ".");
    }

    fprintf(stderr, " done\n");
}

void read_file(const char *file, int len){
    char *buf[512];

    int f = open(file, O_WRONLY | O_CREAT, 0666);
    if( f < 0 )
        err(-1, "open failed %s", file);

    uint32_t addr = FLASH;

    fprintf(stderr, "reading");

    while( len > 0 ){
        readbuf( addr, (void*)buf, pagesize );
        int wlen = (len > pagesize) ? pagesize : len;
        write(f, buf, wlen);

        addr += pagesize;
        len  -= pagesize;
        fprintf(stderr, ".");
    }

    fprintf(stderr, " done\n");
}

void
write_file(const char *file, int erased){
    char *buf[512];

    int f = open(file, O_RDONLY);
    if( f < 0 )
        err(-1, "open failed %s", file);

    uint32_t addr = FLASH;

    fprintf(stderr, "writing");
    sleep(1);

    while(1){
        memset(buf, 0xFF, 512);
        int r = read(f, buf, pagesize);
        if( r < 0 ) err(-1, "writefile read failed");
        if( r == 0 ) break;
        program(addr, (void*)buf, erased);
        addr += pagesize;
        fprintf(stderr, ".");
    }

    fprintf(stderr, " done\n");
}

void
verify_file(const char *file){
    char fbuf[512];
    char dbuf[512];

    int f = open(file, O_RDONLY);
    if( f < 0 )
        err(-1, "open failed %s", file);

    uint32_t addr = FLASH;
    int failed = 0;

    fprintf(stderr, "verifying");

    while(1){
        int r = read(f, fbuf, pagesize);
        if( r < 0  ) err(-1, "verify read failed");
        if( r == 0 ) break;

        readbuf(addr, (void*)dbuf, pagesize);

        int c = memcmp(fbuf, dbuf, r);

        if( c ){
            fprintf(stderr, "x");
            failed = 1;

            int i;
            for(i=0; i<r; i++){
                if( fbuf[i] != dbuf[i] ) fprintf(stderr, "ERR: [%x] file: %x != dev: %x\n", addr+i, fbuf[i] & 0xFF, dbuf[i] & 0xFF);
            }

        }else{
            fprintf(stderr, ".");
        }

        addr += pagesize;
    }

    fprintf(stderr, " done\n");
    printf("verify %s\n", failed ? "failed" : "passed");
    if( failed ) exit(1);
}


/****************************************************************/

void
help(void){

    printf(
        "usage: [opts] [file]\n"
        "    -b\t\tset boot flag\n"
        "    -d\t\tenable debugging\n"
        "    -e\t\terase entire flash\n"
        "    -i\t\tshow device info\n"
        "    -p dev\tset device port\n"
        "    -s speed\tset baud rate\n"
        "    -R\t\treboot the device\n"
        "    -r size\tread flash + save to file\n"
        "    -w\t\twrite file to flash\n"
        "    -v\t\tverify flash write\n"
        "    -x addr\texamine flash\n"
        "\nexample:\n"
        "    typical programming example, erase, write, verify, and run\n"
        "        -p /dev/tty.usbmodem.1234 -e -b -R -v -w blinky.bin\n"
        "    save flash to file:\n"
        "        -p /dev/tty.usbmodem.1234 -r 65536 image.bin\n"
        );
}


int
main(int argc, char **argv){

    extern char *optarg;
    extern int optind;
    const char *devfile = DEVFILE;
    int speed=115200;
    int readsize=0;
    int dumpat=0;
    int setboot=0;
    int erase=0;
    int showinfo=0;
    int dowrite=0;
    int doverify=0;
    int testat=0;
    int doreboot=0;
    int c;

     while( (c = getopt(argc, argv, "bdehip:Pr:Rs:t:vwx:")) != -1 ){
	 switch(c){
         case 'b':
             setboot = 1;
             break;
         case 'd':
             debugp = 1;
             break;
         case 'e':
             erase = 1;
             break;
         case 'h':
             help();
             exit(0);
             break;
         case 'i':
             showinfo = 1;
             break;
         case 'p':
             devfile = optarg;
             break;
         case 'R':
             doreboot = 1;
             break;
         case 'r':
             readsize = strtoul( optarg, 0, 0 );
             break;
         case 's':
             speed = atoi( optarg );
             break;
         case 'w':
             dowrite = 1;
             break;
         case 't':
             testat = strtoul( optarg, 0, 0 );
             break;
         case 'v':
             doverify = 1;
             break;
         case 'x':
             dumpat = strtoul( optarg, 0, 0 );
             break;
         case 'P':
             erase = dowrite = doverify = setboot = doreboot = showinfo = 1;
             break;

         }
     }
     argc -= optind;
     argv += optind;

     const char *file = 0;
     if( argc ) file = argv[0];

     open_dev( devfile, speed );

    get_info( showinfo );
    if( erase )    erase_all();

    if( file && readsize ) read_file( file, readsize );
    if( file && dowrite  ) write_file( file, erase );
    if( file && doverify ) verify_file( file );
    if( testat ) write_pattern( testat, readsize, erase );

    if( dumpat ){
        // -d addr -r size
        if( !readsize ) readsize = 256;
        dump( dumpat, readsize );
    }

    if( setboot )  set_boot();
    if( doreboot ) reboot_device();

    return 0;
}

