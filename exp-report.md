# BUAA OS Raspberry Pi 3 - boot a small linux operating system

*（注：笔者MacOS系统）*

## 参考资料整理

- 一些驱动程序(Bare Metal)的资源: 

  https://github.com/bztsrc/raspi3-tutorial 

- 一些现有的可以提供参考的实现: 

  https://github.com/tonnylyz/Raspberry-Pi-3-JOS 
  https://github.com/Yradex/RaspberryPi3_OS 
  https://github.com/tonnylyz/RPI-EVO

- tutorials
	https://jsandler18.github.io/
	https://github.com/s-matyukevich/raspberry-pi-os
	


## 实验报告

### Lab0 - 实验环境搭建
- host操作系统：Ubuntu Linux，~~Win10(useless)~~, using Parallels Desktop
- 安装linaro内核编译器，应该可以在linux环境下运行。
	这里必须选择bare-metal（裸金属），因为我们将在小派上自行建立一个os，而不是利用现成的任何一个os。选binaries之后会有不同的操作系统对应的资源，mingw32是win的，还有Linux的i686和x86_64。可以在这两个平台上支持。
  
  - 否则下载gcc自行编译一个交叉编译器，在编译gcc的时候发生了很多bug……说是没有gmp, mpfr, mpc。但是我都下完了之后还是报没有……(而且也用--with-gmp=/usr/bin/.../lib这样的命令行参数标明了) 
  
  - 再或者，群里有人说可以利用LLVM（mac如果安装xcode或者command line tools已内置），对clang添加参数--target=aarch64-none-elf即可完成编译，但仍需下载binutils编译，configure时添加参数--target=aarch64-none-elf。
  
    这两步理论上完成了，但是在clang编译的时候，如果加上target参数会报stdio找不到。我觉得可能是binutils编译的时候没有改全局变量。
- 硬件仿真器qemu
  
  - 自行编译了。__路径不能有中文。__以下git clong获得qemu源代码。
```shell
git clone https://github.com/qemu/qemu.git
cd qemu
./configure --target-list=aarch64-softmmu --enable-modules --enable-tcg-interpreter --enable-
debug-tcg
make -jX
# X is the number of compile thread，多线程make可以加速，我写的X=4
```
​	可以用这个命令试一下好了没，会输出System started!并进入回显模式。
`qemu-system-aarch64 -M raspi3 -serial stdio -kernel kernel8.img`

- 安装minicom，用于连接小派
  - sudo apt install minicom



### Lab1 - boot

#### 点亮

##### 仿真器player：
`qemu-system-aarch64 -M raspi3 -serial stdio -kernel kernel8.img`
System start!

##### 实物player
- 烧写树莓派**官方给的镜像**的步骤

  1. 格式化sd卡，一般不需要操作

  2. 下载树莓派官方镜像

  3. 用Etcher把镜像烧到sd卡上

  4. 在sd卡上config.txt最后加一句enable_uart=1

  5. 把sd卡插入小派

  6. 在linux平台下，用minicom连上小派

  7. 小派亮绿灯表示在boot，并且一闪一闪，然后熄灭

  8. 登录用户名pi，密码raspberry

- 烧写实验用的镜像的步骤
  1. 格式化sd卡
  2. 把5个文件复制到sd卡
  3. 把sd卡插入小派
  4. minicom无反应，无回显之类的



#### to do list
##### 确定要移植的文件清单

not sure

##### 修改Makefile，配置交叉编译工具链
makefile 暂时不知道改哪。
交叉编译工具链，应该是include.mk，里面有一个
`CROSS_COMPILE   := /usr/local/gcc-linaro-7.2.1-2017.11-x86_64_aarch64-elf/bin/aarch64-elf-`
我觉得应该改成自己的aarch64-elf的路径。


##### 修改链接脚本 link script
暂时还没想好怎么改。这是目前的：
```cpp
ENTRY(_start)

SECTIONS
{
    . = 0x00080000;
    .text : {
        boot/start.o
        *(.text*)
    }
    .bss : {
        *(.bss*)
    }
    .rodata : {
        *(.rodata*)
    }
    .data : {
        *(.data*)
    }
}
```
其中一份参考的：
```cpp
SECTIONS
{
    . = 0x80000;
    init : {
        boot/init.o
        mm/mm_init.o
    }
    _init_end = ABSOLUTE(.);
    
    .text 0xFFFFFF0000000000 + _init_end : AT(_init_end) {
        boot/main.o
        *(.text*)
    }
    _text_end = .;
    
    .data : {
        *(.data*)
    }
    _data_end = .;
    
    .rodata : {
        *(.rodata*)
    }
    
    .bss : {
        *(.bss*)
    }
}
```

##### 实现UART驱动，替换字符输出相关的代码
cscore上下载的代码和指导书的代码不太一样。看来是下载的代码更加完整一些。
两份参考的代码也都不太一样。
目前改成这样：
```cpp
#define UART0_IBRD      ((volatile unsigned int*)(0x3F201024))
#define UART0_FBRD      ((volatile unsigned int*)(0x3F201028))
#define UART0_LCRH      ((volatile unsigned int*)(0x3F20102C))
#define UART0_CR        ((volatile unsigned int*)(0x3F201030))
#define UART0_ICR       ((volatile unsigned int*)(0x3F201044))
#define GPFSEL1         ((volatile unsigned int*)(0x3F200004))
#define GPPUD           ((volatile unsigned int*)(0x3F200094))
#define GPPUDCLK0       ((volatile unsigned int*)(0x3F200098))
#define UART0_DR        ((volatile unsigned int*)(0x3F201000))
#define UART0_FR        ((volatile unsigned int*)(0x3F201018))

void uart_init_boot() {

    register unsigned int r;

    /* initialize UART */
    *UART0_CR = 0;      // turn off UART0

    /* map UART0 to GPIO pins */
    r = *GPFSEL1;
    r &= ~((7 << 12) | (7 << 15)); // gpio14, gpio15
    r |= (4 << 12) | (4 << 15);    // alt0
    *GPFSEL1 = r;
    *GPPUD = 0;            // enable pins 14 and 15
    r = 150;
    while (r--) { asm volatile("nop"); }
    *GPPUDCLK0 = (1 << 14) | (1 << 15);
    r = 150;
    while (r--) { asm volatile("nop"); }
    *GPPUDCLK0 = 0;        // flush GPIO setup

    *UART0_ICR = 0x7FF;    // clear interrupts
    *UART0_IBRD = 2;       // 115200 baud
    *UART0_FBRD = 0xB;
    *UART0_LCRH = 0b11 << 5; // 8n1
    *UART0_CR = 0x301;     // enable Tx, Rx, FIFO
}

/**
 * Send a character
 */
void uart_send_boot(unsigned int c) {
    /* wait until we can send */
    do { asm volatile("nop"); } while (*UART0_FR & 0x20);
    /* write the character to the buffer */
    *UART0_DR = c;
}

/**
 * Receive a character
 */
char uart_getc_boot() {
    char r;
    /* wait until something is in the buffer */
    do { asm volatile("nop"); } while (*UART0_FR & 0x10);
    /* read it and return */
    r = (char)(*UART0_DR);
    /* convert carrige return to newline */
    return r == '\r' ? '\n' : r;
}

/**
 * Display a string
 */
void uart_puts(char *s) {
    while(*s) {
        /* convert newline to carrige return + newline */
        if(*s=='\n')
            uart_send('\r');
        uart_send(*s++);
    }
}

/**
 * Display a binary value in hexadecimal
 */
void uart_hex(unsigned int d) {
    unsigned int n;
    int c;
    for(c=28;c>=0;c-=4) {
        // get highest tetrad
        n=(d>>c)&0xF;
        // 0-9 => '0'-'9', 10-15 => 'A'-'F'
        n+=n>9?0x37:0x30;
        uart_send(n);
    }
}
```

##### 撰写启动用的汇编代码
应该就是指的是start.S这类。
已给的代码理解如下：

```cpp
.global _start		// _start is a name that should be visible from outside of the assembly file

_start:
// Find CPU-0 and jump to _start_master
// These lines will send three out of the four cores to halt
    mrs x0, mpidr_el1
    and x0, x0, #3 // & 0b11
    cbz x0, _start_master
1:  wfe // halt
    b 1b

_start_master:
// Set kernel stack, and grow downwards
// since our kernel runs from 0x1000000 and up
    ldr x0, =0x1000000
    mov sp, x0
    bl main	// This loads the address of the C function called kernel_main into a register and jumps to that location
    b 1b	// When the C function returns, it enters the halt procedure where it loops forever doing nothing.(1b↑)
```

> 但是指导书上有说，小派用的是四核心处理器，每个核心拥有自己一组寄存器，Cache 各级的共用情况不同，主存是共用的。按照推断，当内核镜像载入到主存后，四个核心都从同样的 PC 开始执行，若不做处理，在遇到对主存互斥的访问时会出现不可预知的问题。请寻找解决这一问题的方法。

我推测应该是在start.S这一步的时候加一些东西，改成对四核心处理器的启动。

一份参考代码：

```cpp
#include "sysconfig.h"

.globl _start
_start:
    mrs x0, mpidr_el1        // check core id, only one core is used.
    mov x1, #0xc1000000
    bic x0, x0, x1
    cbz x0, master
    b hang

master:
    ldr x0, =0x1000000
    mov sp, x0               // set el2 sp
    bl vm_init
    bl jump_to_el1

hang:
    b hang

.globl jump_to_el1
jump_to_el1:
    mrs x0, currentel        // check if already in el1
    cmp x0, #4
    beq 1f

    ldr x0, =0xffffff0001000000
    msr sp_el1, x0           // init the stack of el1

    // disable coprocessor traps
    mov x0, #0x33ff
    msr cptr_el2, x0         // disable coprocessor traps to el2
    msr hstr_el2, xzr        // disable coprocessor traps to el2
    mov x0, #3 << 20
    msr cpacr_el1, x0        // enable fp/simd at el1

    // initialize hcr_el2
    mov x0, #(1 << 31)
    msr hcr_el2, x0          // set el1 to 64 bit
    mov x0, #0x0800
    movk x0, #0x30d0, lsl #16
    msr sctlr_el1, x0

    // return to the el1_sp1 mode from el2
    mov x0, #0x5
    msr spsr_el2, x0         // el1_sp1 with DAIF = 0
    adr x0, 1f
    msr elr_el2, x0
    eret

1:
    mrs x0, sctlr_el1
    orr x0, x0, #(1 << 12)
    msr sctlr_el1, x0        // enable instruction cache

    ldr x0, =vectors
    msr vbar_el1, x0         // init exception vector table

    b el1_mmu_activate

.globl el1_mmu_activate
el1_mmu_activate:
    ldr x0, =0x04cc
    msr mair_el1, x0
    isb

    ldr x1, =0x01000000
    msr ttbr0_el1, x1
    msr ttbr1_el1, x1
    isb

    mrs x2, tcr_el1
    ldr x3, =0x70040ffbf
    bic x2, x2, x3

    ldr x3, =0x2bf183f18
    orr x2, x2, x3
    msr tcr_el1, x2
    isb

    ldr x5, =main
    mrs x3, sctlr_el1
    ldr x4, =0x80000
    bic x3, x3, x4

    ldr x4, =0x1005
    orr x3, x3, x4
    msr sctlr_el1, x3
    isb
    br x5
```



### Lab2 mmu
#### 确定需要引入的文件清单

#### 设置间接内存属性寄存器(MAIR_EL1) 

#### 设置翻译控制寄存器(TCR_EL1)
#### 准备内核用的页目录
#### 设置页表基地址寄存器(TTBR*_EL1) 
#### 设置系统控制寄存器(SCTLR_EL1)
#### 修改启动代码，从EL2回落到EL1
#### 改写链接脚本
