#![feature(custom_attribute)]
#![feature(alloc)]
#![feature(alloc_error_handler)]
#![no_main]
#![no_std]

#[macro_use]
extern crate alloc;
extern crate alloc_cortex_m;
extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
#[macro_use]
extern crate stm32f7;
#[macro_use]
extern crate stm32f7_discovery;
extern crate futures;
extern crate smoltcp;
extern crate spin;

use alloc_cortex_m::CortexMHeap;
use cortex_m::{asm, interrupt};
use rt::{entry, exception, ExceptionFrame};
use sh::hio::{self, HStdout};

use core::alloc::Layout as AllocLayout;
use core::fmt::Write;
use core::panic::PanicInfo;

use stm32f7_discovery::{
    ethernet,
    future_mutex::FutureMutex,
    gpio::{GpioPort, InputPin, OutputPin},
    i2c::I2C,
    init,
    interrupts::{self, InterruptRequest, Priority},
    lcd::{self, AudioWriter, Color, Framebuffer, Layer},
    random::Rng,
    sd,
    system_clock::{self, Hz},
    task_runtime, touch,
};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[exception]
fn SysTick() {
    system_clock::tick();
    // print a `.` every 500ms
    if system_clock::ticks() % 50 == 0 && lcd::stdout::is_initialized() {
        print!(".");
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn rust_oom(_: AllocLayout) -> ! {
    panic!("out of memory");
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    interrupt::disable();

    if lcd::stdout::is_initialized() {
        println!("{}", info);
    }

    if let Ok(mut hstdout) = hio::hstdout() {
        let _ = writeln!(hstdout, "{}", info);
    }

    // OK to fire a breakpoint here because we know the microcontroller is connected to a debugger
    asm::bkpt();

    loop {}
}

#[entry]
fn main() -> ! {
    loop {}
}
