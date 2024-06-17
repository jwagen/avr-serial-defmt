#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(asm_experimental_arch)]
#![feature(asm_const)]

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

//use panic_halt as _;

use avr_serial_defmt::logger_init;
use defmt::{debug, info, trace};

#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        led(true);
        avr_device::asm::delay_cycles(200_000);
        led(false);
        avr_device::asm::delay_cycles(1000_000);
    }
}
fn led(on: bool) {
    let p = unsafe { avr_device::avr64du32::Peripherals::steal() };
    if on {
        p.PORTF.dirset.write(|w| w.pf2().set_bit());
    } else {
        p.PORTF.dirclr.write(|w| w.pf2().set_bit());
    }
}

fn ccp_write<const REG_ADR: usize, const CCP_VALUE: u8>(data: u8) {
    const ccpreg: u8 = 0x34;
    unsafe {
        core::arch::asm!(
            "out {ccpreg},{ccp_value}",
            "sts {data_reg}, {value}",
            ccpreg = const ccpreg ,
            ccp_value = in (reg) CCP_VALUE,
            data_reg = const REG_ADR,
            value = in(reg) data,
        )
    }
}

fn io_protect_write<const REGISTER: usize>(data: u8) {
    ccp_write::<REGISTER, 0xd8>(data);
}

#[avr_device::entry]
fn main() -> ! {
    let p = unsafe { avr_device::avr64du32::Peripherals::steal() };
    p.PORTF.dirset.write(|w| w.pf2().set_bit());

    //let log = SerialLogger::new();
    //io_protect_write::<{avr_device::avr64du32::CLKCTRL::PTR as usize}>(0x09);
    ccp_write::<0x100, 0x34>(0u8);
    logger_init(4_000_000);

    //SerialLogger::write(b"Init done normal\n");
    avr_device::asm::delay_cycles(100_0000);
    debug!("defmt Init done\n");
    trace!("defmt Init done\n");
    info!("defmt Init done\n");

    loop {
        avr_device::asm::delay_cycles(100_0000);
        led(false);

        avr_device::asm::delay_cycles(100_0000);
        info!("TEST LOOPING");
        //SerialLogger::write(b"123456789ABCDEF\n");
        led(true);
    }
}
