#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

//use panic_halt as _;

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

#[avr_device::entry]
fn main() -> ! {
    let p = unsafe { avr_device::avr64du32::Peripherals::steal() };
    p.PORTF.dirset.write(|w| w.pf2().set_bit());

    //let log = SerialLogger::new();
    unsafe{LOGGER.init()};

    //SerialLogger::write(b"Init done normal\n");
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

static mut LOGGER: SerialLogger = SerialLogger::new();
struct SerialLogger {
    buffer: CircularBuffer<200>,
}

static mut BUFFER: CircularBuffer<200> = CircularBuffer::new();
impl SerialLogger {
    pub const fn new() -> SerialLogger {
        Self {
            buffer: CircularBuffer::new()
        }
    }
    fn init(&mut self) {
        let p = unsafe { avr_device::avr64du32::Peripherals::steal() };
        p.PORTMUX.usartroutea.write(|w| w.usart1().alt2());
        p.USART1
            .baud
            .write(|w| w.bits(const { ((64u32 * 4_000_000u32) / (16 * 115200)) as u16 }));
        p.PORTD.dirset.write(|w| w.pd6().set_bit());
        p.USART1
            .ctrlb
            .write(|w| w.txen().set_bit().rxen().set_bit());
        unsafe { avr_device::interrupt::enable() };
    }

    pub fn write(data: &[u8]) {
        avr_device::interrupt::free(|_| unsafe {
            let p = avr_device::avr64du32::Peripherals::steal();
            LOGGER.buffer.write(data);
            // Enable interrupt in case buffer is completly sendt
            p.USART1.ctrla.write(|w| w.dreie().set_bit());
        })
    }
}

#[avr_device::interrupt(avr64du32)]
fn USART1_DRE() {
    unsafe {
        let p = avr_device::avr64du32::Peripherals::steal();
        if let Some(data) = LOGGER.buffer.get_byte() {
            p.USART1.txdatal.write(|w| w.bits(data));
        } else {
            // Disable interrupt if no more data
            p.USART1.ctrla.write(|w| w.dreie().clear_bit());
        }
    }
}

static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static TAKEN: AtomicBool = AtomicBool::new(false);
static mut CS_RESTORE: critical_section::RestoreState = critical_section::RestoreState::invalid();

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let restore = unsafe { critical_section::acquire() };

        if TAKEN.load(Ordering::Relaxed) {
            defmt::panic!("defmt logger taken reentrantly");
        }

        TAKEN.store(true, Ordering::Relaxed);

        unsafe {
            CS_RESTORE = restore;
        }

        unsafe { ENCODER.start_frame(SerialLogger::write) }
    }


    unsafe fn release() {
        ENCODER.end_frame(SerialLogger::write);
        TAKEN.store(false, Ordering::Relaxed);

        let restore = CS_RESTORE;
        critical_section::release(restore);
    }

    unsafe fn write(bytes: &[u8]) {
        ENCODER.write(bytes, SerialLogger::write);
    }

    unsafe fn flush() {
    }

}

struct CircularBuffer<const N: usize> {
    read_index: usize,
    write_index: usize,
    full: bool,
    buffer: [u8; N],
}

impl<const N: usize> CircularBuffer<N> {
    pub const fn new() -> Self {
        const { assert!(N < 256, "Buffer size has to fit in an u8") };
        Self {
            read_index: 0,
            write_index: 0,
            full: false,
            buffer: [0; N],
        }
    }

    fn free_space(&self) -> usize {
        N - self.used_space()
    }

    fn used_space(&self) -> usize {
        if self.write_index < self.read_index {
            N - (self.read_index - self.write_index)
        } else {
            self.write_index - self.read_index
        }
    }

    /// Returns number of bytes written
    pub fn write_non_blocking(&mut self, data: &[u8]) -> usize {
        let bytes_to_write = core::cmp::min(data.len(), self.free_space() as usize);
        //led(true);
        //loop{};
        //FIXME: handle full buffer read and write pointer same
        for d in &data[..bytes_to_write] {
            self.buffer[self.write_index as usize] = *d;
            self.write_index = (self.write_index + 1) % N;
        }

        bytes_to_write
    }

    /// Blocks if buffer is full
    pub fn write(&mut self, data: &[u8]) {
        let mut written = 0;

        while written < data.len() {
            written += self.write_non_blocking(&data[written..]);
        }
    }

    pub fn get_byte(&mut self) -> Option<u8> {
        if self.used_space() > 0 {
            let data = self.buffer[self.read_index as usize];
            //TODO check write ptr
            self.read_index = (self.read_index + 1) % N;

            Some(data)
        } else {
            None
        }
    }
}
