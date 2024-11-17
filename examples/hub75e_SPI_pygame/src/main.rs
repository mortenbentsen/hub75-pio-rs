//! This example drives two daisy-chained 128x64 HUB75E matrixes.
//! It receives frames via SPI sent from pygame, and displays them on the matrixes.
//! A separate GPIO is used to commmit the frame to the display, to facilitate 
//! easier syncronizationbetween multiple drivers.
//!
#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)] // Don't want the extra fluff in build output :))))


use bsp::entry;
use core::ptr;
//use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal::pio::PIOExt;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog
};
use hub75_pio;
use hub75_pio::dma::DMAExt;
use hub75_pio::lut::GammaLut;

use core::cell::RefCell;
use critical_section::Mutex;
use embedded_graphics::{
    //prelude::*,
    pixelcolor::Rgb888
};
use rp_pico as bsp;


// SPI related imports
use embedded_hal::{
    spi::MODE_3,
    digital::v2::InputPin
};
use bsp::hal::{
    spi::Spi,
    gpio::{
        Pins,
        FunctionSpi
    }
};

//use fugit::RateExtU32;
//use cortex_m::prelude::_embedded_hal_blocking_spi_Read;
use embedded_hal::spi::FullDuplex;

use core::fmt::Debug;
use defmt::info;

const DISPLAY_WIDTH: usize = 256;
const DISPLAY_HEIGHT: usize = 64;
const DISPLAY_BITS: usize = 12;


static mut DISPLAY_BUFFER: hub75_pio::DisplayMemory<DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_BITS> = hub75_pio::DisplayMemory::new();
static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0u32));

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Split PIO0 SM
    let (mut pio, sm0, sm1, sm2, _) = pac.PIO0.split(&mut pac.RESETS);

    // Reset DMA
    let resets = &mut pac.RESETS;
    resets.reset.modify(|_, w| w.dma().set_bit());
    resets.reset.modify(|_, w| w.dma().clear_bit());
    while resets.reset_done.read().dma().bit_is_clear() {}

    // Split DMA
    let dma = &pac.DMA;
    dma.inte0.write(|w| unsafe { w.bits(1 << 0) });

    let dma = pac.DMA.split();

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    let lut = {
        let lut: GammaLut<12, Rgb888, _> = GammaLut::new();
        lut.init((2.1, 2.1, 2.1))
    };
    let mut display = unsafe {
        hub75_pio::Display::new(
            &mut DISPLAY_BUFFER,
            hub75_pio::DisplayPins {
                r1: pins.gpio0.into_function().into_pull_type().into_dyn_pin(),
                g1: pins.gpio1.into_function().into_pull_type().into_dyn_pin(),
                b1: pins.gpio2.into_function().into_pull_type().into_dyn_pin(),
                r2: pins.gpio3.into_function().into_pull_type().into_dyn_pin(),
                g2: pins.gpio4.into_function().into_pull_type().into_dyn_pin(),
                b2: pins.gpio5.into_function().into_pull_type().into_dyn_pin(),
                addra: pins.gpio6.into_function().into_pull_type().into_dyn_pin(),
                addrb: pins.gpio7.into_function().into_pull_type().into_dyn_pin(),
                addrc: pins.gpio8.into_function().into_pull_type().into_dyn_pin(),
                addrd: pins.gpio9.into_function().into_pull_type().into_dyn_pin(),
                addre: pins.gpio10.into_function().into_pull_type().into_dyn_pin(),
                clk: pins.gpio11.into_function().into_pull_type().into_dyn_pin(),
                lat: pins.gpio12.into_function().into_pull_type().into_dyn_pin(),
                oe: pins.gpio13.into_function().into_pull_type().into_dyn_pin(),
            },
            &mut pio,
            (sm0, sm1, sm2),
            (dma.ch0, dma.ch1, dma.ch2, dma.ch3),
            false,
            &lut,
        )
    };


    // SPI setup
    let miso = pins.gpio16.into_function::<FunctionSpi>();
    let sclk = pins.gpio18.into_function::<FunctionSpi>();
    let mosi = pins.gpio19.into_function::<FunctionSpi>();

    let csel = pins.gpio17.into_pull_up_input();


    let spi_device = pac.SPI0;
    let spi_pin_layout = (mosi, miso, sclk); // works


    let mut spi = Spi::<_, _, _, 8>::new(spi_device, spi_pin_layout)
        .init_slave(&mut pac.RESETS, MODE_3);

    let buf_size = DISPLAY_WIDTH * DISPLAY_HEIGHT / 2 * DISPLAY_BITS;

    // Create our own pointer to the buffer not currently being displayed
    let mut fb_pointer = display.mem.fb0.as_mut_ptr() as *mut u32;
    

    loop {

        // Wait for the CS line to to be asserted (go low)
        //info!("Waiting for CS line to be asserted");
        while csel.is_high().unwrap() {
            delay.delay_us(1u32);
        }

        // Point the display buffer to the framebuffer that is not currently being displayed
        if display.mem.fbptr[0] == display.mem.fb0.as_ptr() as u32 {
            fb_pointer = display.mem.fb1.as_mut_ptr() as *mut u32;
        } else {
            fb_pointer = display.mem.fb0.as_mut_ptr() as *mut u32;
        }


        // CS line went low, proceed to read the frame
        //info!("CS line was asserted, reading frame");

        for i in 0..buf_size {
            unsafe {
                *(fb_pointer as *mut u8).add(i) = read_spi(&mut spi, &csel);
            }
        }

        /* for y in 0..64 {
            //info!("{}", y);
            for x in 0..256 {

                /* while spi.
                let r = spi.read().unwrap();
                let g = spi.read().unwrap();
                let b = spi.read().unwrap();
                let a = spi.read().unwrap(); */

                let r = read_spi(&mut spi, &csel);
                let g = read_spi(&mut spi, &csel);
                let b = read_spi(&mut spi, &csel);
                //let _a = read_spi(&mut spi, &csel);

                display.set_pixel(x, y, Rgb888::new(r, g, b));
            }
        } */

        //info!("Frame read, waiting for CS line to be de-asserted");

        // Wait for the CS line to be de-asserted (go high) before committing the frame
        while csel.is_low().unwrap() {
            delay.delay_us(1u32);
        }

        //info!("CS line was deasserted, committing frame to display");

        display.commit();
        
	}
    
}



fn read_spi<T, P>(spi: &mut T, csel: &P) -> u8
where
    T: FullDuplex<u8>,
    <T as FullDuplex<u8>>::Error: Debug,
    P: InputPin,
{
    
    loop {
        // If the CS line is high, we can't read data and should just return 0s...
        match csel.is_high() {
            Ok(true) => return 0,  // If CS is high, return 0
            Ok(false) => { /* continue as normal */ }
            Err(_) => return 0, // If there's an error reading the pin, also return 0
        }

        // Check if data is available
        if let Ok(_) = spi.send(0) {
            // Data is available, read it
            match spi.read() {
                Ok(byte) => {
                    //info!("Read byte: {}", byte);
                    return byte;
                }
                Err(nb::Error::WouldBlock) => {
                    // Continue the loop to wait for data
                }
                Err(e) => panic!("SPI read error: {:?}", e),
            }
        } 
    }
}

#[interrupt]
fn DMA_IRQ_0() {
    critical_section::with(|cs| {
        COUNTER.replace_with(cs, |counter| (*counter + 1) % 100000000);
    });
    // Clear the DMA interrupt flag
    const INTS: *mut u32 = (0x50000000 + 0x40c) as *mut u32;
    unsafe { ptr::write_volatile(INTS, 0b1) };
}
