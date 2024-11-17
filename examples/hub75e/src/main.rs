//! Displays the text "Rust embedded-graphics" on two daisy-chained 128x64 HUB75E matrixes
//! With a circle that walks across the screen
//!
#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)] // Don't want the extra fluff in build output :))))


use bsp::entry;
use core::ptr;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal::pio::PIOExt;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
};
use hub75_pio;
use hub75_pio::dma::DMAExt;
use hub75_pio::lut::GammaLut;

use core::cell::RefCell;
use critical_section::Mutex;
use embedded_graphics::{
    prelude::*,
    pixelcolor::Rgb888,
    primitives::{
        PrimitiveStyle,
        Circle
    },
    mono_font::{
        iso_8859_15::FONT_9X18_BOLD,
        MonoTextStyle,
    },
    text::Text
};
use rp_pico as bsp;

static mut DISPLAY_BUFFER: hub75_pio::DisplayMemory<256, 64, 12> = hub75_pio::DisplayMemory::new();
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
    let resets = pac.RESETS;
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
        let lut: GammaLut<12, _, _> = GammaLut::new();
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

    let bigfont = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb888::WHITE);

    loop {
		for x in (0..=189).step_by(2) {
			// Walking circle
            let _ = Circle::new(Point::new(20 + x, 19), 26)
                .into_styled(PrimitiveStyle::with_stroke(Rgb888::GREEN, 3))
                .draw(&mut display);

            // Text
			let _ = Text::new("Rust embedded-graphics", Point::new(29, 35), bigfont)
				.draw(&mut display);

			display.commit();
		}

		for x in (0..=189).step_by(2) {
            // Walking circle
            let _ = Circle::new(Point::new(209 - x, 19), 26)
                .into_styled(PrimitiveStyle::with_stroke(Rgb888::RED, 3))
                .draw(&mut display);

            // Text
			let _ = Text::new("Rust embedded-graphics", Point::new(29, 35), bigfont)
				.draw(&mut display);

			display.commit();
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
