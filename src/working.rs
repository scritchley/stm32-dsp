//!
//! # I2S example for STM32F411
//!
//! This application demonstrates I2S communication with the DAC on an STM32F411E-DISCO board.
//! Unlike `i2s-audio-out`, this example uses DMA instead of writing one sample at a time.
//!
//! # Hardware required
//!
//! * STM32F407G-DISC1 or STM32F411E-DISCO evaluation board
//! * Headphones or speakers with a headphone plug
//!
//! # Procedure
//!
//! 1. Connect the headphones or speakers to the headphone jack on the evaluation board
//!    (warning: the DAC may produce a powerful signal that becomes a very loud sound.
//!    Set the speaker volume to minimum, or do not put on the headphones.)
//! 2. Load this compiled application on the microcontroller and run it
//!
//! Expected behavior: the speakers/headphones emit a continuous 750 Hz tone
//!
//! # Pins and addresses
//!
//! * PD4 -> DAC ~RESET (pulled low)
//!
//! * PB9 -> SDA (pulled high)
//! * PB6 -> SCL (pulled high)
//!
//! * PC7 -> MCLK
//! * PC10 -> SCK (bit clock)
//! * PC12 -> SD
//! * PA4 -> WS
//!
//! DAC I2C address 0x94
//!

#![no_std]
#![no_main]

mod cs43l22;
mod dsp;

use core::cell::RefCell;
use stm32f4xx_hal::dma::Stream0;

use core::panic::PanicInfo;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

use stm32_i2s_v12x::format::{Data16Frame16, FrameFormat};
use stm32_i2s_v12x::{MasterClock, MasterConfig, Polarity, ReceiveMode, TransmitMode};

use dsp::{DelayEffect, HighPass, PitchShift, Processor};
use stm32f4xx_hal::delay::Delay;
use stm32f4xx_hal::dma::config::{DmaConfig, Priority};
use stm32f4xx_hal::dma::{Channel0, CurrentBuffer, Stream4, StreamsTuple, Transfer};
use stm32f4xx_hal::dma::{MemoryToPeripheral, PeripheralToMemory};
use stm32f4xx_hal::gpio::gpioa::PA4;
use stm32f4xx_hal::gpio::gpiob::{PB10, PB12};
use stm32f4xx_hal::gpio::gpioc::{PC10, PC12, PC3, PC6, PC7};
use stm32f4xx_hal::gpio::Alternate;
use stm32f4xx_hal::gpio::{AF5, AF6};
use stm32f4xx_hal::i2s::I2s;
use stm32f4xx_hal::pac::{interrupt, Interrupt};
use stm32f4xx_hal::pac::{CorePeripherals, Peripherals};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::stm32::{DMA1, SPI2, SPI3};
use rtt_target::{rprintln, rtt_init_print};

const BUFFER_SIZE: usize = 128;

// Allocate the audio buffers
static mut TX_FIRST_BUFFER: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut TX_DOUBLE_BUFFER: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];

static mut RX_FIRST_BUFFER: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut RX_DOUBLE_BUFFER: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    // The 86 MHz frequency can be divided to get a sample rate very close to 48 kHz.
    let clocks = rcc.cfgr.use_hse(8.mhz()).i2s_clk(86.mhz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let mut delay = Delay::new(cp.SYST, clocks);

    // I2S pins: (WS, CK, MCLK, SD) for I2S2
    let i2s_pins_tx = (
        gpiob.pb12.into_alternate(),
        gpiob.pb10.into_alternate(),
        gpioc.pc6.into_alternate(),
        gpioc.pc3.into_alternate(),
    );
    let hal_i2s_tx = I2s::i2s2(dp.SPI2, i2s_pins_tx, clocks);
    let i2s_clock_tx = hal_i2s_tx.input_clock();

    // I2S pins: (WS, CK, MCLK, SD) for I2S3
    let i2s_pins_rx = (
        gpioa.pa4.into_alternate(),
        gpioc.pc10.into_alternate(),
        gpioc.pc7.into_alternate(),
        gpioc.pc12.into_alternate(),
    );
    let hal_i2s_rx = I2s::i2s3(dp.SPI3, i2s_pins_rx, clocks);
    let i2s_clock_rx = hal_i2s_tx.input_clock();

    // Audio timing configuration:
    // Sample rate 48 kHz
    // 16 bits per sample -> SCK rate 1.536 MHz
    // MCK frequency = 256 * sample rate -> MCK rate 12.228 MHz (also equal to 8 * SCK rate)
    let sample_rate = 48000;

    let i2s = stm32_i2s_v12x::I2s::new(hal_i2s_tx);
    let mut i2s_tx = i2s.configure_master_transmit(MasterConfig::with_sample_rate(
        i2s_clock_tx.0,
        sample_rate,
        Data16Frame16,
        FrameFormat::PhilipsI2s,
        Polarity::IdleLow,
        MasterClock::Enable,
    ));
    i2s_tx.set_dma_enabled(true);

    let i2s3 = stm32_i2s_v12x::I2s::new(hal_i2s_rx);
    let mut i2s_rx = i2s3.configure_master_receive(MasterConfig::with_sample_rate(
        i2s_clock_rx.0,
        sample_rate,
        Data16Frame16,
        FrameFormat::PhilipsI2s,
        Polarity::IdleLow,
        MasterClock::Enable,
    ));
    i2s_rx.set_dma_enabled(true);

    // Wait at least 550 ns before starting I2C communication
    delay.delay_us(1u8);

    // Set up DMA: DMA 1 stream 4 channel 0 memory -> peripheral
    let dma1_streams = StreamsTuple::new(dp.DMA1);

    unsafe {
        let tx_dma_config = DmaConfig::default()
            .memory_increment(true)
            .double_buffer(true)
            .priority(Priority::Low)
            .transfer_complete_interrupt(true);
        let mut dma_transfer_tx: I2sDmaTransferTx = Transfer::init_memory_to_peripheral(
            dma1_streams.4,
            i2s_tx,
            &mut TX_FIRST_BUFFER,
            Some(&mut TX_DOUBLE_BUFFER),
            tx_dma_config,
        );
        dma_transfer_tx.start(|i2s| i2s.enable());
        let dma_config_rx = DmaConfig::default()
            .memory_increment(true)
            .double_buffer(true)
            .priority(Priority::High)
            .transfer_complete_interrupt(true);
        let mut dma_transfer_rx: I2sDmaTransferRx = Transfer::init_peripheral_to_memory(
            dma1_streams.0,
            i2s_rx,
            &mut RX_FIRST_BUFFER,
            Some(&mut RX_DOUBLE_BUFFER),
            dma_config_rx,
        );
        dma_transfer_rx.start(|i2s| i2s.enable());
        // Hand off transfer to interrupt handler
        cortex_m::interrupt::free(|cs| {
            *G_TRANSFER_TX.borrow(cs).borrow_mut() = Some(dma_transfer_tx)
        });
        cortex_m::interrupt::free(|cs| {
            *G_TRANSFER_RX.borrow(cs).borrow_mut() = Some(dma_transfer_rx)
        })
    }

    // Enable interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_STREAM4);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_STREAM0);
    }

    loop {
        // Don't WFI. That can cause problems attaching the debugger.
    }
}

type I2sDmaTransferTx = Transfer<
    Stream4<DMA1>,
    Channel0,
    stm32_i2s_v12x::I2s<
        I2s<
            SPI2,
            (
                PB12<Alternate<AF5>>,
                PB10<Alternate<AF5>>,
                PC6<Alternate<AF5>>,
                PC3<Alternate<AF5>>,
            ),
        >,
        TransmitMode<Data16Frame16>,
    >,
    MemoryToPeripheral,
    &'static mut [u16; BUFFER_SIZE],
>;

/// DMA transfer handoff from main() to interrupt handler
static G_TRANSFER_TX: Mutex<RefCell<Option<I2sDmaTransferTx>>> = Mutex::new(RefCell::new(None));

/// This interrupt handler runs when DMA 1 finishes a transfer to the I2S peripheral
#[interrupt]
fn DMA1_STREAM4() {
    static mut TRANSFER: Option<I2sDmaTransferTx> = None;

    let transfer = TRANSFER.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_TRANSFER_TX.borrow(cs).replace(None).unwrap())
    });

    transfer.clear_half_transfer_interrupt();
    unsafe {
        transfer
            .next_transfer_with(|buffer, active_buffer| (buffer, active_buffer))
            .unwrap();
    }
}

/// DMA transfer handoff from main() to interrupt handler
static G_TRANSFER_RX: Mutex<RefCell<Option<I2sDmaTransferRx>>> = Mutex::new(RefCell::new(None));

type I2sDmaTransferRx = Transfer<
    Stream0<DMA1>,
    Channel0,
    stm32_i2s_v12x::I2s<
        I2s<
            SPI3,
            (
                PA4<Alternate<AF6>>,
                PC10<Alternate<AF6>>,
                PC7<Alternate<AF6>>,
                PC12<Alternate<AF6>>,
            ),
        >,
        ReceiveMode<Data16Frame16>,
    >,
    PeripheralToMemory,
    &'static mut [u16; BUFFER_SIZE],
>;

/// This interrupt handler runs when DMA 1 finishes a transfer to the I2S peripheral
#[interrupt]
fn DMA1_STREAM0() {
    static mut TRANSFER: Option<I2sDmaTransferRx> = None;
    static mut FX: Option<(HighPass, DelayEffect, PitchShift)> = None;
    // static mut HIPASS: Option<HighPass> = None;
    // static mut PITCH_SHIFT: Option<DelayEffect> = None;
    // static mut buf1: [u16; BUFFER_SIZE] = [0u16; BUFFER_SIZE];

    let transfer = TRANSFER.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_TRANSFER_RX.borrow(cs).replace(None).unwrap())
    });

    let fx = FX.get_or_insert_with(|| (HighPass::new(), DelayEffect::new(), PitchShift::new()));
    // let hp = HIPASS.get_or_insert_with(|| HighPass::new());
    // let pitch_shift = PITCH_SHIFT.get_or_insert_with(|| PitchShift::new());

    transfer.clear_half_transfer_interrupt();
    unsafe {
        transfer
            .next_transfer_with(|input, active_buffer| {
                // Write to output
                let dest_buffer = match active_buffer {
                    CurrentBuffer::FirstBuffer => &mut TX_FIRST_BUFFER,
                    CurrentBuffer::DoubleBuffer => &mut TX_DOUBLE_BUFFER,
                };
                for i in 0..BUFFER_SIZE {
                    dest_buffer[i] = fx.1.apply(input[i]);
                }
                (input, active_buffer)
            })
            .unwrap();
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
