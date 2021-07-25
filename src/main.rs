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

use crate::dsp::Buffer;
use core::cell::RefCell;
use stm32f4xx_hal::dma::Stream0;

use core::panic::PanicInfo;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

use stm32_i2s_v12x::format::{Data16Frame16, FrameFormat};
use stm32_i2s_v12x::{MasterClock, MasterConfig, Polarity, ReceiveMode, TransmitMode};

use dsp::{DelayLine, HighPass, PitchShift, Processor, IntoF32, FromF32};
use rtt_target::{rprintln, rtt_init_print};
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
use stm32f4xx_hal::pac::{CorePeripherals, Peripherals, DWT};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::stm32::{DMA1, SPI2, SPI3};

const BUFFER_SIZE: usize = 128;

const U32_MAX: u32 = 4_294_967_295u32;

macro_rules! op_cyccnt_diff {
    ( $( $x:expr )* ) => {
        {
            let before = DWT::get_cycle_count();
            $(
                let res = $x;
            )*
            let after = DWT::get_cycle_count();
            let diff =
                if after >= before {
                    after - before
                } else {
                    after + (U32_MAX - before)
                };
            (res, diff)
        }
    };
}

// Allocate the audio buffers
static mut TX_FIRST_BUFFER: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut TX_DOUBLE_BUFFER: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];

static mut RX_FIRST_BUFFER: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];
static mut RX_DOUBLE_BUFFER: [u16; BUFFER_SIZE] = [0; BUFFER_SIZE];

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    let rcc = dp.RCC.constrain();
    // The 86 MHz frequency can be divided to get a sample rate very close to 48 kHz.
    let clocks = rcc.cfgr.sysclk(168.mhz()).i2s_clk(86.mhz()).freeze();

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
            *G_TRANSFER_TX.borrow(cs).borrow_mut() = Some(dma_transfer_tx);
            *G_TRANSFER_RX.borrow(cs).borrow_mut() = Some(dma_transfer_rx);
        });
    }

    // Enable interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_STREAM0);
    }

    rprintln!("pedal running");

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

/// This interrupt handler runs when DMA 1 finishes a transfer from the I2S peripheral
#[interrupt]
fn DMA1_STREAM0() {
    static mut TRANSFER: Option<I2sDmaTransferRx> = None;
    static mut COUNT: usize = 0usize;
    static mut FX: Option<(HighPass, DelayLine, PitchShift)> = None;
    static mut FEEDBACK: Option<Buffer> = None;
    static mut TMP: [f32; BUFFER_SIZE] = [0f32; BUFFER_SIZE];
    static mut TMP2: [f32; BUFFER_SIZE] = [0f32; BUFFER_SIZE];

    let transfer = TRANSFER.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_TRANSFER_RX.borrow(cs).replace(None).unwrap())
    });

    let feedback = FEEDBACK.get_or_insert_with(|| Buffer::new());
    let fx = FX.get_or_insert_with(|| (HighPass::new(), DelayLine::new(), PitchShift::new()));
    // let hp = HIPASS.get_or_insert_with(|| HighPass::new());
    // let pitch_shift = PITCH_SHIFT.get_or_insert_with(|| PitchShift::new());

    transfer.clear_half_transfer_interrupt();
    unsafe {
        if let Err(e) = transfer.next_transfer_with(|input_u16, active_buffer| {
            *COUNT += input_u16.len();
            let (res, diff) = op_cyccnt_diff!({
                // Write to output
                let dest_buffer = match active_buffer {
                    CurrentBuffer::FirstBuffer => &mut TX_FIRST_BUFFER,
                    CurrentBuffer::DoubleBuffer => &mut TX_DOUBLE_BUFFER,
                };

                let mut dry = Buffer::from_u16(input_u16);

                dry.sum(&feedback);

                let mut pitch_wet = Buffer::from_f32(TMP);
                let mut delay_wet = Buffer::from_f32(TMP2);

                fx.2.process(&dry, &mut pitch_wet);
                fx.1.process(&pitch_wet.scale(0.5).sum(&dry), &mut delay_wet);

                let mix = dry.sum(&delay_wet.scale(0.5f32));
                
                mix.to_u16(dest_buffer);

                feedback.fill(0f32).sum(&delay_wet.scale(0.5f32));
                
                (input_u16, active_buffer)
            });
            if *COUNT % 48000 < BUFFER_SIZE {
                rprintln!("diff: {:?}", diff);
            }
            res
        }) {
            rprintln!("error transferring: {:?}", e);
        }
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
