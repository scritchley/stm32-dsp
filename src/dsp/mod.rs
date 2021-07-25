use micromath::F32Ext;
use no_std_compat::slice::IterMut;

pub trait IntoF32 {
    fn into_f32(self) -> f32;
}

impl IntoF32 for u16 {
    fn into_f32(self) -> f32 {
        self as i16 as f32
    }
}

pub trait FromF32 {
    fn from_f32(value: f32) -> Self;
}

impl FromF32 for u16 {
    fn from_f32(value: f32) -> Self {
        value.round() as i16 as u16
    }
}

pub trait Processor {
    fn process(&mut self, input: &Buffer, output: &mut Buffer);
}

pub struct DelayLine {
    delay_buffer: [u16; 50000],
    delay_index: usize,
}

impl DelayLine {
    pub fn new() -> Self {
        Self {
            delay_buffer: [0u16; 50000],
            delay_index: 0,
        }
    }
}

impl Processor for DelayLine {
    fn process(&mut self, input: &Buffer, output: &mut Buffer) {
        let l = self.delay_buffer.len();
        let mut output_iter = output.buffer.iter_mut();
        for rx in input.buffer.iter() {
            let tx = output_iter.next().unwrap();
            let out = self.delay_buffer[self.delay_index % l];
            *tx = out.into_f32();
            self.delay_buffer[self.delay_index % l] = u16::from_f32(*rx);
            self.delay_index += 1;
        }
    }
}

pub struct PitchShift {
    buf: [f32; 5000],
    wp: f32,
    rp: f32,
    shift: f32,
    xf: f32,
    hp: HighPass,
}

impl PitchShift {
    pub fn new() -> Self {
        Self {
            buf: [0f32; 5000],
            wp: 0f32,
            rp: 0f32,
            shift: 1.5f32,
            xf: 1f32,
            hp: HighPass::new(),
        }
    }
}

impl Processor for PitchShift {
    fn process(&mut self, input: &Buffer, output: &mut Buffer) {
        let mut iter = input.buffer.iter();
        let mut output_iter = output.buffer.iter_mut();
        for _ in 0..64 {
            let left = iter.next().unwrap();
            let right = iter.next().unwrap();

            let mut sum = left + right;

            sum = self.hp.apply(sum);

            // Write to ring buffer
            self.buf[self.wp as usize] = sum;

            let r1 = self.rp;
            let l = self.buf.len() as f32;
            let lh = l / 2f32;
            let mut r2 = r1 + lh;
            if r1 >= lh {
                r2 = r1 - lh;
            }

            let rd0 = self.buf[r1 as usize];
            let rd1 = self.buf[r2 as usize];
            let overlap = 500f32;

            //Check if first readpointer starts overlap with write pointer?
            // if yes -> do cross-fade to second read-pointer
            if overlap >= (self.wp - r1) && (self.wp - r1) >= 0f32 && self.shift != 1f32 {
                let rel = self.wp - r1;
                self.xf = rel as f32 / (overlap as f32);
            } else if self.wp - r1 == 0f32 {
                self.xf = 0f32;
            }

            // //Check if second readpointer starts overlap with write pointer?
            // if yes -> do cross-fade to first read-pointer
            if overlap >= (self.wp - r2) && (self.wp - r2) >= 0f32 && self.shift != 1f32 {
                let rel = self.wp - r2;
                self.xf = 1f32 - (rel as f32 / (overlap as f32));
            } else if self.wp - r2 == 0f32 {
                self.xf = 1f32;
            }

            // // Sum the crossfade.
            sum = rd0 * self.xf + rd1 * (1f32 - self.xf);

            self.rp += self.shift;
            self.wp += 1f32;
            if self.wp == l {
                self.wp = 0f32;
            }
            if self.rp >= l as f32 {
                self.rp = 0f32;
            }

            let ltx = output_iter.next().unwrap();
            let rtx = output_iter.next().unwrap();
            *ltx = sum;
            *rtx = sum;
        }
    }
}

pub struct HighPass {
    a0: f32,
    a1: f32,
    a2: f32,
    b1: f32,
    b2: f32,
    hp_in_z1: f32,
    hp_in_z2: f32,
    hp_out_z1: f32,
    hp_out_z2: f32,
}

impl HighPass {
    pub fn new() -> Self {
        Self {
            a0: 0.9862117951198142f32,
            a1: -1.9724235902396283f32,
            a2: 0.9862117951198142f32,
            b1: -1.972233470205696f32,
            b2: 0.9726137102735608f32,
            hp_in_z1: 0f32,
            hp_in_z2: 0f32,
            hp_out_z1: 0f32,
            hp_out_z2: 0f32,
        }
    }
    pub fn apply(&mut self, i: f32) -> f32 {
        let o = self.a0 * i + self.a1 * self.hp_in_z1 + self.a2 * self.hp_in_z2
            - self.b1 * self.hp_out_z1
            - self.b2 * self.hp_out_z2;
        self.hp_in_z2 = self.hp_in_z1;
        self.hp_in_z1 = i;
        self.hp_out_z2 = self.hp_out_z1;
        self.hp_out_z1 = o;
        o
    }
}

impl Processor for HighPass {
    fn process(&mut self, input: &Buffer, output: &mut Buffer) {
        let mut output_iter = output.buffer.iter_mut();
        for rx in input.buffer.iter() {
            let tx = output_iter.next().unwrap();
            *tx = self.apply(*rx);
        }
    }
}

#[derive(PartialEq)]
enum CompressorState {
    None,
    Attack,
    GainReduction,
    Release,
}

pub struct Compressor {
    state: CompressorState,
    attack: usize,
    release: usize,
    hold: usize,
    timeout: usize,
    threshold: f32,
    gain_reduce: f32,
    gain: f32,
    gain_step_attack: f32,
    gain_step_release: f32,
}

impl Processor for Compressor {
    fn process(&mut self, input: &Buffer, output: &mut Buffer) {
        let mut output_iter = output.buffer.iter_mut();
        for i in input.buffer.iter() {
            if *i > self.threshold {
                if self.gain >= self.gain_reduce {
                    match self.state {
                        CompressorState::None => {
                            self.state = CompressorState::Attack;
                            self.timeout = self.attack;
                        }
                        CompressorState::Release => {
                            self.state = CompressorState::Attack;
                            self.timeout = self.attack;
                        }
                        _ => {}
                    }
                }
                if self.state == CompressorState::GainReduction {
                    self.timeout = self.hold;
                }
            }
            if *i < self.threshold && self.gain <= 1f32 {
                if self.timeout == 0 && self.state == CompressorState::GainReduction {
                    self.state = CompressorState::Release;
                    self.timeout = self.release;
                }
            }
            match self.state {
                CompressorState::Attack => {
                    if self.timeout > 0 && self.gain > self.gain_reduce {
                        self.gain -= self.gain_step_attack;
                        self.timeout -= 1;
                    } else {
                        self.state = CompressorState::GainReduction;
                        self.timeout = self.hold;
                    }
                }
                CompressorState::GainReduction => {
                    if self.timeout > 0 {
                        self.timeout -= 1;
                    } else {
                        self.state = CompressorState::Release;
                        self.timeout = self.release;
                    }
                }
                CompressorState::Release => {
                    if self.timeout > 0 && self.gain < 1f32 {
                        self.timeout -= 1;
                        self.gain += self.gain_step_release
                    } else {
                        self.state = CompressorState::None;
                    }
                }
                CompressorState::None => {
                    if self.gain < 1f32 {
                        self.gain = 1f32;
                    }
                }
            }
            let tx = output_iter.next().unwrap();
            *tx = *i * self.gain;
        }
    }
}

pub struct Buffer {
    pub buffer: [f32; 128],
}

impl Buffer {
    pub fn new() -> Self {
        Self {
            buffer: [0f32; 128],
        }
    }
    pub fn from_u16(input: &[u16; 128]) -> Self {
        let mut buffer = [0f32; 128];
        for i in 0..128 {
            buffer[i] = input[i].into_f32();
        }
        Self { buffer }
    }
    pub fn from_f32(input: &[f32; 128]) -> Self {
        Self {
            buffer: input.clone(),
        }
    }
    pub fn to_u16(&self, output: &mut [u16; 128]) {
        let iter = self.buffer.iter();
        let mut output_iter = output.iter_mut();
        for i in iter {
            let o = output_iter.next().unwrap();
            *o = u16::from_f32(*i);
        }
    }
    pub fn sum_to_u16(&self, output: &mut [u16; 128]) {
        let iter = self.buffer.iter();
        let mut output_iter = output.iter_mut();
        for i in iter {
            let o = output_iter.next().unwrap();
            *o = u16::from_f32(*i + (*o).into_f32());
        }
    }
    pub fn sum(&mut self, input: &Buffer)  -> &mut Self{
        unsafe {
            cmsis_dsp_sys_pregenerated::arm_add_f32(self.buffer.as_ptr(), input.buffer.as_ptr(), self.buffer.as_mut_ptr(), 128);
        }
        self
    }
    pub fn scale(&mut self, scale: f32) -> &mut Self {
        unsafe {
            cmsis_dsp_sys_pregenerated::arm_scale_f32(self.buffer.as_ptr(), scale, self.buffer.as_mut_ptr(), 128);
        }
        self
    }
    pub fn fill(&mut self, fill: f32) -> &mut Self {
        unsafe {
            cmsis_dsp_sys_pregenerated::arm_fill_f32(fill, self.buffer.as_mut_ptr(), 128);
        }
        self
    }
}
