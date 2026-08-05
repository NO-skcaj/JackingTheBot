use anyhow::{anyhow, Result};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::{Duration, Instant};

use vigem_client::{Client, TargetId, XButtons, XGamepad, Xbox360Wired, XButtons as XB};

// ---------- Configuration ----------

const SAMPLE_WINDOW: usize = 4096;   // samples analyzed per pitch estimate
const HOP_SIZE: usize = 2048;        // how often detection runs
const ONSET_THRESHOLD: f32 = 0.002;   // RMS amplitude gate, tune to your pickup level
const YIN_THRESHOLD: f32 = 0.14;     // YIN confidence threshold
const RETRIGGER_COOLDOWN_MS: u64 = 500; // ignore repeat triggers on same string
const BUTTON_HOLD_MS: u64 = 1000;      // virtual button hold duration

// Standard tuning fundamental frequencies (Hz), low E to high E
const STRING_FREQS: [(&str, f32); 6] = [
    ("E2", 82.41),
    ("A2", 110.00),
    ("D3", 146.83),
    ("G3", 196.00),
    ("B3", 246.94),
    ("E4", 329.63),
];

#[derive(Debug, Clone, Copy)]
enum StringEvent {
    Struck(usize), // index into STRING_FREQS
}

fn main() -> Result<()> {
    let (tx, rx) = channel::<StringEvent>();
    let _stream = start_audio_capture(tx)?;
    run_gamepad_controller(rx)?;
    Ok(())
}

// ---------- Audio capture + pitch detection ----------

fn start_audio_capture(tx: Sender<StringEvent>) -> Result<cpal::Stream> {
    let host = cpal::default_host();

    // Prefer a device whose name suggests it's the USB guitar adapter;
    // fall back to the default input device otherwise.
    let device = host
        .input_devices()?
        .find(|d| {
            d.name()
                .map(|n| {
                    let n = n.to_lowercase();
                    n.contains("usb") || n.contains("rocksmith") || n.contains("realtone")
                })
                .unwrap_or(false)
        })
        .or_else(|| host.default_input_device())
        .ok_or_else(|| anyhow!("No input audio device found"))?;

    println!("Using input device: {}", device.name()?);

    let config = device.default_input_config()?;
    let sample_rate = config.sample_rate().0 as f32;
    let channels = config.channels() as usize;

    println!(
        "Input config: {} Hz, {} channel(s), format {:?}",
        sample_rate,
        channels,
        config.sample_format()
    );

    let mut ring_buf: Vec<f32> = Vec::with_capacity(SAMPLE_WINDOW * 2);
    let mut last_trigger: [Option<Instant>; 6] = [None; 6];

    let err_fn = |err| eprintln!("Audio stream error: {}", err);

    let stream = match config.sample_format() {
        cpal::SampleFormat::F32 => device.build_input_stream(
            &config.into(),
            move |data: &[f32], _| {
                process_audio_chunk(data, channels, sample_rate, &mut ring_buf, &mut last_trigger, &tx);
            },
            err_fn,
            None,
        )?,
        cpal::SampleFormat::I16 => {
            let cfg = config.into();
            device.build_input_stream(
                &cfg,
                move |data: &[i16], _| {
                    let converted: Vec<f32> =
                        data.iter().map(|s| *s as f32 / i16::MAX as f32).collect();
                    process_audio_chunk(&converted, channels, sample_rate, &mut ring_buf, &mut last_trigger, &tx);
                },
                err_fn,
                None,
            )?
        }
        other => return Err(anyhow!("Unsupported sample format: {:?}", other)),
    };

    stream.play()?;
    Ok(stream)
}

fn process_audio_chunk(
    data: &[f32],
    channels: usize,
    sample_rate: f32,
    ring_buf: &mut Vec<f32>,
    last_trigger: &mut [Option<Instant>; 6],
    tx: &Sender<StringEvent>,
) {
    // Downmix to mono (guitar signal typically arrives on channel 0).
    for frame in data.chunks(channels) {
        ring_buf.push(frame[0]);
    }

    while ring_buf.len() >= SAMPLE_WINDOW {
        let window: Vec<f32> = ring_buf[..SAMPLE_WINDOW].to_vec();
        ring_buf.drain(..HOP_SIZE.min(ring_buf.len()));

        let level = rms(&window);
        if level < ONSET_THRESHOLD {
            continue; // too quiet, no note being played
        }

        if let Some(freq) = yin_pitch(&window, sample_rate, YIN_THRESHOLD) {
            if let Some(idx) = closest_string(freq) {
                let now = Instant::now();
                let cooldown_ok = match last_trigger[idx] {
                    Some(t) => now.duration_since(t) > Duration::from_millis(RETRIGGER_COOLDOWN_MS),
                    None => true,
                };
                if cooldown_ok {
                    last_trigger[idx] = Some(now);
                    let _ = tx.send(StringEvent::Struck(idx));
                }
            }
        }
    }
}

fn rms(samples: &[f32]) -> f32 {
    let sum_sq: f32 = samples.iter().map(|s| s * s).sum();
    (sum_sq / samples.len() as f32).sqrt()
}

/// Maps a detected frequency to the closest guitar string, folding
/// across a few octaves so fretted notes on a string still resolve
/// back to that string (not just open-string strums).
fn closest_string(freq: f32) -> Option<usize> {
    let mut best_idx = None;
    let mut best_cents = f32::MAX;

    for (i, (_, string_freq)) in STRING_FREQS.iter().enumerate() {
        for octave_mult in [0.5, 1.0, 2.0, 4.0] {
            let ref_freq = string_freq * octave_mult;
            let cents = 1200.0 * (freq / ref_freq).log2();
            if cents.abs() < best_cents {
                best_cents = cents.abs();
                best_idx = Some(i);
            }
        }
    }

    if best_cents < 70.0 {
        best_idx
    } else {
        None
    }
}

/// Simplified YIN pitch detection. Returns estimated fundamental
/// frequency in Hz, or None if no confident pitch was found.
fn yin_pitch(samples: &[f32], sample_rate: f32, threshold: f32) -> Option<f32> {
    let n = samples.len();
    let max_tau = n / 2;
    let mut diff = vec![0.0f32; max_tau];

    for tau in 1..max_tau {
        let mut sum = 0.0;
        for i in 0..max_tau {
            let d = samples[i] - samples[i + tau];
            sum += d * d;
        }
        diff[tau] = sum;
    }

    let mut cmnd = vec![1.0f32; max_tau];
    let mut running_sum = 0.0;
    for tau in 1..max_tau {
        running_sum += diff[tau];
        cmnd[tau] = diff[tau] * tau as f32 / running_sum.max(1e-10);
    }

    let mut tau_estimate: Option<usize> = None;
    let mut tau = 2;
    while tau < max_tau {
        if cmnd[tau] < threshold {
            while tau + 1 < max_tau && cmnd[tau + 1] < cmnd[tau] {
                tau += 1;
            }
            tau_estimate = Some(tau);
            break;
        }
        tau += 1;
    }

    let tau = tau_estimate?;

    let better_tau = if tau > 0 && tau + 1 < max_tau {
        let s0 = cmnd[tau - 1];
        let s1 = cmnd[tau];
        let s2 = cmnd[tau + 1];
        let denom = 2.0 * (2.0 * s1 - s2 - s0);
        if denom.abs() > 1e-10 {
            tau as f32 + (s2 - s0) / denom
        } else {
            tau as f32
        }
    } else {
        tau as f32
    };

    if better_tau <= 0.0 {
        return None;
    }

    Some(sample_rate / better_tau)
}

// ---------- Virtual gamepad output ----------

fn run_gamepad_controller(rx: Receiver<StringEvent>) -> Result<()> {
    let client = Client::connect()
        .map_err(|e| anyhow!("Failed to connect to ViGEmBus. Is it installed? ({:?})", e))?;

    let id = TargetId::XBOX360_WIRED;
    let mut target = Xbox360Wired::new(client, id);
    target
        .plugin()
        .map_err(|e| anyhow!("Failed to plug in virtual controller: {:?}", e))?;
    target
        .wait_ready()
        .map_err(|e| anyhow!("Virtual controller not ready: {:?}", e))?;

    println!("Virtual Xbox 360 controller ready. Listening for string events...");

    // String index (low E .. high E) -> controller button.
    // Adjust to match your FRC driver station bindings.
    let button_map: [u16; 6] = [
        XB::A,
        XB::B,
        XB::X,
        XB::Y,
        XB::LB,
        XB::RB,
    ];

    loop {
        let event = rx.recv().map_err(|_| anyhow!("Audio thread disconnected"))?;
        let StringEvent::Struck(idx) = event;

        println!("String {} struck -> button pressed", STRING_FREQS[idx].0);

        let mut gamepad = XGamepad::default();
        gamepad.buttons = XButtons::from(button_map[idx]);
        target
            .update(&gamepad)
            .map_err(|e| anyhow!("Failed to update gamepad: {:?}", e))?;

        std::thread::sleep(Duration::from_millis(BUTTON_HOLD_MS));

        let gamepad = XGamepad::default();
        target
            .update(&gamepad)
            .map_err(|e| anyhow!("Failed to release gamepad: {:?}", e))?;
    }
}