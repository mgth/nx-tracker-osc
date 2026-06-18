//! Candidate payload interpretations and the live range tracker.
//!
//! The internal layout of the `a015` payload is unknown, so we present several
//! competing readings side by side and track, per slot, the min/max observed
//! across the session. Moving the head on a single axis makes the slots that
//! encode that axis stand out (wide range / long bar) while constant header
//! bytes and monotonic counters are easy to dismiss.

use std::io::Write as _;
use std::time::{Duration, Instant};

use crate::ble::Frame;

/// A candidate way to read the payload as a sequence of numbers.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Layout {
    I16Le,
    I16Be,
    F32Le,
    F32Be,
}

impl Layout {
    pub const ALL: [Layout; 4] = [Layout::I16Le, Layout::I16Be, Layout::F32Le, Layout::F32Be];

    fn label(self) -> &'static str {
        match self {
            Layout::I16Le => "i16 LE",
            Layout::I16Be => "i16 BE",
            Layout::F32Le => "f32 LE",
            Layout::F32Be => "f32 BE",
        }
    }

    fn elem_size(self) -> usize {
        match self {
            Layout::I16Le | Layout::I16Be => 2,
            Layout::F32Le | Layout::F32Be => 4,
        }
    }

    fn is_float(self) -> bool {
        matches!(self, Layout::F32Le | Layout::F32Be)
    }

    fn count(self, len: usize) -> usize {
        len / self.elem_size()
    }

    fn value(self, bytes: &[u8], idx: usize) -> f64 {
        let off = idx * self.elem_size();
        match self {
            Layout::I16Le => i16::from_le_bytes([bytes[off], bytes[off + 1]]) as f64,
            Layout::I16Be => i16::from_be_bytes([bytes[off], bytes[off + 1]]) as f64,
            Layout::F32Le => f32::from_le_bytes([
                bytes[off],
                bytes[off + 1],
                bytes[off + 2],
                bytes[off + 3],
            ]) as f64,
            Layout::F32Be => f32::from_be_bytes([
                bytes[off],
                bytes[off + 1],
                bytes[off + 2],
                bytes[off + 3],
            ]) as f64,
        }
    }
}

#[derive(Clone, Copy)]
struct ColStat {
    min: f64,
    max: f64,
    last: f64,
    seen: bool,
}

impl Default for ColStat {
    fn default() -> Self {
        ColStat {
            min: f64::INFINITY,
            max: f64::NEG_INFINITY,
            last: 0.0,
            seen: false,
        }
    }
}

impl ColStat {
    fn update(&mut self, v: f64) {
        self.last = v;
        self.seen = true;
        if v.is_finite() {
            self.min = self.min.min(v);
            self.max = self.max.max(v);
        }
    }

    fn range(&self) -> f64 {
        if self.min.is_finite() && self.max.is_finite() {
            self.max - self.min
        } else {
            0.0
        }
    }
}

pub struct Analyzer {
    stream_mode: bool,
    decode: bool,
    interval: Duration,
    frame_count: u64,
    payload_len: usize,
    first: Option<Instant>,
    last_render: Instant,
    frames_at_last_render: u64,
    /// Per-layout, per-slot statistics. Index matches [`Layout::ALL`].
    stats: [Vec<ColStat>; 4],
}

impl Analyzer {
    pub fn new(stream_mode: bool, print_hz: f64, decode: bool) -> Self {
        let hz = if print_hz > 0.0 { print_hz } else { 10.0 };
        Analyzer {
            stream_mode,
            decode,
            interval: Duration::from_secs_f64(1.0 / hz),
            frame_count: 0,
            payload_len: 0,
            first: None,
            last_render: Instant::now(),
            frames_at_last_render: 0,
            stats: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
        }
    }

    pub fn on_frame(&mut self, frame: &Frame) {
        if self.first.is_none() {
            self.first = Some(Instant::now());
        }
        self.frame_count += 1;
        self.ensure_capacity(frame.bytes.len());

        for (i, layout) in Layout::ALL.iter().enumerate() {
            for idx in 0..layout.count(frame.bytes.len()) {
                let v = layout.value(&frame.bytes, idx);
                self.stats[i][idx].update(v);
            }
        }

        if self.stream_mode {
            self.print_stream(frame);
        } else if self.last_render.elapsed() >= self.interval {
            self.render_table(frame);
        }
    }

    /// Reset the per-slot statistics when the payload length changes.
    fn ensure_capacity(&mut self, len: usize) {
        if len == self.payload_len {
            return;
        }
        self.payload_len = len;
        for (i, layout) in Layout::ALL.iter().enumerate() {
            self.stats[i] = vec![ColStat::default(); layout.count(len)];
        }
    }

    fn print_stream(&self, frame: &Frame) {
        let secs = frame.ts_us as f64 / 1_000_000.0;
        let mut out = String::new();
        out.push_str(&format!(
            "[t={secs:>11.6}s  #{:<6}  len={}]\n",
            self.frame_count,
            frame.bytes.len()
        ));
        out.push_str(&format!("  hex: {}\n", hex_string(&frame.bytes)));
        if self.decode {
            out.push_str(&format!("  {}\n", decoded_line(&frame.bytes)));
        }
        for layout in Layout::ALL {
            let mut cells = String::new();
            for idx in 0..layout.count(frame.bytes.len()) {
                cells.push_str(&format!("{} ", fmt_value(layout, layout.value(&frame.bytes, idx))));
            }
            out.push_str(&format!("  {:<7}: {}\n", layout.label(), cells.trim_end()));
        }
        print(&out);
    }

    fn render_table(&mut self, frame: &Frame) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_render).as_secs_f64();
        let recent = (self.frame_count - self.frames_at_last_render) as f64;
        let inst_hz = if dt > 0.0 { recent / dt } else { 0.0 };
        let avg_hz = self
            .first
            .map(|f| self.frame_count as f64 / f.elapsed().as_secs_f64().max(1e-9))
            .unwrap_or(0.0);
        self.last_render = now;
        self.frames_at_last_render = self.frame_count;

        let mut out = String::from("\x1b[2J\x1b[H");
        out.push_str("NX raw monitor — move your head on ONE axis at a time; watch the RANGE / bar\n");
        out.push_str(&format!(
            "frames: {}   rate: {:.1} Hz (avg {:.1})   payload: {} bytes\n",
            self.frame_count,
            inst_hz,
            avg_hz,
            frame.bytes.len()
        ));
        out.push_str(&format!("hex: {}\n", hex_string(&frame.bytes)));
        if self.decode {
            out.push_str(&format!("{}\n", decoded_line(&frame.bytes)));
        }

        for (i, layout) in Layout::ALL.iter().enumerate() {
            out.push('\n');
            out.push_str(&format!(
                "=== {} ({} values) ===\n",
                layout.label(),
                self.stats[i].len()
            ));
            out.push_str(" idx       current           min           max         range\n");
            let max_range = self.stats[i]
                .iter()
                .map(ColStat::range)
                .fold(0.0_f64, f64::max);
            for (idx, s) in self.stats[i].iter().enumerate() {
                out.push_str(&format!(
                    "{:>3}  {:>12}  {:>12}  {:>12}  {:>12}  {}\n",
                    idx,
                    fmt_value(*layout, s.last),
                    fmt_value(*layout, s.min),
                    fmt_value(*layout, s.max),
                    fmt_value(*layout, s.range()),
                    bar(s.range(), max_range, 18),
                ));
            }
        }
        print(&out);
    }
}

fn decoded_line(bytes: &[u8]) -> String {
    match crate::decode::decode(bytes) {
        Ok(o) => format!(
            "decoded: quat(w,x,y,z)=({:+.4},{:+.4},{:+.4},{:+.4})  yaw={:+7.2}  pitch={:+7.2}  roll={:+7.2}",
            o.quat[0], o.quat[1], o.quat[2], o.quat[3], o.yaw_deg, o.pitch_deg, o.roll_deg
        ),
        Err(e) => format!("decoded: <error: {e}>"),
    }
}

fn hex_string(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn fmt_value(layout: Layout, v: f64) -> String {
    if !v.is_finite() {
        format!("{v}")
    } else if layout.is_float() {
        format!("{v:.4}")
    } else {
        format!("{}", v as i64)
    }
}

fn bar(range: f64, max_range: f64, width: usize) -> String {
    if max_range <= 0.0 || !range.is_finite() || range <= 0.0 {
        return String::new();
    }
    let filled = ((range / max_range) * width as f64).round() as usize;
    "\u{2588}".repeat(filled.min(width))
}

fn print(s: &str) {
    let mut stdout = std::io::stdout().lock();
    let _ = stdout.write_all(s.as_bytes());
    let _ = stdout.flush();
}
