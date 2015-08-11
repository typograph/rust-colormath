/*
   Color conversions
   
   Rust version:
   Copyright (c) 2015, Tim Fuchs (typograph@elec.ru)
   
   Original Javascript library:
   Copyright (c) 2011, Cory Nelson (phrosty@gmail.com)
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
       * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   When possible, constants are given as accurate pre-computed rationals. When not,
   they are given at double precision with a comment on how to compute them.
*/

#![allow(non_snake_case, unstable, dead_code)]
use std::f32;
use std::num::Float;

const REFX: f32 = 31271./32902.; // normalized standard observer D65.
const REFY: f32 = 1.;
const REFZ: f32 = 35827./32902.;

mod yuvmatrices {

    pub struct YUVMatrix {
        pub name: &'static str,
        pub rScale: f32,
        pub gScale: f32,
        pub bScale: f32,
        }
    
    pub static BT601: YUVMatrix = 
        YUVMatrix {
            name: "ITU-R BT.601 (DVD, JPEG, Youtube)",
            rScale: 0.299,
            gScale: 0.587,
            bScale: 0.114
            };
    pub static BT709: YUVMatrix = 
        YUVMatrix {
            name: "ITU-R BT.709 (HDTV)",
            rScale: 0.2125,
            gScale: 0.7154,
            bScale: 0.0721
            };
    pub static SMPTE240M: YUVMatrix = 
        YUVMatrix {
            name: "SMPTE 240.0M (very old HDTV)",
            rScale: 0.212,
            gScale: 0.701,
            bScale: 0.087
            };
    pub static FCC: YUVMatrix = 
        YUVMatrix {
            name: "FCC",
            rScale: 0.3,
            gScale: 0.59,
            bScale: 0.11
            };
    }

pub fn clamp(x: f32, min: f32, max: f32) -> f32 {
    if x < min { min }
    else if x > max { max }
    else { x }
    }

pub fn clamphue(hue: f32) -> f32{
    let mut h = hue % (2.0 * f32::consts::PI);
    if h < 0.0 {
        h += f32::consts::PI * 2.0;
        }
    h
    }

pub struct ColorLShuv {
    pub clamped: bool,
    pub L: f32,
    pub S: f32,
    pub h: f32,
    }
impl Copy for ColorLShuv {}
impl ColorLShuv {
pub fn new(L: f32, S: f32, h: f32, clamped: bool) -> ColorLShuv {
    ColorLShuv {
        clamped: clamped || L < 0.0 || L > 100.0, // TODO: what is S min/max?
        L: clamp(L, 0.0, 100.0),
        S: S,
        h: clamphue(h),
        }
    }

    pub fn toLChuv(&self) -> ColorLChuv {
        ColorLChuv::new(self.L, self.S * self.L, self.h, self.clamped)
        }
    }

pub struct ColorLChuv {
    pub clamped: bool,
    pub L: f32,
    pub C: f32,
    pub h: f32,
    }
impl Copy for ColorLChuv {}
impl ColorLChuv {

    pub fn new(L: f32, C: f32, h: f32, clamped: bool) -> ColorLChuv {
        const maxC: f32 = 240.0/316141.0*950343809713.sqrt()
        ColorLChuv {
            clamped: clamped || L < 0.0 || L > 100.0 || C < 0.0 || C > maxC,
            L: clamp(L, 0.0, 100.0),
            C: clamp(C, 0.0, maxC),
            h: clamphue(h),
            }
        }

    pub fn toLShuv(&self) -> ColorLShuv {
        ColorLShuv::new(self.L, self.C/self.L, self.h, self.clamped)
        }

    pub fn toLuv(&self) -> ColorLuv {
        let h: f32 = clamphue(self.h);
        ColorLuv::new(self.L, h.cos() * self.C, h.sin() * self.C, self.clamped)
        }
    }

pub struct ColorLuv {
    pub clamped: bool,
    pub L: f32,
    pub u: f32,
    pub v: f32,
    }
impl Copy for ColorLuv {}
impl ColorLuv {
    pub fn new(L: f32, u: f32, v: f32, clamped: bool) -> ColorLuv {
        ColorLuv {
            clamped: clamped || L < 0.0 || L > 100.0 || u < -81304600.0/316141.0 || v > 54113280.0/316141.0, // TODO: what is u max, and v min??
            L: clamp(L, 0.0, 100.0),
            u: if u < -81304600.0/316141.0 {-81304600.0/316141.0} else {u},
            v: if v > 54113280.0/316141.0  { 54113280.0/316141.0} else {v},
            }
        }

    pub fn toLChuv(&self) -> ColorLChuv {
        ColorLChuv::new(
            self.L,
            (self.u * self.u + self.v * self.v).sqrt(),
            self.v.atan2(self.u),
            self.clamped)
        }

    pub fn toXYZ(&self) -> ColorXYZ {
        let rdiv: f32 = REFX + REFY * 15.0 + REFZ * 3.0;
        let ur: f32 = REFX * 4.0 / rdiv;
        let vr: f32 = REFY * 9.0 / rdiv;
        let Y: f32 = 
            if self.L > 8.0 { ((self.L + 16.0) / 116.0).powi(3) }
            else { self.L * (27.0/24389.0) };

        let a: f32 = (self.L * 52.0 / (self.u + self.L * 13.0 * ur) - 1.0) / 3.0;
        let b: f32 = -5.0 * Y;
        let d: f32 = (self.L * 39.0 / (self.v + self.L * 13.0 * vr) - 5.0) * Y;

        let X: f32  = (d - b) / (a + 1.0/3.0);
        let Z: f32 = X * a + b;

        ColorXYZ::new(X, Y, Z, self.clamped)
        }
    }

pub struct ColorLChab {
    pub clamped: bool,
    pub L: f32,
    pub C: f32,
    pub h: f32,    
    }
impl Copy for ColorLChab {}
impl ColorLChab {
    pub fn new(L: f32, C: f32, h: f32, clamped: bool) -> ColorLChab {
        const maxC: f32 = 2500.0/29.sqrt();
        ColorLChab {
            clamped: clamped || L < 0.0 || L > 100.0 || C < 0.0 || C > maxC,
            L: clamp(L, 0.0, 100.0),
            C: clamp(C, 0.0, maxC),
            h: clamphue(h),
            }
        }

    pub fn toLab(&self) -> ColorLab {
        let h = clamphue(self.h);
        ColorLab::new(self.L, h.cos() * self.C, h.sin() * self.C, self.clamped)
        }
}

pub struct ColorLab {
    pub clamped: bool,
    pub L: f32,
    pub a: f32,
    pub b: f32,
    }
impl Copy for ColorLab {}
impl ColorLab {
    pub fn new(L: f32, a: f32, b: f32, clamped: bool) -> ColorLab {
        ColorLab {
            clamped: clamped || L < 0.0 || L > 100.0 || a < -12500.0/29.0 || a > 12500.0/29.0 || b < -5000.0/29.0 || b > 5000.0/29.0,
            L: clamp(L, 0.0, 100.0),
            a: clamp(a, -12500.0/29.0, 12500.0/29.0),
            b: clamp(b, -5000.0/29.0, 5000.0/29.0),
            }
        }

    pub fn toXYZ(&self) -> ColorXYZ {
        fn toXYZc(c: f32) -> f32 {
            let c3 = c * c * c;
            if c3 > 216.0 / 24389.0 { c3 }
            else { c * (108.0/841.0) - (432.0/24389.0) }
            }

        let Y = (self.L + 16.0) / 116.0;

        ColorXYZ::new(
            toXYZc(Y + self.a / 500.0) * REFX,
            toXYZc(Y) * REFY,
            toXYZc(Y - self.b / 200.0) * REFZ,
            self.clamped)
        }

    pub fn toLChab(&self) -> ColorLChab {
            ColorLChab::new(self.L,
                            (self.a * self.a + self.b * self.b).sqrt(),
                            self.b.atan2(self.a),
                            self.clamped)
        }
    }

pub struct ColorxyY {
    pub clamped: bool,
    pub x: f32,
    pub y: f32,
    pub Y: f32
    }
impl Copy for ColorxyY {}
impl ColorxyY {
    pub fn new(x: f32, y: f32, Y: f32, clamped: bool) -> ColorxyY {
        ColorxyY {
            clamped: clamped || x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0 || Y < 0.0 || Y > 1.0,
            x: clamp(x, 0.0, 1.0),
            y: clamp(y, 0.0, 1.0),
            Y: clamp(Y, 0.0, 1.0),
            }
        }   

    pub fn toXYZ(&self) -> ColorXYZ {
        if self.y != 0.0 {
            let mul = self.Y / self.y;
            ColorXYZ::new(self.x * mul, self.Y, (1.0 - self.x - self.y) * mul, self.clamped)
            }
        else { ColorXYZ::new(0.0, 0.0, 0.0, self.clamped) }
        }
    }

pub struct ColorXYZ {
    pub clamped: bool,
    pub X: f32,
    pub Y: f32,
    pub Z: f32,
    }
impl Copy for ColorXYZ {}
impl ColorXYZ {
    pub fn new(X: f32, Y: f32, Z: f32, clamped: bool) -> ColorXYZ {
        ColorXYZ {
            clamped: clamped || X < 0.0 || X > REFX || Y < 0.0 || Y > REFY || Z < 0.0 || Z > REFZ,
            X: clamp(X, 0.0, REFX),
            Y: clamp(Y, 0.0, REFY),
            Z: clamp(Z, 0.0, REFZ),
            }
        }

    pub fn toLinearRGB(&self) -> ColorLinearRGB {
        ColorLinearRGB::new(
            self.X * (641589.0/197960.0)      + self.Y * (-608687.0/395920.0)    + self.Z * (-49353.0/98980.0),
            self.X * (-42591639.0/43944050.0) + self.Y * (82435961.0/43944050.0) + self.Z * (1826061.0/43944050.0),
            self.X * (49353.0/887015.0)       + self.Y * (-180961.0/887015.0)    + self.Z * (49353.0/46685.0),
            self.clamped)
        }

    pub fn toxyY(&self) -> ColorxyY {
        let mut div = self.X + self.Y + self.Z;

        if div == 0.0 { div = 1.0; };

        ColorxyY::new(self.X / div, self.Y / div, self.Y, self.clamped)
        }

    pub fn toLab(&self) -> ColorLab {
        fn toLabc(c: f32) -> f32 {
            if c > 216.0 / 24389.0 { c.cbrt() }
            else { c * (841.0 / 108.0) + (4.0 / 49.0) }
            }

        let X = toLabc(self.X / REFX);
        let Y = toLabc(self.Y / REFY);
        let Z = toLabc(self.Z / REFZ);

        ColorLab::new(116.0 * Y - 16.0, 500.0 * (X - Y), 200.0 * (Y - Z), self.clamped)
        }

    pub fn toLuv(&self) -> ColorLuv {
        let rdiv = REFX + REFY * 15.0 + REFZ * 3.0;
        let ur = REFX * 4.0 / rdiv;
        let vr = REFY * 9.0 / rdiv;

        let mut div = self.X + self.Y * 15.0 + self.Z * 3.0;

        if div == 0.0 { div = 1.0; };

        let u = self.X * 4.0 / div;
        let v = self.Y * 9.0 / div;
        let yr = self.Y / REFY;

        let L = if yr > 216.0/24389.0 { yr.cbrt() * 116.0 - 16.0 }
                else { yr * (24389.0/27.0) };

        ColorLuv::new(L, L * 13.0 * (u - ur), L * 13.0 * (v - vr), self.clamped)
        }
    }

pub struct ColorLinearRGB {
    pub clamped: bool,
    pub R: f32,
    pub G: f32,
    pub B: f32,
    }
impl Copy for ColorLinearRGB {}
impl std::fmt::String for ColorLinearRGB {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "#{:02X}{:02X}{:02X}", (self.R*255.0) as u8, (self.G*255.0) as u8, (self.B*255.0) as u8)
        }
    }
impl ColorLinearRGB {
    pub fn new(R: f32, G: f32, B: f32, clamped: bool) -> ColorLinearRGB {
        ColorLinearRGB {
            clamped: clamped || R < 0.0 || R > 1.0 || G < 0.0 || G > 1.0 || B < 0.0 || B > 1.0,
            R: clamp(R, 0.0, 1.0),
            G: clamp(G, 0.0, 1.0),
            B: clamp(B, 0.0, 1.0),
            }
        }

    pub fn toRGB(&self) -> ColorRGB {
        fn toRGBc(c: f32) -> f32 {
            if c > 0.0031308 { c.powf(1.0 / 2.4) * 1.055 - 0.055 }
            else { c * 12.92 }
            }

        ColorRGB::new(toRGBc(self.R), toRGBc(self.G), toRGBc(self.B), self.clamped)
        }

    pub fn toXYZ(&self) -> ColorXYZ {
        ColorXYZ::new(
            self.R * (5067776.0/12288897.0) + self.G * (4394405.0/12288897.0) + self.B * (4435075.0/24577794.0),
            self.R * (871024.0/4096299.0)   + self.G * (8788810.0/12288897.0) + self.B * (887015.0/12288897.0),
            self.R * (79184.0/4096299.0)    + self.G * (4394405.0/36866691.0) + self.B * (70074185.0/73733382.0),
            self.clamped)
        }
    }

pub struct ColorHSV {
    pub clamped: bool,
    pub H: f32,
    pub S: f32,
    pub V: f32,
    }
impl Copy for ColorHSV {}
impl ColorHSV {
    pub fn new(H: f32, S: f32, V: f32, clamped: bool) -> ColorHSV {
        ColorHSV {
            clamped: clamped || S < 0.0 || S > 1.0 || V < 0.0 || V > 1.0,
            H: clamphue(H),
            S: clamp(S, 0.0, 1.0),
            V: clamp(V, 0.0, 1.0),
            }
        }

    pub fn toRGB(&self) -> ColorRGB {
        if self.S <= 0.0 {
            return ColorRGB::new(self.V, self.V, self.V, self.clamped);
            }

        let H = clamphue(self.H) / 60.0;
        let mut C = self.V * self.S;
        let m = self.V - C;
        let X = C * (1.0 - (H % 2.0 - 1.0).abs()) + m;

        C += m;

        if H >= 5.0 { ColorRGB::new(C, m, X, self.clamped) }
        else if H >= 4.0 { ColorRGB::new(X, m, C, self.clamped) }
        else if H >= 3.0 { ColorRGB::new(m, X, C, self.clamped) }
        else if H >= 2.0 { ColorRGB::new(m, C, X, self.clamped) }
        else if H >= 1.0 { ColorRGB::new(X, C, m, self.clamped) }
        else { ColorRGB::new(C, X, m, self.clamped) }
        }
    }

pub struct ColorHSL {
    pub clamped: bool,
    pub H: f32,
    pub S: f32,
    pub L: f32,
    }
impl Copy for ColorHSL {}
impl ColorHSL {
    pub fn new(H: f32, S: f32, L: f32, clamped: bool) -> ColorHSL {
        ColorHSL {
            clamped: clamped || S < 0.0 || S > 1.0 || L < 0.0 || L > 1.0,
            H: clamphue(H),
            S: clamp(S, 0.0, 1.0),
            L: clamp(L, 0.0, 1.0),
            }
        }

    pub fn toRGB(&self) -> ColorRGB {
        if self.S <= 0.0 {
            return ColorRGB::new(self.S, self.S, self.S, self.clamped)
            }

        let H = clamphue(self.H) / 60.0;
        let mut C = (1.0 - (self.L * 2.0 - 1.0).abs()) * self.S;
        let m = self.L - C * 0.5;
        let X = C * (1.0 - (H % 2.0 - 1.0).abs()) + m;

        C += m;

        if H >= 5.0 { ColorRGB::new(C, m, X, self.clamped) }
        else if H >= 4.0 { ColorRGB::new(X, m, C, self.clamped) }
        else if H >= 3.0 { ColorRGB::new(m, X, C, self.clamped) }
        else if H >= 2.0 { ColorRGB::new(m, C, X, self.clamped) }
        else if H >= 1.0 { ColorRGB::new(X, C, m, self.clamped) }
        else { ColorRGB::new(C, X, m, self.clamped) }
        }
    }

pub struct ColorRGB {
    pub clamped: bool,
    pub R: f32,
    pub G: f32,
    pub B: f32,
    }
impl Copy for ColorRGB {}
impl ColorRGB {
    pub fn new(R: f32, G: f32, B: f32, clamped: bool) -> ColorRGB {
        ColorRGB {
            clamped: clamped || R < 0.0 || R > 1.0 || G < 0.0 || G > 1.0 || B < 0.0 || B > 1.0,
            R: clamp(R, 0.0, 1.0),
            G: clamp(G, 0.0, 1.0),
            B: clamp(B, 0.0, 1.0),
            }
        }

    pub fn toHSV(&self) -> ColorHSV {
        let min = self.R.min(self.G).min(self.B);
        let max = self.R.max(self.G).max(self.B);
        let delta = max - min;

        let S = if delta != 0.0 { delta / max } else {0.0};

        let H = if delta != 0.0 {
                    if max == self.R { (self.G - self.B) / delta }
                    else if max == self.G { (self.B - self.R) / delta + 2.0 }
                    else { (self.R - self.G) / delta + 4.0 }
                    }
                else {0.0};

        ColorHSV::new(H * 60.0, S, max, self.clamped)
        }

    pub fn toHSL(&self) -> ColorHSL {
        let min = self.R.min(self.G).min(self.B);
        let max = self.R.max(self.G).max(self.B);
        let delta = max - min;

        let L = (max + min) * 0.5;
        let S = if delta == 0.0 {0.0}
                else if L < 0.5 { delta / (max + min) }
                else { delta / (2.0 - max - min) };

        let H = if delta == 0.0 {0.0}
                else if max == self.R { (self.G - self.B) / delta }
                else if max == self.G { (self.B - self.R) / delta + 2.0 }
                else { (self.R - self.G) / delta + 4.0 };

        ColorHSL::new(H * 60.0, S, L, self.clamped)
        }

    pub fn toLinearRGB(&self) -> ColorLinearRGB {
        fn toLinearRGBc(c: f32) -> f32 {
            if c > 0.04045 { ((c + 0.055) / 1.055).powf(2.4) }
            else { c / 12.92 }
            }

        ColorLinearRGB::new(toLinearRGBc(self.R), toLinearRGBc(self.G), toLinearRGBc(self.B), self.clamped)
        }

    pub fn toYUV(&self, yuvmatrix: yuvmatrices::YUVMatrix) -> ColorYUV {
        let mat = yuvmatrix;

        let y = self.R * mat.rScale + self.G * mat.gScale + self.B * mat.bScale;
        let u = (self.B - y) / (1.0 - mat.bScale) * 0.5 + 0.5;
        let v = (self.R - y) / (1.0 - mat.rScale) * 0.5 + 0.5;

        ColorYUV::new(y, u, v, self.clamped)
        }

    pub fn toYIQ(&self) -> ColorYIQ {
        ColorYIQ::new(
            self.R *  0.299                + self.G *  0.587               + self.B *  0.114,
            self.R *  0.5                  + self.G * -0.23038159508364756 + self.B * -0.26961840491635244  + 0.5,
            self.R * -0.202349432337541121 + self.G *  0.5                 + self.B * -0.297650567662458879 + 0.5,
            self.clamped)
        }
    }

pub struct ColorYUV {
    pub clamped: bool,
    pub Y: f32,
    pub U: f32,
    pub V: f32
    }
impl Copy for ColorYUV {}
impl ColorYUV{
    pub fn new(Y: f32, U: f32, V: f32, clamped: bool) -> ColorYUV  {
        ColorYUV {
            clamped: clamped || Y < 0.0 || Y > 1.0 || U < 0.0 || V > 1.0 || V < 0.0 || V > 1.0,
            Y: clamp(Y, 0.0, 1.0),
            U: clamp(U, 0.0, 1.0),
            V: clamp(V, 0.0, 1.0),
            }
        }

    pub fn toRGB(&self, yuvmatrix: yuvmatrices::YUVMatrix) -> ColorRGB {
        let mat = yuvmatrix;

        let u = (self.U - 0.5) / 0.5 * (1.0 - mat.bScale);
        let v = (self.V - 0.5) / 0.5 * (1.0 - mat.rScale);

        let r = v + self.Y;
        let b = u + self.Y;
        let g = (self.Y - r * mat.rScale - b * mat.bScale) / mat.gScale;

        ColorRGB::new(r, g, b, self.clamped)
        }
    }

pub struct ColorYIQ {
    pub clamped: bool,
    pub Y: f32,
    pub I: f32,
    pub Q: f32,
    }
impl Copy for ColorYIQ {}
impl ColorYIQ {
    pub fn new(Y: f32, I: f32, Q: f32, clamped: bool) -> ColorYIQ {
        ColorYIQ {
            clamped: clamped || Y < 0.0 || Y > 1.0 || I < 0.0 || I > 1.0 || Q < 0.0 || Q > 1.0,
            Y: clamp(Y, 0.0, 1.0),
            I: clamp(I, 0.0, 1.0),
            Q: clamp(Q, 0.0, 1.0),
            }
        }

    pub fn toRGB(&self) -> ColorRGB {
        let i = self.I - 0.5;
        let q = self.Q - 0.5;

        ColorRGB::new(
            self.Y + i * 1.13933588212202582 - q * 0.649035964281386078,
            self.Y - i * 0.32416610079155499 + q * 0.676636193255190191,
            self.Y - i * 1.31908708412142932 - q * 1.78178677298826495,
            self.clamped)
        }
    }

// CIE Delta E 1976
// JND: ~2.3
pub fn deltaE1976(lab1: &ColorLab, lab2: &ColorLab) -> f32 {
    let delta_L = lab1.L - lab2.L;
    let delta_a = lab1.a - lab2.a;
    let delta_b = lab1.b - lab2.b;

    (delta_L * delta_L + delta_a * delta_a + delta_b * delta_b).sqrt()
    }

pub enum DE1994Type {
    GraphicArts,
    Textiles
    }
impl Copy for DE1994Type {}

// CIE Delta E 1994
pub fn deltaE1994(lab1: &ColorLab, lab2: &ColorLab, t: DE1994Type) -> f32 {
    let C1 = (lab1.a * lab1.a + lab1.b * lab1.b).sqrt();
    let C2 = (lab2.a * lab2.a + lab2.b * lab2.b).sqrt();

    let mut delta_L = lab1.L - lab2.L;
    let mut delta_C = C1 - C2;
    let delta_a = lab1.a - lab2.a;
    let delta_b = lab1.b - lab2.b;
    let mut delta_H = (delta_a * delta_a + delta_b * delta_b - delta_C * delta_C).sqrt();

    match t {
        DE1994Type::GraphicArts => {
            delta_C /= C1 * 0.045 + 1.0;
            delta_H /= C1 * 0.015 + 1.0;
            },
        DE1994Type::Textiles => {
            delta_L *= 0.5;
            delta_C /= C1 * 0.048 + 1.0;
            delta_H /= C1 * 0.014 + 1.0;
            },
        }

    (delta_L * delta_L + delta_C * delta_C + delta_H * delta_H).sqrt()
    }

// CIE Delta E 2000
// Note: maximum is about 158.0 for colors in the sRGB gamut.
pub fn deltaE2000(lab1: &ColorLab, lab2: &ColorLab) -> f32 {
    let lch1 = lab1.toLChab();
    let lch2 = lab2.toLChab();
    
    let avg_L = (lch1.L + lch2.L) * 0.5;
    let mut delta_L = lch2.L - lch1.L;

    let avg_C = (lch1.C + lch2.C) * 0.5;
    let mut delta_C = lch1.C - lch2.C;

    let mut avg_H = (lch1.h + lch2.h) * 0.5;
    if (lch1.h - lch2.h).abs() > f32::consts::PI { avg_H += f32::consts::PI };

    let mut delta_H = lch2.h - lch1.h;

    if delta_H.abs() > f32::consts::PI {
        if lch2.h <= lch1.h { delta_H += f32::consts::PI * 2.0 }
        else { delta_H -= f32::consts::PI * 2.0 }
    }

    delta_H = (lch1.C * lch2.C).sqrt() * delta_H.sin() * 2.0;

    let T = 1.0
        - 0.17 * (avg_H - f32::consts::PI / 6.0).cos()
        + 0.24 * (avg_H * 2.0).cos()
        + 0.32 * (avg_H * 3.0 + f32::consts::PI / 30.0).cos()
        - 0.20 * (avg_H * 4.0 - f32::consts::PI * 7.0/20.0).cos();

    let mut SL = avg_L - 50.0;
    SL *= SL;
    SL = SL * 0.015 / (SL + 20.0).sqrt() + 1.0;

    let SC = avg_C * 0.045 + 1.0;

    let SH = avg_C * T * 0.015 + 1.0;

    let mut delta_Theta = avg_H / 25.0 - f32::consts::PI * 11.0/180.0;
    delta_Theta = (delta_Theta * -delta_Theta).exp() * (f32::consts::PI / 6.0);

    let mut RT = avg_C.powi(7);
    RT = (RT / (RT + 25.0.powi(7))).sqrt() * delta_Theta.sin() * -2.0;

    delta_L /= SL;
    delta_C /= SC;
    delta_H /= SH;

    (delta_L * delta_L + delta_C * delta_C + delta_H * delta_H + RT * delta_C * delta_H).sqrt()
    }
