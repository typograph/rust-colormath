extern crate colors;

const SUBDIV: usize = 256;
const NDIV: usize = 3;
const N: usize = 1 + NDIV*NDIV*NDIV;

fn main() {
//     let mut rgb_clrs: [[[colors::ColorLinearRGB; SUBDIV]; SUBDIV]; SUBDIV] = [[[colors::ColorLinearRGB { clamped: true, R:0.0, G: 0.0, B: 0.0}; SUBDIV]; SUBDIV]; SUBDIV];
//     let mut lab_clrs: [[[colors::ColorLab; SUBDIV]; SUBDIV]; SUBDIV] = [[[colors::ColorLab { clamped: true, L:0.0, a: 0.0, b: 0.0}; SUBDIV]; SUBDIV]; SUBDIV];
//     
//     for r in 0..(SUBDIV) {
//         for g in 0..(SUBDIV) {
//             for b in 0..(SUBDIV) {
//                 let rgb: colors::ColorLinearRGB = 
//                     colors::ColorLinearRGB::new(
//                         (r as f32)/(SUBDIV as f32),
//                         (g as f32)/(SUBDIV as f32),
//                         (b as f32)/(SUBDIV as f32),
//                         true,
//                         );
//                 rgb_clrs[r][g][b] = rgb;
//                 lab_clrs[r][g][b] = rgb.toXYZ().toLab();
//                 };
//             };
//         };

    let background: colors::ColorLab = colors::ColorLinearRGB::new(0.9, 0.9, 0.9, true).toXYZ().toLab();
    
    let mut dcolors: [colors::ColorLab; N] = [colors::ColorLab { clamped: true, L:0.0, a: 0.0, b: 0.0}; N];
    dcolors[0] = background;
    for r in 0..(NDIV) {
        for g in 0..(NDIV) {
            for b in 0..(NDIV) {
                let rgb: colors::ColorLinearRGB = 
                    colors::ColorLinearRGB::new(
                        (r as f32)/(NDIV as f32),
                        (g as f32)/(NDIV as f32),
                        (b as f32)/(NDIV as f32),
                        true,
                        );
                dcolors[1 + (r*NDIV + g)*NDIV + b] = rgb.toXYZ().toLab();
                };
            };
        };

//     for i in 0..(N) {
//         println!("{}",dcolors[i].toXYZ().toLinearRGB());
//         }
       
    let mut maxshift: f32 = 100.0;
    
    while maxshift > 0.1 {
        maxshift = 0.0;

        let mut centroids = [[0 as usize; 3]; N];
        let mut counts = [0; N];

        for r in 0..(SUBDIV) {
            for g in 0..(SUBDIV) {
                for b in 0..(SUBDIV) {
                    let mut mind = 10000.0;
                    let mut mini = 0;
                    let lab = 
                        colors::ColorLinearRGB::new(
                            (r as f32)/(SUBDIV as f32),
                            (g as f32)/(SUBDIV as f32),
                            (b as f32)/(SUBDIV as f32),
                            true,
                            ).toXYZ().toLab();

                    for i in 0..(N) {
                        let dst = colors::deltaE1976(&dcolors[i], &lab);
                        if dst < mind {
                            mind = dst;
                            mini = i;
                            }
                        }
                    centroids[mini][0] += r;
                    centroids[mini][1] += g;
                    centroids[mini][2] += b;
                    counts[mini] += 1;
                    };
                };
            };
                        
        for i in 1..(N) {
            let ncolor = colors::ColorLinearRGB::new(
                (centroids[i][0] as f32)/(counts[i] as f32)/(SUBDIV as f32),
                (centroids[i][1] as f32)/(counts[i] as f32)/(SUBDIV as f32),
                (centroids[i][2] as f32)/(counts[i] as f32)/(SUBDIV as f32),
                true
                ).toXYZ().toLab();
            let shift = colors::deltaE1976(&dcolors[i], &ncolor);
            if shift > maxshift { maxshift = shift };
            dcolors[i] = ncolor;
            }
            
        println!("Shifted by {}", maxshift);

//         for i in 0..(N) {
//             print!("{} ",dcolors[i].toXYZ().toLinearRGB());
//             }
        }

    print!("[");
    for i in 0..(N) {
        print!("'{}', ",dcolors[i].toXYZ().toLinearRGB());
        }
    println!("]");

    }
    
