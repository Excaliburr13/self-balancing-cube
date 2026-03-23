// Self-Balancing Cube — IMU Mount
// Fits: ICM-42688-P breakout (~20x20mm)
// Mounts at cube center of mass
// Print: PLA, 30% infill

$fn = 32;

board_l    = 22;
board_w    = 22;
standoff_h = 4;
standoff_d = 5;
wall_t     = 2;
corner_r   = 2;

difference() {
    union() {
        // Base plate with rounded corners
        minkowski() {
            cube([board_l, board_w, wall_t], center=true);
            cylinder(h=0.1, r=corner_r);
        }

        // 4 standoffs
        for (x = [-board_l/2 + 2, board_l/2 - 2]) {
            for (y = [-board_w/2 + 2, board_w/2 - 2]) {
                translate([x, y, wall_t/2])
                    cylinder(h=standoff_h, d=standoff_d);
            }
        }
    }

    // M2 holes in standoffs
    for (x = [-board_l/2 + 2, board_l/2 - 2]) {
        for (y = [-board_w/2 + 2, board_w/2 - 2]) {
            translate([x, y, -1])
                cylinder(h=standoff_h + wall_t + 2, d=2.2);
        }
    }

    // SPI cable exit
    translate([0, -board_w/2 - 1, 0])
        cube([12, 4, wall_t + 2], center=true);
}