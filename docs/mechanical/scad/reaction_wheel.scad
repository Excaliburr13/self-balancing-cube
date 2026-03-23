// Self-Balancing Cube — Reaction Wheel
// Radius: 30.0mm, Thickness: 6.0mm
// Mass concentrated at rim for max rotational inertia
// Print: PETG, 80% infill at rim, 20% center
// No supports needed

$fn = 128;

wheel_r    = 30.0;
wheel_t    = 6.0;
hub_d      = 8.0;
shaft_d    = 3.4;
rim_w      = 8;          // rim width
spoke_w    = 4;          // spoke width
n_spokes   = 5;
set_screw_d= 3;          // M3 set screw

difference() {
    union() {
        // Outer rim — mass concentrated here for inertia
        difference() {
            cylinder(h=wheel_t, d=wheel_r*2, center=true);
            cylinder(h=wheel_t+1, d=(wheel_r-rim_w)*2, center=true);
        }

        // Hub
        cylinder(h=wheel_t, d=hub_d, center=true);

        // Spokes
        for (i = [0:n_spokes-1]) {
            rotate([0, 0, i * (360/n_spokes)])
                translate([0, 0, 0])
                    rotate([0, 0, 0])
                        cube([wheel_r*2 - rim_w, spoke_w, wheel_t], center=true);
        }
    }

    // Shaft bore (D-flat for grub screw)
    cylinder(h=wheel_t+2, d=shaft_d + 0.2, center=true);

    // Grub screw hole (M3, radial)
    translate([hub_d/2 + set_screw_d/2, 0, 0])
        rotate([0, 90, 0])
            cylinder(h=hub_d, d=set_screw_d, center=true);
}