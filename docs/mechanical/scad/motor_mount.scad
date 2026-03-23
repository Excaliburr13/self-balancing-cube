// Self-Balancing Cube — Motor Mount
// Fits: 2204 gimbal motor (27.85mm dia)
// Mounts to: 20x20mm aluminum extrusion (M4 T-nuts)
// Print: PETG or ABS, 40% infill, 3 perimeters
// Layer height: 0.2mm
// Supports: yes (for motor cavity)

$fn = 64;

motor_d      = 27.85;
motor_h      = 13.1;
shaft_d      = 3.4;
bolt_d       = 3.0;
bcd          = 16.0;
base_t       = 6;        // base plate thickness
base_l       = 40;       // base plate length
base_w       = 30;       // base plate width
extrusion_w  = 20.0;
slot_w       = 6;        // T-nut slot width
slot_h       = 3.4;      // T-nut slot height

difference() {
    union() {
        // Base plate
        cube([base_l, base_w, base_t], center=true);

        // Motor collar
        translate([0, 0, base_t/2])
            cylinder(h=motor_h + 2, d=motor_d + 6, center=false);
    }

    // Motor cavity
    translate([0, 0, base_t/2 + 1])
        cylinder(h=motor_h + 4, d=motor_d + 0.4, center=false);

    // Shaft clearance through base
    cylinder(h=base_t + 2, d=shaft_d + 1, center=true);

    // Motor mounting bolts (M3) on BCD
    for (i = [0:3]) {
        angle = i * 90;
        translate([bcd/2 * cos(angle), bcd/2 * sin(angle), 0])
            cylinder(h=base_t + motor_h + 4, d=bolt_d, center=true);
    }

    // Frame mounting slots (M4 T-nuts)
    translate([0, 0, -base_t/2 - 0.1]) {
        translate([-base_l/4, 0, 0])
            cylinder(h=base_t + 0.2, d=4.5);
        translate([base_l/4, 0, 0])
            cylinder(h=base_t + 0.2, d=4.5);
    }
}