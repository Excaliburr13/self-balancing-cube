// Self-Balancing Cube — Corner Bracket
// Reinforces 90-degree joints in 2020 aluminum frame
// Print: PETG or ABS, 60% infill for strength

$fn = 32;

arm_l  = 30;
arm_w  = 20.0;
arm_t  = 3;
bolt_d = 4.5;   // M4 clearance

difference() {
    union() {
        // Arm 1
        cube([arm_l, arm_w, arm_t]);

        // Arm 2 (perpendicular)
        cube([arm_t, arm_w, arm_l]);
    }

    // Bolt holes arm 1
    for (x = [arm_l/3, 2*arm_l/3]) {
        translate([x, arm_w/2, -1])
            cylinder(h=arm_t + 2, d=bolt_d);
    }

    // Bolt holes arm 2
    for (z = [arm_l/3, 2*arm_l/3]) {
        translate([-1, arm_w/2, z])
            rotate([0, 90, 0])
                cylinder(h=arm_t + 2, d=bolt_d);
    }
}