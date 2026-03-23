// Self-Balancing Cube — Cable Guide Clip
// Snap-fit clip for 2020 aluminum extrusion
// Holds 3x motor phase wires + SPI bundle
// Print: TPU or flexible PETG for snap-fit

$fn = 32;

extrusion_w = 20.0;
clip_t      = 2.5;
channel_d   = 8;      // cable bundle diameter
arm_l       = 6;
snap_t      = 1.2;

difference() {
    union() {
        // Back plate
        cube([extrusion_w + 2*clip_t, clip_t, extrusion_w + 2*clip_t], center=true);

        // Snap arms (top and bottom)
        translate([0, clip_t/2, extrusion_w/2 + clip_t/2])
            cube([extrusion_w - 2, arm_l, snap_t], center=true);
        translate([0, clip_t/2, -extrusion_w/2 - clip_t/2])
            cube([extrusion_w - 2, arm_l, snap_t], center=true);

        // Cable channel holder
        translate([0, -(clip_t + channel_d)/2, 0])
            difference() {
                cylinder(h=extrusion_w + 2*clip_t, d=channel_d + 2*clip_t, center=true);
                cylinder(h=extrusion_w + 2*clip_t + 1, d=channel_d, center=true);
                translate([0, channel_d, 0])
                    cube([channel_d + 2, channel_d*2, extrusion_w + 3], center=true);
            }
    }

    // Extrusion slot
    cube([extrusion_w + 0.4, clip_t + 2, extrusion_w + 0.4], center=true);
}