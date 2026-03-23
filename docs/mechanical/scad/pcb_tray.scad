// Self-Balancing Cube — PCB Tray (Nucleo F446RE)
// PCB: 70mm x 50mm
// M3 standoffs at 4 corners
// Mounts to aluminum frame bottom via T-nuts
// Print: PLA or PETG, 30% infill

$fn = 32;

pcb_l       = 70.0;
pcb_w       = 50.0;
pcb_t       = 1.6;
standoff_h  = 5.0;
standoff_d  = 6;
tray_t      = 2;
margin      = 5;

tray_l = pcb_l + 2*margin;
tray_w = pcb_w + 2*margin;

difference() {
    union() {
        // Tray base
        cube([tray_l, tray_w, tray_t], center=true);

        // Standoffs at PCB corners
        corner_x = pcb_l/2 - 3.5;
        corner_y = pcb_w/2 - 3.5;
        for (x = [-corner_x, corner_x]) {
            for (y = [-corner_y, corner_y]) {
                translate([x, y, tray_t/2])
                    cylinder(h=standoff_h, d=standoff_d);
            }
        }

        // Edge rails to retain PCB
        translate([0, tray_w/2 - 2, tray_t/2 + 1])
            cube([tray_l, 2, 3], center=true);
        translate([0, -tray_w/2 + 2, tray_t/2 + 1])
            cube([tray_l, 2, 3], center=true);
    }

    // M3 holes in standoffs
    corner_x = pcb_l/2 - 3.5;
    corner_y = pcb_w/2 - 3.5;
    for (x = [-corner_x, corner_x]) {
        for (y = [-corner_y, corner_y]) {
            translate([x, y, -1])
                cylinder(h=standoff_h + tray_t + 2, d=3.2);
        }
    }

    // USB access cutout (side)
    translate([tray_l/2, 0, tray_t/2 + 1])
        cube([6, 14, 8], center=true);

    // Frame mounting holes (M4)
    for (x = [-tray_l/2 + 8, tray_l/2 - 8]) {
        translate([x, 0, 0])
            cylinder(h=tray_t + 2, d=4.5, center=true);
    }
}