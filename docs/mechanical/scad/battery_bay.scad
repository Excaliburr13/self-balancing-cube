// Self-Balancing Cube — Battery Bay
// Fits: 3S 1300mAh LiPo (~105 x 34 x 22mm)
// Velcro strap slots on both sides
// Print: PETG, 40% infill

$fn = 32;

lipo_l  = 105.0;
lipo_w  = 34.0;
lipo_h  = 22.0;
wall    = 2.5;
strap_w = 15;
strap_t = 2;
tol     = 0.5;   // clearance

inner_l = lipo_l + tol;
inner_w = lipo_w + tol;
outer_l = inner_l + 2*wall;
outer_w = inner_w + 2*wall;
outer_h = lipo_h + wall;

difference() {
    // Outer shell (3 sides + bottom, top open)
    cube([outer_l, outer_w, outer_h], center=true);

    // Inner cavity
    translate([0, 0, wall])
        cube([inner_l, inner_w, outer_h], center=true);

    // Wire exit slot (front)
    translate([outer_l/2, 0, outer_h/4])
        cube([wall*2 + 1, 20, lipo_h/2], center=true);

    // Velcro strap slots (top, both sides)
    for (y = [-lipo_w/4, lipo_w/4]) {
        translate([0, y, outer_h/2])
            cube([outer_l + 1, strap_w, strap_t*2], center=true);
    }

    // Mounting holes (bottom, M3)
    for (x = [-lipo_l/3, lipo_l/3]) {
        translate([x, 0, -outer_h/2])
            cylinder(h=wall + 1, d=3.2);
    }
}