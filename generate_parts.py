"""
Self-Balancing Cube — 3D Print Parts Generator
===============================================
Generates OpenSCAD source files for all printable parts.
Install OpenSCAD (free) to render and export STL files.

Parts generated:
  1. motor_mount.scad    — motor mount for 2204 gimbal motor (×3)
  2. reaction_wheel.scad — reaction wheel disc (×3)
  3. pcb_tray.scad       — Nucleo F446RE PCB mounting tray
  4. battery_bay.scad    — LiPo battery holder with strap slots
  5. cable_guide.scad    — internal cable routing clip (×6)
  6. corner_bracket.scad — aluminum frame corner reinforcement (×8)
  7. imu_mount.scad      — IMU breakout board mount

Run: python generate_parts.py
Output: docs/mechanical/scad/*.scad
"""

import os

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'docs', 'mechanical', 'scad')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ─── Physical constants ────────────────────────────────────────
CUBE_SIDE     = 150.0   # mm
WALL_T        = 3.0     # mm wall thickness
FRAME_W       = 20.0    # mm 2020 extrusion width

# Motor 2204 dims
MOTOR_D       = 27.85   # mm motor body diameter
MOTOR_H       = 13.1    # mm motor body height
MOTOR_SHAFT_D = 3.4     # mm shaft hole
MOTOR_BOLT_D  = 3.0     # mm M3 mounting bolt
MOTOR_BCD     = 16.0    # mm bolt circle diameter

# Reaction wheel
WHEEL_R       = 30.0    # mm wheel radius
WHEEL_T       = 6.0     # mm wheel thickness
WHEEL_HUB_D   = 8.0     # mm hub diameter

# PCB dims (Nucleo F446RE)
PCB_L         = 70.0    # mm
PCB_W         = 50.0    # mm
PCB_T         = 1.6     # mm
PCB_STANDOFF  = 5.0     # mm standoff height

# LiPo dims (1300mAh 3S)
LIPO_L        = 105.0
LIPO_W        = 34.0
LIPO_H        = 22.0

# ─── Part generators ──────────────────────────────────────────

def write_motor_mount():
    """Motor mount for 2204 gimbal motor — mounts to 2020 extrusion."""
    scad = f"""
// Self-Balancing Cube — Motor Mount
// Fits: 2204 gimbal motor (27.85mm dia)
// Mounts to: 20x20mm aluminum extrusion (M4 T-nuts)
// Print: PETG or ABS, 40% infill, 3 perimeters
// Layer height: 0.2mm
// Supports: yes (for motor cavity)

$fn = 64;

motor_d      = {MOTOR_D};
motor_h      = {MOTOR_H};
shaft_d      = {MOTOR_SHAFT_D};
bolt_d       = {MOTOR_BOLT_D};
bcd          = {MOTOR_BCD};
base_t       = 6;        // base plate thickness
base_l       = 40;       // base plate length
base_w       = 30;       // base plate width
extrusion_w  = {FRAME_W};
slot_w       = 6;        // T-nut slot width
slot_h       = 3.4;      // T-nut slot height

difference() {{
    union() {{
        // Base plate
        cube([base_l, base_w, base_t], center=true);

        // Motor collar
        translate([0, 0, base_t/2])
            cylinder(h=motor_h + 2, d=motor_d + 6, center=false);
    }}

    // Motor cavity
    translate([0, 0, base_t/2 + 1])
        cylinder(h=motor_h + 4, d=motor_d + 0.4, center=false);

    // Shaft clearance through base
    cylinder(h=base_t + 2, d=shaft_d + 1, center=true);

    // Motor mounting bolts (M3) on BCD
    for (i = [0:3]) {{
        angle = i * 90;
        translate([bcd/2 * cos(angle), bcd/2 * sin(angle), 0])
            cylinder(h=base_t + motor_h + 4, d=bolt_d, center=true);
    }}

    // Frame mounting slots (M4 T-nuts)
    translate([0, 0, -base_t/2 - 0.1]) {{
        translate([-base_l/4, 0, 0])
            cylinder(h=base_t + 0.2, d=4.5);
        translate([base_l/4, 0, 0])
            cylinder(h=base_t + 0.2, d=4.5);
    }}
}}
"""
    _write('motor_mount.scad', scad)


def write_reaction_wheel():
    """Reaction wheel disc — attaches to motor shaft."""
    scad = f"""
// Self-Balancing Cube — Reaction Wheel
// Radius: {WHEEL_R}mm, Thickness: {WHEEL_T}mm
// Mass concentrated at rim for max rotational inertia
// Print: PETG, 80% infill at rim, 20% center
// No supports needed

$fn = 128;

wheel_r    = {WHEEL_R};
wheel_t    = {WHEEL_T};
hub_d      = {WHEEL_HUB_D};
shaft_d    = {MOTOR_SHAFT_D};
rim_w      = 8;          // rim width
spoke_w    = 4;          // spoke width
n_spokes   = 5;
set_screw_d= 3;          // M3 set screw

difference() {{
    union() {{
        // Outer rim — mass concentrated here for inertia
        difference() {{
            cylinder(h=wheel_t, d=wheel_r*2, center=true);
            cylinder(h=wheel_t+1, d=(wheel_r-rim_w)*2, center=true);
        }}

        // Hub
        cylinder(h=wheel_t, d=hub_d, center=true);

        // Spokes
        for (i = [0:n_spokes-1]) {{
            rotate([0, 0, i * (360/n_spokes)])
                translate([0, 0, 0])
                    rotate([0, 0, 0])
                        cube([wheel_r*2 - rim_w, spoke_w, wheel_t], center=true);
        }}
    }}

    // Shaft bore (D-flat for grub screw)
    cylinder(h=wheel_t+2, d=shaft_d + 0.2, center=true);

    // Grub screw hole (M3, radial)
    translate([hub_d/2 + set_screw_d/2, 0, 0])
        rotate([0, 90, 0])
            cylinder(h=hub_d, d=set_screw_d, center=true);
}}
"""
    _write('reaction_wheel.scad', scad)


def write_pcb_tray():
    """PCB tray for STM32 Nucleo F446RE."""
    scad = f"""
// Self-Balancing Cube — PCB Tray (Nucleo F446RE)
// PCB: 70mm x 50mm
// M3 standoffs at 4 corners
// Mounts to aluminum frame bottom via T-nuts
// Print: PLA or PETG, 30% infill

$fn = 32;

pcb_l       = {PCB_L};
pcb_w       = {PCB_W};
pcb_t       = {PCB_T};
standoff_h  = {PCB_STANDOFF};
standoff_d  = 6;
tray_t      = 2;
margin      = 5;

tray_l = pcb_l + 2*margin;
tray_w = pcb_w + 2*margin;

difference() {{
    union() {{
        // Tray base
        cube([tray_l, tray_w, tray_t], center=true);

        // Standoffs at PCB corners
        corner_x = pcb_l/2 - 3.5;
        corner_y = pcb_w/2 - 3.5;
        for (x = [-corner_x, corner_x]) {{
            for (y = [-corner_y, corner_y]) {{
                translate([x, y, tray_t/2])
                    cylinder(h=standoff_h, d=standoff_d);
            }}
        }}

        // Edge rails to retain PCB
        translate([0, tray_w/2 - 2, tray_t/2 + 1])
            cube([tray_l, 2, 3], center=true);
        translate([0, -tray_w/2 + 2, tray_t/2 + 1])
            cube([tray_l, 2, 3], center=true);
    }}

    // M3 holes in standoffs
    corner_x = pcb_l/2 - 3.5;
    corner_y = pcb_w/2 - 3.5;
    for (x = [-corner_x, corner_x]) {{
        for (y = [-corner_y, corner_y]) {{
            translate([x, y, -1])
                cylinder(h=standoff_h + tray_t + 2, d=3.2);
        }}
    }}

    // USB access cutout (side)
    translate([tray_l/2, 0, tray_t/2 + 1])
        cube([6, 14, 8], center=true);

    // Frame mounting holes (M4)
    for (x = [-tray_l/2 + 8, tray_l/2 - 8]) {{
        translate([x, 0, 0])
            cylinder(h=tray_t + 2, d=4.5, center=true);
    }}
}}
"""
    _write('pcb_tray.scad', scad)


def write_battery_bay():
    """LiPo battery holder."""
    scad = f"""
// Self-Balancing Cube — Battery Bay
// Fits: 3S 1300mAh LiPo (~105 x 34 x 22mm)
// Velcro strap slots on both sides
// Print: PETG, 40% infill

$fn = 32;

lipo_l  = {LIPO_L};
lipo_w  = {LIPO_W};
lipo_h  = {LIPO_H};
wall    = 2.5;
strap_w = 15;
strap_t = 2;
tol     = 0.5;   // clearance

inner_l = lipo_l + tol;
inner_w = lipo_w + tol;
outer_l = inner_l + 2*wall;
outer_w = inner_w + 2*wall;
outer_h = lipo_h + wall;

difference() {{
    // Outer shell (3 sides + bottom, top open)
    cube([outer_l, outer_w, outer_h], center=true);

    // Inner cavity
    translate([0, 0, wall])
        cube([inner_l, inner_w, outer_h], center=true);

    // Wire exit slot (front)
    translate([outer_l/2, 0, outer_h/4])
        cube([wall*2 + 1, 20, lipo_h/2], center=true);

    // Velcro strap slots (top, both sides)
    for (y = [-lipo_w/4, lipo_w/4]) {{
        translate([0, y, outer_h/2])
            cube([outer_l + 1, strap_w, strap_t*2], center=true);
    }}

    // Mounting holes (bottom, M3)
    for (x = [-lipo_l/3, lipo_l/3]) {{
        translate([x, 0, -outer_h/2])
            cylinder(h=wall + 1, d=3.2);
    }}
}}
"""
    _write('battery_bay.scad', scad)


def write_imu_mount():
    """IMU breakout board mount — mounts near CoM."""
    scad = f"""
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

difference() {{
    union() {{
        // Base plate with rounded corners
        minkowski() {{
            cube([board_l, board_w, wall_t], center=true);
            cylinder(h=0.1, r=corner_r);
        }}

        // 4 standoffs
        for (x = [-board_l/2 + 2, board_l/2 - 2]) {{
            for (y = [-board_w/2 + 2, board_w/2 - 2]) {{
                translate([x, y, wall_t/2])
                    cylinder(h=standoff_h, d=standoff_d);
            }}
        }}
    }}

    // M2 holes in standoffs
    for (x = [-board_l/2 + 2, board_l/2 - 2]) {{
        for (y = [-board_w/2 + 2, board_w/2 - 2]) {{
            translate([x, y, -1])
                cylinder(h=standoff_h + wall_t + 2, d=2.2);
        }}
    }}

    // SPI cable exit
    translate([0, -board_w/2 - 1, 0])
        cube([12, 4, wall_t + 2], center=true);
}}
"""
    _write('imu_mount.scad', scad)


def write_cable_guide():
    """Cable routing clip for internal wiring."""
    scad = f"""
// Self-Balancing Cube — Cable Guide Clip
// Snap-fit clip for 2020 aluminum extrusion
// Holds 3x motor phase wires + SPI bundle
// Print: TPU or flexible PETG for snap-fit

$fn = 32;

extrusion_w = {FRAME_W};
clip_t      = 2.5;
channel_d   = 8;      // cable bundle diameter
arm_l       = 6;
snap_t      = 1.2;

difference() {{
    union() {{
        // Back plate
        cube([extrusion_w + 2*clip_t, clip_t, extrusion_w + 2*clip_t], center=true);

        // Snap arms (top and bottom)
        translate([0, clip_t/2, extrusion_w/2 + clip_t/2])
            cube([extrusion_w - 2, arm_l, snap_t], center=true);
        translate([0, clip_t/2, -extrusion_w/2 - clip_t/2])
            cube([extrusion_w - 2, arm_l, snap_t], center=true);

        // Cable channel holder
        translate([0, -(clip_t + channel_d)/2, 0])
            difference() {{
                cylinder(h=extrusion_w + 2*clip_t, d=channel_d + 2*clip_t, center=true);
                cylinder(h=extrusion_w + 2*clip_t + 1, d=channel_d, center=true);
                translate([0, channel_d, 0])
                    cube([channel_d + 2, channel_d*2, extrusion_w + 3], center=true);
            }}
    }}

    // Extrusion slot
    cube([extrusion_w + 0.4, clip_t + 2, extrusion_w + 0.4], center=true);
}}
"""
    _write('cable_guide.scad', scad)


def write_corner_bracket():
    """Corner bracket for aluminum extrusion joints."""
    scad = f"""
// Self-Balancing Cube — Corner Bracket
// Reinforces 90-degree joints in 2020 aluminum frame
// Print: PETG or ABS, 60% infill for strength

$fn = 32;

arm_l  = 30;
arm_w  = {FRAME_W};
arm_t  = 3;
bolt_d = 4.5;   // M4 clearance

difference() {{
    union() {{
        // Arm 1
        cube([arm_l, arm_w, arm_t]);

        // Arm 2 (perpendicular)
        cube([arm_t, arm_w, arm_l]);
    }}

    // Bolt holes arm 1
    for (x = [arm_l/3, 2*arm_l/3]) {{
        translate([x, arm_w/2, -1])
            cylinder(h=arm_t + 2, d=bolt_d);
    }}

    // Bolt holes arm 2
    for (z = [arm_l/3, 2*arm_l/3]) {{
        translate([-1, arm_w/2, z])
            rotate([0, 90, 0])
                cylinder(h=arm_t + 2, d=bolt_d);
    }}
}}
"""
    _write('corner_bracket.scad', scad)


def _write(filename, content):
    path = os.path.join(OUTPUT_DIR, filename)
    with open(path, 'w') as f:
        f.write(content.strip())
    print(f"[GENERATED] {filename}")


def write_readme():
    readme = """# 3D Print Parts — Self-Balancing Cube

## Files
| File | Part | Qty | Material | Infill |
|------|------|-----|----------|--------|
| motor_mount.scad | Motor mount (2204) | 3 | PETG | 40% |
| reaction_wheel.scad | Reaction wheel disc | 3 | PETG | 80% rim |
| pcb_tray.scad | Nucleo F446RE tray | 1 | PLA | 30% |
| battery_bay.scad | LiPo 1300mAh holder | 1 | PETG | 40% |
| imu_mount.scad | IMU breakout mount | 1 | PLA | 30% |
| cable_guide.scad | Cable routing clip | 6 | TPU | solid |
| corner_bracket.scad | Frame corner bracket | 8 | PETG | 60% |

## How to use
1. Install OpenSCAD (free): https://openscad.org
2. Open any .scad file
3. Press F6 to render
4. File → Export → Export as STL
5. Slice in Cura or PrusaSlicer

## Print settings
- Layer height: 0.2mm
- Perimeters: 3 minimum
- Supports: motor_mount only
- Bed adhesion: brim for motor_mount and corner_bracket

## Assembly order
1. Build aluminum frame (4x 150mm 2020 extrusion per edge)
2. Install corner brackets at all 8 corners
3. Mount PCB tray to bottom face
4. Mount battery bay above PCB tray
5. Mount motor mounts to 3 orthogonal faces (X, Y, Z)
6. Attach reaction wheels to motor shafts (grub screw)
7. Mount IMU at center of frame
8. Route cables with cable guides
"""
    path = os.path.join(OUTPUT_DIR, 'README.md')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(readme)
    print("[GENERATED] README.md")


if __name__ == '__main__':
    print("=" * 50)
    print("  Self-Balancing Cube — Part Generator")
    print(f"  Output: {os.path.abspath(OUTPUT_DIR)}")
    print("=" * 50)

    write_motor_mount()
    write_reaction_wheel()
    write_pcb_tray()
    write_battery_bay()
    write_imu_mount()
    write_cable_guide()
    write_corner_bracket()
    write_readme()

    print("=" * 50)
    print("  Done. Open .scad files in OpenSCAD to render STL.")
    print("  Install: https://openscad.org/downloads.html")
    print("=" * 50)
