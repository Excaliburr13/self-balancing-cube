# 3D Print Parts — Self-Balancing Cube

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
