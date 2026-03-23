"""
Self-Balancing Cube — Project Structure Setup
=============================================
Creates all remaining folders in the project.
Run once from the project root.
"""

import os

BASE = os.path.dirname(os.path.abspath(__file__))

folders = [
    "docs/diagrams",
    "docs/mechanical/scad",
    "docs/mechanical/stl",
    "docs/mechanical/assembly",
    "docs/schematics",
]

for folder in folders:
    path = os.path.join(BASE, folder)
    os.makedirs(path, exist_ok=True)
    print(f"[OK] {folder}")

print("\nFolder structure ready.")
print("Next: run generate_parts.py to create SCAD files.")
print("Then open each .scad in OpenSCAD and export STL.")
