import sys, pathlib

KINESIS = r"C:\Program Files\Thorlabs\Kinesis"

# First confirm the folder exists and list everything
p = pathlib.Path(KINESIS)
print("Folder exists:", p.exists())
print("\nAll DLLs:")
for dll in sorted(p.glob("*.dll")):
    print(dll.name)

    import pathlib

# Search common install locations
search_roots = [
    r"C:\Program Files",
    r"C:\Program Files (x86)",
    r"C:\ProgramData",
]

for root in search_roots:
    for p in pathlib.Path(root).rglob("Thorlabs.MotionControl.DeviceManagerCLI.dll"):
        print(p.parent)

import struct
print(struct.calcsize("P") * 8, "bit Python")

import sys
import clr

KINESIS = r"C:\Program Files\Thorlabs\EDU-QOP1"
sys.path.insert(0, KINESIS)

clr.AddReference(r"C:\Program Files\Thorlabs\EDU-QOP1\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\Program Files\Thorlabs\EDU-QOP1\Thorlabs.MotionControl.KCube.LiquidCrystalCLI.dll")

from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.KCube.LiquidCrystalCLI import *

# List all public methods on the KLC101 class
methods = [m for m in dir(KCubeLiquidCrystal) if not m.startswith('_')]
for m in sorted(methods):
    print(m)