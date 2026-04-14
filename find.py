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
from sys import Decimal

DeviceManagerCLI.BuildDeviceList()

device = KCubeLiquidCrystal.CreateKCubeLiquidCrystal('39443416')
device.Connect('39443416')
device.WaitForSettingsInitialized(3000)
device.StartPolling(250)

import time
time.sleep(0.5)
device.EnableDevice()
time.sleep(0.5)

params = device.GetPresetParams(1)   # get preset 1
print(type(params))
print([m for m in dir(params) if not m.startswith('_')])

device.StopPolling()
device.Disconnect()