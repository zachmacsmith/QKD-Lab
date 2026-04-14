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