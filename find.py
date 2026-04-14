import sys, pathlib

KINESIS = r"C:\Program Files\Thorlabs\Kinesis"

# First confirm the folder exists and list everything
p = pathlib.Path(KINESIS)
print("Folder exists:", p.exists())
print("\nAll DLLs:")
for dll in sorted(p.glob("*.dll")):
    print(dll.name)