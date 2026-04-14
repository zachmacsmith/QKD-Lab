import sys, pathlib

KINESIS = r"C:\Program Files\Thorlabs\Kinesis"
sys.path.insert(0, KINESIS)

# List all KLC-related DLLs
for dll in pathlib.Path(KINESIS).glob("*Liquid*"):
    print(dll.name)