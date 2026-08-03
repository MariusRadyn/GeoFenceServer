Import("env")

from pathlib import Path

# Semantic prefix + auto-incremented build counter → e.g. V1.1.42
VERSION_PREFIX = "V1.1"
BUILD_NUMBER_FILE = Path(env["PROJECT_DIR"]) / "BUILD_NUMBER"

build_number = 1
if BUILD_NUMBER_FILE.exists():
    try:
        build_number = int(BUILD_NUMBER_FILE.read_text(encoding="utf-8").strip()) + 1
    except ValueError:
        build_number = 1

BUILD_NUMBER_FILE.write_text(str(build_number) + "\n", encoding="utf-8")

fw_version = f"{VERSION_PREFIX}.{build_number}"
print(f"Firmware version: {fw_version}")

env.Append(
    CPPDEFINES=[
        ("FW_VERSION", env.StringifyMacro(fw_version)),
        ("FW_BUILD", build_number),
    ]
)
