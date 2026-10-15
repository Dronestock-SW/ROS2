import os

Import("env")

framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
mcu = env.BoardConfig().get("build.mcu", "esp32s3")
flash_mode = env.BoardConfig().get("build.flash_mode", "qio")
memory_type = "%s_qspi" % flash_mode
sdkconfig = os.path.join(
    framework_dir, "tools", "sdk", mcu, memory_type, "include", "sdkconfig.h"
)

if not os.path.isfile(sdkconfig):
    raise RuntimeError("ESP32 system sdkconfig.h not found: %s" % sdkconfig)

env.Append(CCFLAGS=["-include", sdkconfig])

# Same Heltec archive as the production project, resolved inside this package.
heltec_lib_dir = os.path.join(
    env.subst("$PROJECT_DIR"), "lib", "Heltec ESP32 Dev-Boards", "src", "esp32s3"
)
heltec_archive = os.path.join(heltec_lib_dir, "liblorawan.a")
if not os.path.isfile(heltec_archive):
    raise RuntimeError("Bundled Heltec ESP32-S3 LoRa archive not found: %s" % heltec_archive)

env.Append(LIBPATH=[heltec_lib_dir], LIBS=["lorawan"])
