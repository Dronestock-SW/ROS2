"""Record the actual flash addresses/settings without running an uploader."""
import json
from pathlib import Path

Import("env")


def export_flash_layout(source, target, env):
    images = []
    for offset, file in env.get("FLASH_EXTRA_IMAGES", []):
        images.append({"offset": str(offset), "path": env.subst(str(file))})
    images.append({"offset": env.subst("$ESP32_APP_OFFSET"), "path": str(target[0])})
    board = env.BoardConfig()
    payload = {
        "environment": env["PIOENV"],
        "board": env.subst("$BOARD"),
        "chip": board.get("build.mcu"),
        "flash_mode": env.subst("${__get_board_flash_mode(__env__)}"),
        "flash_frequency": env.subst("${__get_board_f_image(__env__)}"),
        "flash_size": board.get("upload.flash_size"),
        "application_offset": env.subst("$ESP32_APP_OFFSET"),
        "images": images,
        "upload_performed": False,
    }
    destination = Path(env.subst("$BUILD_DIR")) / "flash_layout_export.json"
    destination.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", export_flash_layout)
