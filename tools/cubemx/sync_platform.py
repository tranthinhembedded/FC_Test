#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]

SYNC_MAP = {
    "Core/Inc/dma.h": "Core/Inc/platform/dma.h",
    "Core/Inc/gpio.h": "Core/Inc/platform/gpio.h",
    "Core/Inc/i2c.h": "Core/Inc/platform/i2c.h",
    "Core/Inc/spi.h": "Core/Inc/platform/spi.h",
    "Core/Inc/tim.h": "Core/Inc/platform/tim.h",
    "Core/Inc/usart.h": "Core/Inc/platform/usart.h",
    "Core/Inc/stm32f4xx_it.h": "Core/Inc/platform/stm32f4xx_it.h",
    "Core/Src/dma.c": "Core/Src/platform/dma.c",
    "Core/Src/gpio.c": "Core/Src/platform/gpio.c",
    "Core/Src/i2c.c": "Core/Src/platform/i2c.c",
    "Core/Src/spi.c": "Core/Src/platform/spi.c",
    "Core/Src/tim.c": "Core/Src/platform/tim.c",
    "Core/Src/usart.c": "Core/Src/platform/usart.c",
    "Core/Src/stm32f4xx_it.c": "Core/Src/platform/stm32f4xx_it.c",
    "Core/Src/stm32f4xx_hal_msp.c": "Core/Src/platform/stm32f4xx_hal_msp.c",
    "Core/Src/system_stm32f4xx.c": "Core/Src/platform/system_stm32f4xx.c",
}

INCLUDE_REWRITES = {
    '#include "dma.h"': '#include "platform/dma.h"',
    '#include "gpio.h"': '#include "platform/gpio.h"',
    '#include "i2c.h"': '#include "platform/i2c.h"',
    '#include "spi.h"': '#include "platform/spi.h"',
    '#include "tim.h"': '#include "platform/tim.h"',
    '#include "usart.h"': '#include "platform/usart.h"',
    '#include "stm32f4xx_it.h"': '#include "platform/stm32f4xx_it.h"',
}

MAIN_INCLUDE_REWRITES = {
    '#include "dma.h"': '#include "platform/dma.h"',
    '#include "gpio.h"': '#include "platform/gpio.h"',
    '#include "i2c.h"': '#include "platform/i2c.h"',
    '#include "spi.h"': '#include "platform/spi.h"',
    '#include "tim.h"': '#include "platform/tim.h"',
    '#include "usart.h"': '#include "platform/usart.h"',
}


def read_text_with_newline(path: Path) -> tuple[str, str]:
    raw = path.read_bytes()
    newline = "\r\n" if b"\r\n" in raw else "\n"
    return raw.decode("utf-8"), newline


def write_text_with_newline(path: Path, content: str, newline: str) -> None:
    normalized = content.replace("\r\n", "\n")
    if newline == "\r\n":
        normalized = normalized.replace("\n", "\r\n")
    path.write_text(normalized, encoding="utf-8")


def rewrite_includes(text: str) -> str:
    for old, new in INCLUDE_REWRITES.items():
        text = text.replace(old, new)
    return text


def sync_generated_file(source_rel: str, target_rel: str) -> bool:
    source_path = PROJECT_ROOT / source_rel
    target_path = PROJECT_ROOT / target_rel

    if not source_path.exists():
        return False

    content, newline = read_text_with_newline(source_path)
    content = rewrite_includes(content)

    target_path.parent.mkdir(parents=True, exist_ok=True)
    write_text_with_newline(target_path, content, newline)
    source_path.unlink()

    print(f"synced {source_rel} -> {target_rel}")
    return True


def rewrite_main_includes() -> bool:
    main_path = PROJECT_ROOT / "Core/Src/main.c"
    if not main_path.exists():
        return False

    content, newline = read_text_with_newline(main_path)
    updated = content
    for old, new in MAIN_INCLUDE_REWRITES.items():
        updated = updated.replace(old, new)

    if updated == content:
        return False

    write_text_with_newline(main_path, updated, newline)
    print("rewrote Core/Src/main.c platform includes")
    return True


def main() -> int:
    changed = False

    for source_rel, target_rel in SYNC_MAP.items():
        changed |= sync_generated_file(source_rel, target_rel)

    changed |= rewrite_main_includes()

    if not changed:
        print("no generated files needed syncing")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
