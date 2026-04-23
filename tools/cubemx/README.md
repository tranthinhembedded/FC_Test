# CubeMX Platform Sync

Use this after STM32CubeMX/STM32CubeIDE code generation when peripheral files must live under `Core/Inc/platform` and `Core/Src/platform`.

- `sync_platform.py`: Linux-friendly post-generation script.
- `sync_platform.bat`: Windows wrapper that calls the Python script.

What it does:

- Moves generated peripheral headers from `Core/Inc` into `Core/Inc/platform`.
- Moves generated peripheral sources from `Core/Src` into `Core/Src/platform`.
- Rewrites generated includes so platform sources include `platform/*.h`.
- Rewrites `Core/Src/main.c` to include platform headers.

Recommended hook:

- Linux: `ProjectManager.UAScriptAfterPath=tools/cubemx/sync_platform.py`
- Windows: set the CubeMX user action to `tools\\cubemx\\sync_platform.bat`
