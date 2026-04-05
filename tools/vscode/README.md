# STM32 VS Code Template

Bo cau hinh nay giup VS Code hoat dong gan giong STM32CubeIDE cho cac du an STM32 dung CubeMX/CubeIDE.

## Cac file chinh

- `tools/vscode/init-stm32-vscode.ps1`: Tool khoi tao tu dong `.vscode/` cho project STM32.
- `.vscode/settings.json`: Noi duy nhat can sua khi mang sang project khac.
- `.vscode/tasks.json`: Build, clean, flash, cap nhat `compile_commands.json`.
- `.vscode/launch.json`: Debug voi Cortex-Debug + ST-LINK.
- `tools/vscode/stm32.ps1`: Script build/flash dung ARM GCC cua CubeIDE.

## Cach dung trong project hien tai

1. Cai cac extension duoc goi y trong `.vscode/extensions.json`.
2. Mo VS Code tai thu muc project.
3. Chay `Terminal > Run Task > STM32: Build`.
4. Neu can nap firmware, chay `STM32: Flash`.
5. Neu can debug, nhan `F5`.

## Cach mang sang du an STM32 khac

1. Copy thu muc `tools/vscode/` sang project moi.
2. Chay:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\vscode\init-stm32-vscode.ps1 -ProjectRoot . -Configuration Debug -Force
```

3. Neu muon tool build kiem tra ngay sau khi sinh cau hinh, them `-BuildAfterSetup`.
4. Mo VS Code va dung cac task `STM32:*` nhu binh thuong.

## Tool bootstrap tu dong lam gi

- Doc `.ioc`, `.project`, `.cproject`.
- Tu xac dinh MCU, CPU, FPU, linker script, startup file.
- Lay include path, define, library search path va library tu cau hinh CubeIDE.
- Tu tim duong dan ARM GCC, GDB, ST-LINK GDB Server, STM32CubeProgrammer trong STM32CubeIDE.
- Tu chon file SVD phu hop voi MCU.
- Sinh `.vscode/settings.json`, `tasks.json`, `launch.json`, `c_cpp_properties.json`, `extensions.json`.
- Tao `compile_commands.json` de IntelliSense hoat dong ngay.

## Cac khoa thuong phai sua trong `.vscode/settings.json`

- `stm32.tools.*`: Duong dan ARM GCC, GDB, ST-LINK GDB Server, STM32CubeProgrammer.
- `stm32.project.artifactName`: Ten file output.
- `stm32.project.cpu`: Vi du `cortex-m0`, `cortex-m3`, `cortex-m4`, `cortex-m7`.
- `stm32.project.fpu`, `stm32.project.floatAbi`: Sua neu MCU co FPU khac hoac khong co FPU.
- `stm32.project.device`: Ten device cho Cortex-Debug, vi du `STM32F401CEUx`.
- `stm32.project.svdFile`: File SVD phu hop MCU.
- `stm32.project.linkerScript`: File linker `.ld`.
- `stm32.project.startupFile`: File startup assembly.
- `stm32.project.flashAddress`: Dia chi flash, thuong la `0x08000000`.
- `stm32.project.sourceDirs`: Danh sach thu muc chua file `.c`.
- `stm32.project.includeDirs`: Danh sach include path.
- `stm32.project.defines`: Cac macro nhu `USE_HAL_DRIVER`, `STM32F401xE`.
- `stm32.project.librarySearchDirs`, `stm32.project.libraries`: Thu muc thu vien va danh sach library khi link.

## Ghi chu

- Script hien tai duoc thiet ke tot nhat cho du an C dung cau truc CubeMX/CubeIDE.
- Neu project co them middleware hoac module rieng, chi can them vao `sourceDirs`, `includeDirs` hoac `libraries`.
- Sau khi sua cau hinh, chay `STM32: Build` hoac `STM32: Update compile_commands` de VS Code cap nhat IntelliSense.
