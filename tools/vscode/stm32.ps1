param(
    [ValidateSet('build', 'clean', 'flash', 'compile-commands')]
    [string]$Action = 'build',
    [string]$SettingsPath = '',
    [string]$GccPath = '',
    [string]$ProgrammerCli = '',
    [string]$ArtifactName = '',
    [string]$FlashAddress = ''
)

$ErrorActionPreference = 'Stop'

function Get-ProjectRoot {
    return (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
}

function Assert-Tool {
    param(
        [string]$PathValue,
        [string]$Label
    )

    if ([string]::IsNullOrWhiteSpace($PathValue)) {
        throw "$Label is not configured."
    }
    if (-not (Test-Path -LiteralPath $PathValue)) {
        throw "$Label not found: $PathValue"
    }
}

function Get-WorkspaceSettings {
    param(
        [string]$ProjectRoot,
        [string]$OverridePath
    )

    $settingsFile = $OverridePath
    if ([string]::IsNullOrWhiteSpace($settingsFile)) {
        $settingsFile = Join-Path $ProjectRoot '.vscode/settings.json'
    } elseif (-not [System.IO.Path]::IsPathRooted($settingsFile)) {
        $settingsFile = Join-Path $ProjectRoot $settingsFile
    }

    if (-not (Test-Path -LiteralPath $settingsFile)) {
        throw "VS Code settings not found: $settingsFile"
    }

    return Get-Content -LiteralPath $settingsFile -Raw | ConvertFrom-Json
}

function Get-SettingValue {
    param(
        [object]$Settings,
        [string]$Name,
        $Default = $null
    )

    $property = $Settings.PSObject.Properties[$Name]
    if ($null -ne $property) {
        return $property.Value
    }
    return $Default
}

function Get-SettingArray {
    param(
        [object]$Settings,
        [string]$Name
    )

    $value = Get-SettingValue -Settings $Settings -Name $Name -Default @()
    if ($null -eq $value) {
        return @()
    }
    if ($value -is [string]) {
        if ([string]::IsNullOrWhiteSpace($value)) {
            return @()
        }
        return @($value)
    }
    return @($value)
}

function Resolve-ProjectPath {
    param(
        [string]$ProjectRoot,
        [string]$PathValue
    )

    if ([string]::IsNullOrWhiteSpace($PathValue)) {
        return ''
    }
    if ([System.IO.Path]::IsPathRooted($PathValue)) {
        return [System.IO.Path]::GetFullPath($PathValue)
    }
    return [System.IO.Path]::GetFullPath((Join-Path $ProjectRoot $PathValue))
}

function New-DirectoryForFile {
    param([string]$FilePath)

    $dir = Split-Path -Parent $FilePath
    if (-not (Test-Path -LiteralPath $dir)) {
        New-Item -ItemType Directory -Force -Path $dir | Out-Null
    }
}

function Get-RelativePath {
    param(
        [string]$BasePath,
        [string]$TargetPath
    )

    $baseUri = New-Object System.Uri(([System.IO.Path]::GetFullPath($BasePath).TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar))
    $targetUri = New-Object System.Uri([System.IO.Path]::GetFullPath($TargetPath))
    return [System.Uri]::UnescapeDataString($baseUri.MakeRelativeUri($targetUri).ToString()).Replace('\', '/')
}

function Get-MachineArgs {
    param([object]$Settings)

    $cpu = Get-SettingValue -Settings $Settings -Name 'stm32.project.cpu'
    $fpu = Get-SettingValue -Settings $Settings -Name 'stm32.project.fpu'
    $floatAbi = Get-SettingValue -Settings $Settings -Name 'stm32.project.floatAbi'

    if ([string]::IsNullOrWhiteSpace($cpu)) {
        throw 'stm32.project.cpu is not configured.'
    }

    $args = @("-mcpu=$cpu", '-mthumb')
    if (-not [string]::IsNullOrWhiteSpace($fpu)) {
        $args += "-mfpu=$fpu"
    }
    if (-not [string]::IsNullOrWhiteSpace($floatAbi)) {
        $args += "-mfloat-abi=$floatAbi"
    }
    return $args
}

function Get-SourceFiles {
    param(
        [string]$ProjectRoot,
        [object]$Settings
    )

    $sources = New-Object System.Collections.Generic.List[string]
    foreach ($sourceDir in Get-SettingArray -Settings $Settings -Name 'stm32.project.sourceDirs') {
        $resolved = Resolve-ProjectPath -ProjectRoot $ProjectRoot -PathValue $sourceDir
        if ([string]::IsNullOrWhiteSpace($resolved) -or -not (Test-Path -LiteralPath $resolved)) {
            continue
        }

        Get-ChildItem -Path $resolved -Filter '*.c' -File -Recurse | ForEach-Object {
            $sources.Add($_.FullName)
        }
    }

    return $sources | Sort-Object -Unique
}

function Get-CommonArgs {
    param(
        [string]$ProjectRoot,
        [object]$Settings
    )

    $cStandard = Get-SettingValue -Settings $Settings -Name 'stm32.project.cStandard' -Default 'gnu11'
    $debugLevel = Get-SettingValue -Settings $Settings -Name 'stm32.project.debugLevel' -Default 'g3'
    $optimization = Get-SettingValue -Settings $Settings -Name 'stm32.project.optimization' -Default 'O0'
    $args = @(Get-MachineArgs -Settings $Settings)
    $args += @(
        "-std=$cStandard",
        "-$debugLevel",
        "-$optimization",
        '-ffunction-sections',
        '-fdata-sections',
        '-Wall'
    )

    foreach ($define in Get-SettingArray -Settings $Settings -Name 'stm32.project.defines') {
        $args += "-D$define"
    }

    foreach ($includeDir in Get-SettingArray -Settings $Settings -Name 'stm32.project.includeDirs') {
        $resolved = Resolve-ProjectPath -ProjectRoot $ProjectRoot -PathValue $includeDir
        if (-not [string]::IsNullOrWhiteSpace($resolved) -and (Test-Path -LiteralPath $resolved)) {
            $args += "-I$resolved"
        }
    }

    $args += Get-SettingArray -Settings $Settings -Name 'stm32.project.compilerExtraArgs'
    return $args
}

function Get-LinkerArgs {
    param(
        [string]$ProjectRoot,
        [object]$Settings,
        [string]$MapPath
    )

    $linkerScript = Resolve-ProjectPath -ProjectRoot $ProjectRoot -PathValue (Get-SettingValue -Settings $Settings -Name 'stm32.project.linkerScript')
    if ([string]::IsNullOrWhiteSpace($linkerScript) -or -not (Test-Path -LiteralPath $linkerScript)) {
        throw "Linker script not found: $linkerScript"
    }

    $args = @(Get-MachineArgs -Settings $Settings)
    $args += @('-T', $linkerScript)
    $args += Get-SettingArray -Settings $Settings -Name 'stm32.project.linkerSpecs'
    $args += @(
        "-Wl,-Map=$MapPath",
        '-Wl,--gc-sections'
    )

    foreach ($libraryDir in Get-SettingArray -Settings $Settings -Name 'stm32.project.librarySearchDirs') {
        $resolved = Resolve-ProjectPath -ProjectRoot $ProjectRoot -PathValue $libraryDir
        if (-not [string]::IsNullOrWhiteSpace($resolved) -and (Test-Path -LiteralPath $resolved)) {
            $args += "-L$resolved"
        }
    }

    foreach ($symbol in Get-SettingArray -Settings $Settings -Name 'stm32.project.linkerSymbols') {
        $args += @('-u', $symbol)
    }

    $args += Get-SettingArray -Settings $Settings -Name 'stm32.project.linkerExtraArgs'

    $libraries = Get-SettingArray -Settings $Settings -Name 'stm32.project.libraries'
    if ($libraries.Count -gt 0) {
        $args += '-Wl,--start-group'
        $args += $libraries
        $args += '-Wl,--end-group'
    }

    return $args
}

function Get-ObjectPath {
    param(
        [string]$ProjectRoot,
        [string]$ObjectRoot,
        [string]$SourcePath
    )

    $relative = Get-RelativePath -BasePath $ProjectRoot -TargetPath $SourcePath
    $objectPath = Join-Path $ObjectRoot (($relative -replace '/', '\') + '.o')
    return ($objectPath -replace '\.(c|s|S)\.o$', '.o')
}

function Write-CompileCommands {
    param(
        [string]$ProjectRoot,
        [string]$Compiler,
        [object]$Settings
    )

    $commonArgs = Get-CommonArgs -ProjectRoot $ProjectRoot -Settings $Settings
    $entries = foreach ($fullName in Get-SourceFiles -ProjectRoot $ProjectRoot -Settings $Settings) {
        $relative = Get-RelativePath -BasePath $ProjectRoot -TargetPath $fullName

        [pscustomobject]@{
            directory = $ProjectRoot.Replace('\', '/')
            file = $fullName.Replace('\', '/')
            arguments = @(
                $Compiler.Replace('\', '/')
            ) + $commonArgs + @(
                '-c',
                $relative
            )
        }
    }

    $entries | ConvertTo-Json -Depth 4 | Set-Content -Encoding ascii (Join-Path $ProjectRoot 'compile_commands.json')
}

function Invoke-Build {
    param(
        [string]$ProjectRoot,
        [string]$Compiler,
        [string]$Artifact,
        [string]$BuildRoot,
        [object]$Settings
    )

    $toolBin = Split-Path -Parent $Compiler
    $objcopy = Join-Path $toolBin 'arm-none-eabi-objcopy.exe'
    $objdump = Join-Path $toolBin 'arm-none-eabi-objdump.exe'
    $size = Join-Path $toolBin 'arm-none-eabi-size.exe'
    $commonArgs = Get-CommonArgs -ProjectRoot $ProjectRoot -Settings $Settings
    $objects = New-Object System.Collections.Generic.List[string]
    $objRoot = Join-Path $BuildRoot 'obj'
    $artifactRoot = Join-Path $BuildRoot $Artifact
    $elf = $artifactRoot + '.elf'
    $map = $artifactRoot + '.map'
    $list = $artifactRoot + '.list'
    $bin = $artifactRoot + '.bin'
    $hex = $artifactRoot + '.hex'
    $sizeTxt = Join-Path $BuildRoot 'size.txt'
    $startupSrc = Resolve-ProjectPath -ProjectRoot $ProjectRoot -PathValue (Get-SettingValue -Settings $Settings -Name 'stm32.project.startupFile')
    $startupObj = Get-ObjectPath -ProjectRoot $ProjectRoot -ObjectRoot $objRoot -SourcePath $startupSrc

    Assert-Tool -PathValue $objcopy -Label 'objcopy'
    Assert-Tool -PathValue $objdump -Label 'objdump'
    Assert-Tool -PathValue $size -Label 'size'

    if ([string]::IsNullOrWhiteSpace($startupSrc) -or -not (Test-Path -LiteralPath $startupSrc)) {
        throw "Startup file not found: $startupSrc"
    }

    if (Test-Path -LiteralPath $BuildRoot) {
        Remove-Item -Recurse -Force $BuildRoot
    }
    New-Item -ItemType Directory -Force -Path $BuildRoot | Out-Null

    foreach ($fullName in Get-SourceFiles -ProjectRoot $ProjectRoot -Settings $Settings) {
        $objectPath = Get-ObjectPath -ProjectRoot $ProjectRoot -ObjectRoot $objRoot -SourcePath $fullName
        New-DirectoryForFile -FilePath $objectPath

        & $Compiler @commonArgs '-c' $fullName '-o' $objectPath
        if ($LASTEXITCODE -ne 0) {
            throw "Compile failed: $fullName"
        }
        $objects.Add($objectPath)
    }

    New-DirectoryForFile -FilePath $startupObj
    & $Compiler @(Get-MachineArgs -Settings $Settings) '-c' $startupSrc '-o' $startupObj
    if ($LASTEXITCODE -ne 0) {
        throw "Assemble failed: $startupSrc"
    }
    $objects.Add($startupObj)

    $linkArgs = @('-o', $elf) + $objects.ToArray() + (Get-LinkerArgs -ProjectRoot $ProjectRoot -Settings $Settings -MapPath $map)
    & $Compiler @linkArgs
    if ($LASTEXITCODE -ne 0) {
        throw 'Link failed.'
    }

    & $objdump '-h' '-S' $elf | Set-Content -Encoding ascii $list
    & $objcopy '-O' 'binary' $elf $bin
    & $objcopy '-O' 'ihex' $elf $hex
    & $size $elf | Set-Content -Encoding ascii $sizeTxt

    Write-CompileCommands -ProjectRoot $ProjectRoot -Compiler $Compiler -Settings $Settings

    Write-Output "Built: $elf"
    Write-Output "Binary: $bin"
}

function Invoke-Flash {
    param(
        [string]$Programmer,
        [string]$BuildRoot,
        [string]$Artifact,
        [string]$Address,
        [object]$Settings
    )

    $bin = Join-Path $BuildRoot ($Artifact + '.bin')
    Assert-Tool -PathValue $Programmer -Label 'STM32_Programmer_CLI'

    if (-not (Test-Path -LiteralPath $bin)) {
        throw "Binary not found. Build first: $bin"
    }

    $flashArgs = @('-c') + (Get-SettingArray -Settings $Settings -Name 'stm32.flash.connectArgs') + @('-w', $bin, $Address)
    $flashArgs += Get-SettingArray -Settings $Settings -Name 'stm32.flash.extraArgs'

    & $Programmer @flashArgs
    if ($LASTEXITCODE -ne 0) {
        throw 'Flash failed.'
    }
}

$projectRoot = Get-ProjectRoot
Set-Location $projectRoot

$settings = Get-WorkspaceSettings -ProjectRoot $projectRoot -OverridePath $SettingsPath

$gccPathValue = $GccPath
if ([string]::IsNullOrWhiteSpace($gccPathValue)) {
    $gccPathValue = Get-SettingValue -Settings $settings -Name 'stm32.tools.gcc'
}

$programmerCliValue = $ProgrammerCli
if ([string]::IsNullOrWhiteSpace($programmerCliValue)) {
    $programmerCliValue = Get-SettingValue -Settings $settings -Name 'stm32.tools.cubeProgrammerCli'
}

$artifactNameValue = $ArtifactName
if ([string]::IsNullOrWhiteSpace($artifactNameValue)) {
    $artifactNameValue = Get-SettingValue -Settings $settings -Name 'stm32.project.artifactName' -Default 'firmware'
}

$flashAddressValue = $FlashAddress
if ([string]::IsNullOrWhiteSpace($flashAddressValue)) {
    $flashAddressValue = Get-SettingValue -Settings $settings -Name 'stm32.project.flashAddress' -Default '0x08000000'
}

$buildRoot = Resolve-ProjectPath -ProjectRoot $projectRoot -PathValue (Get-SettingValue -Settings $settings -Name 'stm32.project.buildDir' -Default 'build/vscode')

Assert-Tool -PathValue $gccPathValue -Label 'arm-none-eabi-gcc'

switch ($Action) {
    'clean' {
        if (Test-Path -LiteralPath $buildRoot) {
            Remove-Item -Recurse -Force $buildRoot
        }
        Write-Output 'Clean complete.'
    }
    'compile-commands' {
        Write-CompileCommands -ProjectRoot $projectRoot -Compiler $gccPathValue -Settings $settings
        Write-Output 'compile_commands.json updated.'
    }
    'flash' {
        Invoke-Flash -Programmer $programmerCliValue -BuildRoot $buildRoot -Artifact $artifactNameValue -Address $flashAddressValue -Settings $settings
    }
    'build' {
        Invoke-Build -ProjectRoot $projectRoot -Compiler $gccPathValue -Artifact $artifactNameValue -BuildRoot $buildRoot -Settings $settings
    }
}
