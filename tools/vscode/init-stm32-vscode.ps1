param(
    [string]$ProjectRoot = '.',
    [string]$Configuration = 'Debug',
    [string]$CubeIdeRoot = '',
    [switch]$Force,
    [switch]$BuildAfterSetup
)

$ErrorActionPreference = 'Stop'

function Get-AbsolutePath {
    param(
        [string]$BasePath,
        [string]$PathValue
    )

    if ([string]::IsNullOrWhiteSpace($PathValue)) {
        return ''
    }
    if ([System.IO.Path]::IsPathRooted($PathValue)) {
        return [System.IO.Path]::GetFullPath($PathValue)
    }
    return [System.IO.Path]::GetFullPath((Join-Path $BasePath $PathValue))
}

function Get-ProjectRelativePath {
    param(
        [string]$ProjectRoot,
        [string]$PathValue
    )

    $absolute = Get-AbsolutePath -BasePath $ProjectRoot -PathValue $PathValue
    $projectPrefix = [System.IO.Path]::GetFullPath($ProjectRoot).TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    if ($absolute.StartsWith($projectPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $absolute.Substring($projectPrefix.Length).Replace('\', '/')
    }
    return $absolute.Replace('\', '/')
}

function Get-ProjectFile {
    param(
        [string]$Root,
        [string]$Pattern
    )

    return Get-ChildItem -Path $Root -Filter $Pattern -File | Select-Object -First 1
}

function Read-IocProperties {
    param([string]$IocPath)

    $properties = @{}
    if (-not (Test-Path -LiteralPath $IocPath)) {
        return $properties
    }

    foreach ($line in Get-Content -LiteralPath $IocPath) {
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }
        if ($line.StartsWith('#')) {
            continue
        }
        $separator = $line.IndexOf('=')
        if ($separator -lt 1) {
            continue
        }

        $key = $line.Substring(0, $separator).Trim()
        $value = $line.Substring($separator + 1).Trim()
        $properties[$key] = $value
    }

    return $properties
}

function Get-ProjectName {
    param(
        [string]$ProjectRoot,
        [hashtable]$IocProperties
    )

    $projectFile = Join-Path $ProjectRoot '.project'
    if (Test-Path -LiteralPath $projectFile) {
        [xml]$projectXml = Get-Content -LiteralPath $projectFile -Raw
        $nameNode = $projectXml.SelectSingleNode('/projectDescription/name')
        if ($null -ne $nameNode -and -not [string]::IsNullOrWhiteSpace($nameNode.InnerText)) {
            return $nameNode.InnerText.Trim()
        }
    }

    if ($IocProperties.ContainsKey('ProjectManager.ProjectName')) {
        return $IocProperties['ProjectManager.ProjectName']
    }

    return Split-Path -Leaf $ProjectRoot
}

function Get-XmlAttributeValue {
    param(
        [System.Xml.XmlNode]$Node,
        [string]$AttributeName
    )

    if ($null -eq $Node) {
        return ''
    }
    $attribute = $Node.Attributes[$AttributeName]
    if ($null -eq $attribute) {
        return ''
    }
    return $attribute.Value
}

function Get-XmlNodeAttributeByXPath {
    param(
        [System.Xml.XmlNode]$RootNode,
        [string]$XPath,
        [string]$AttributeName
    )

    $node = $RootNode.SelectSingleNode($XPath)
    return Get-XmlAttributeValue -Node $node -AttributeName $AttributeName
}

function Get-XmlListAttributeByXPath {
    param(
        [System.Xml.XmlNode]$RootNode,
        [string]$XPath,
        [string]$AttributeName
    )

    $values = New-Object System.Collections.Generic.List[string]
    foreach ($node in $RootNode.SelectNodes($XPath)) {
        $value = Get-XmlAttributeValue -Node $node -AttributeName $AttributeName
        if (-not [string]::IsNullOrWhiteSpace($value)) {
            $values.Add($value)
        }
    }
    return $values.ToArray()
}

function Resolve-CubeProjectValue {
    param(
        [string]$Value,
        [string]$ProjectName
    )

    if ([string]::IsNullOrWhiteSpace($Value)) {
        return ''
    }

    $resolved = $Value.Trim().Replace('\', '/')
    $projMacroPrefix = '${workspace_loc:/${ProjName}/'
    $projectPrefix = '${workspace_loc:/' + $ProjectName + '/'

    if ($resolved.StartsWith($projMacroPrefix)) {
        $resolved = $resolved.Substring($projMacroPrefix.Length)
    } elseif ($resolved.StartsWith($projectPrefix)) {
        $resolved = $resolved.Substring($projectPrefix.Length)
    }

    if ($resolved.EndsWith('}')) {
        $resolved = $resolved.Substring(0, $resolved.Length - 1)
    }

    while ($resolved.StartsWith('../')) {
        $resolved = $resolved.Substring(3)
    }

    return $resolved.Trim()
}

function Get-UniqueStrings {
    param([string[]]$Values)

    return $Values |
        Where-Object { -not [string]::IsNullOrWhiteSpace($_) } |
        ForEach-Object { $_.Trim().Replace('\', '/') } |
        Select-Object -Unique
}

function Normalize-LinkerLibraryValue {
    param([string]$Value)

    if ([string]::IsNullOrWhiteSpace($Value)) {
        return ''
    }

    $trimmed = $Value.Trim()
    if ($trimmed.StartsWith('-l')) {
        return $trimmed
    }
    if ($trimmed.StartsWith(':')) {
        return '-l' + $trimmed
    }
    return '-l' + $trimmed
}

function Convert-EnumeratedOptionValue {
    param([string]$RawValue)

    if ([string]::IsNullOrWhiteSpace($RawValue)) {
        return ''
    }

    $parts = $RawValue.Split('.')
    return $parts[$parts.Length - 1]
}

function Get-CortexCpuFromFamily {
    param(
        [string]$Family,
        [string]$DeviceName
    )

    $probe = $Family
    if ([string]::IsNullOrWhiteSpace($probe)) {
        $probe = $DeviceName
    }
    $probe = $probe.ToUpperInvariant()

    switch -regex ($probe) {
        '^STM32C0' { return 'cortex-m0plus' }
        '^STM32F0' { return 'cortex-m0' }
        '^STM32G0' { return 'cortex-m0plus' }
        '^STM32L0' { return 'cortex-m0plus' }
        '^STM32U0' { return 'cortex-m0plus' }
        '^STM32F1' { return 'cortex-m3' }
        '^STM32F2' { return 'cortex-m3' }
        '^STM32L1' { return 'cortex-m3' }
        '^STM32F3' { return 'cortex-m4' }
        '^STM32F4' { return 'cortex-m4' }
        '^STM32G4' { return 'cortex-m4' }
        '^STM32L4' { return 'cortex-m4' }
        '^STM32WB' { return 'cortex-m4' }
        '^STM32WL' { return 'cortex-m4' }
        '^STM32F7' { return 'cortex-m7' }
        '^STM32H7' { return 'cortex-m7' }
        '^STM32H5' { return 'cortex-m33' }
        '^STM32L5' { return 'cortex-m33' }
        '^STM32U5' { return 'cortex-m33' }
        default { throw "Khong tu dong xac dinh duoc CPU cho MCU/family: $probe" }
    }
}

function Get-OptimizationFlag {
    param([string]$RawValue)

    switch ((Convert-EnumeratedOptionValue -RawValue $RawValue).ToLowerInvariant()) {
        'o0' { return 'O0' }
        'o1' { return 'O1' }
        'o2' { return 'O2' }
        'o3' { return 'O3' }
        'og' { return 'Og' }
        'os' { return 'Os' }
        'ofast' { return 'Ofast' }
        default { return 'O0' }
    }
}

function Get-DebugLevelFlag {
    param([string]$RawValue)

    switch ((Convert-EnumeratedOptionValue -RawValue $RawValue).ToLowerInvariant()) {
        'g0' { return 'g0' }
        'g1' { return 'g1' }
        'g2' { return 'g2' }
        'g3' { return 'g3' }
        default { return 'g3' }
    }
}

function Resolve-ArtifactName {
    param(
        [string]$RawValue,
        [string]$ProjectName
    )

    if ([string]::IsNullOrWhiteSpace($RawValue) -or $RawValue -eq '${ProjName}') {
        return $ProjectName
    }

    return $RawValue.Replace('${ProjName}', $ProjectName)
}

function Find-LatestDirectory {
    param(
        [string]$ParentPath,
        [string]$Filter
    )

    if (-not (Test-Path -LiteralPath $ParentPath)) {
        return $null
    }

    return Get-ChildItem -Path $ParentPath -Directory -Filter $Filter |
        Sort-Object Name -Descending |
        Select-Object -First 1
}

function Get-CubeIdeRoot {
    param([string]$ExplicitRoot)

    if (-not [string]::IsNullOrWhiteSpace($ExplicitRoot)) {
        $resolved = Get-AbsolutePath -BasePath (Get-Location).Path -PathValue $ExplicitRoot
        if (Test-Path -LiteralPath $resolved) {
            return $resolved
        }
        throw "Khong tim thay CubeIDE root: $resolved"
    }

    $candidates = @(
        'C:\ST',
        'C:\Program Files\STMicroelectronics',
        'C:\Program Files (x86)\STMicroelectronics'
    )

    foreach ($parent in $candidates) {
        $latest = Find-LatestDirectory -ParentPath $parent -Filter 'STM32CubeIDE_*'
        if ($null -ne $latest) {
            return $latest.FullName
        }
    }

    throw 'Khong tim thay STM32CubeIDE. Hay truyen -CubeIdeRoot neu IDE nam o duong dan khac.'
}

function Get-CubePluginExecutable {
    param(
        [string]$CubeIdeRoot,
        [string]$PluginFilter,
        [string]$RelativePath
    )

    $pluginsPath = Join-Path $CubeIdeRoot 'STM32CubeIDE\plugins'
    $plugin = Find-LatestDirectory -ParentPath $pluginsPath -Filter $PluginFilter
    if ($null -eq $plugin) {
        throw "Khong tim thay plugin CubeIDE: $PluginFilter"
    }

    $fullPath = Join-Path $plugin.FullName $RelativePath
    if (-not (Test-Path -LiteralPath $fullPath)) {
        throw "Khong tim thay file trong plugin $($plugin.Name): $RelativePath"
    }
    return $fullPath
}

function Get-SvdFile {
    param(
        [string]$CubeIdeRoot,
        [string]$DeviceName
    )

    $svdDir = Get-CubePluginExecutable -CubeIdeRoot $CubeIdeRoot -PluginFilter 'com.st.stm32cube.ide.mcu.productdb.debug_*' -RelativePath 'resources\cmsis\STMicroelectronics_CMSIS_SVD'
    $svdRoot = Split-Path -Parent $svdDir
    $svdSearchRoot = Join-Path $svdRoot 'STMicroelectronics_CMSIS_SVD'
    if (-not (Test-Path -LiteralPath $svdSearchRoot)) {
        $svdSearchRoot = $svdDir
    }

    $normalizedDevice = $DeviceName.ToUpperInvariant().Replace('X', 'X')
    $best = $null
    $bestScore = -1

    foreach ($file in Get-ChildItem -Path $svdSearchRoot -Filter '*.svd' -File -Recurse) {
        $pattern = [System.IO.Path]::GetFileNameWithoutExtension($file.Name).ToUpperInvariant()
        $maxLength = [Math]::Min($pattern.Length, $normalizedDevice.Length)
        $matches = $true
        $score = 0

        for ($i = 0; $i -lt $maxLength; $i++) {
            $patternChar = $pattern[$i]
            $deviceChar = $normalizedDevice[$i]
            if ($patternChar -eq 'X') {
                continue
            }
            if ($patternChar -ne $deviceChar) {
                $matches = $false
                break
            }
            $score += 2
        }

        if (-not $matches) {
            continue
        }

        if ($pattern.Length -le $normalizedDevice.Length) {
            $score += $pattern.Length
        }

        if ($score -gt $bestScore) {
            $best = $file.FullName
            $bestScore = $score
        }
    }

    if ($null -eq $best) {
        throw "Khong tim thay file SVD phu hop cho device $DeviceName"
    }

    return $best
}

function Get-SourceDirectories {
    param(
        [System.Xml.XmlNode]$ConfigNode,
        [string]$ProjectName
    )

    $paths = Get-XmlListAttributeByXPath -RootNode $ConfigNode -XPath './sourceEntries/entry[@kind="sourcePath"]' -AttributeName 'name'
    $normalized = foreach ($path in $paths) {
        Resolve-CubeProjectValue -Value $path -ProjectName $ProjectName
    }
    return Get-UniqueStrings -Values $normalized
}

function Find-StartupFile {
    param(
        [string]$ProjectRoot,
        [string]$DeviceName
    )

    $candidates = @(
        Get-ChildItem -Path $ProjectRoot -Filter 'startup_*.s' -File -Recurse
        Get-ChildItem -Path $ProjectRoot -Filter 'startup_*.S' -File -Recurse
    ) | Sort-Object FullName -Unique

    if ($candidates.Count -eq 0) {
        throw 'Khong tim thay startup_*.s hoac startup_*.S trong project.'
    }

    $normalizedDevice = $DeviceName.ToLowerInvariant().Replace('(', '').Replace(')', '').Replace('-', '')
    foreach ($candidate in $candidates) {
        $name = $candidate.Name.ToLowerInvariant().Replace('startup_', '').Replace('.s', '').Replace('x', '')
        if ($normalizedDevice.Contains($name)) {
            return $candidate.FullName
        }
    }

    return $candidates[0].FullName
}

function Find-LinkerScript {
    param(
        [string]$ProjectRoot,
        [string]$DeclaredValue,
        [string]$ProjectName
    )

    $resolved = Resolve-CubeProjectValue -Value $DeclaredValue -ProjectName $ProjectName
    if (-not [string]::IsNullOrWhiteSpace($resolved)) {
        $candidate = Get-AbsolutePath -BasePath $ProjectRoot -PathValue $resolved
        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }

    $fallback = Get-ChildItem -Path $ProjectRoot -Filter '*_FLASH.ld' -File | Select-Object -First 1
    if ($null -eq $fallback) {
        throw 'Khong tim thay linker script *_FLASH.ld trong project.'
    }
    return $fallback.FullName
}

function Write-JsonFile {
    param(
        [string]$Path,
        [object]$Data,
        [switch]$AllowOverwrite
    )

    if ((Test-Path -LiteralPath $Path) -and -not $AllowOverwrite) {
        Write-Output "Bo qua file da ton tai: $Path"
        return
    }

    $directory = Split-Path -Parent $Path
    if (-not (Test-Path -LiteralPath $directory)) {
        New-Item -ItemType Directory -Force -Path $directory | Out-Null
    }

    $Data | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath $Path -Encoding ascii
    Write-Output "Da tao: $Path"
}

$projectRootPath = Get-AbsolutePath -BasePath (Get-Location).Path -PathValue $ProjectRoot
$iocFile = Get-ProjectFile -Root $projectRootPath -Pattern '*.ioc'
$cprojectFile = Join-Path $projectRootPath '.cproject'

if ($null -eq $iocFile) {
    throw 'Khong tim thay file .ioc o thu muc goc cua project.'
}
if (-not (Test-Path -LiteralPath $cprojectFile)) {
    throw 'Khong tim thay file .cproject o thu muc goc cua project.'
}

$iocProperties = Read-IocProperties -IocPath $iocFile.FullName
$projectName = Get-ProjectName -ProjectRoot $projectRootPath -IocProperties $iocProperties

[xml]$cprojectXml = Get-Content -LiteralPath $cprojectFile -Raw
$configXPath = "//storageModule[@moduleId='cdtBuildSystem']/configuration[@name='$Configuration']"
$configNode = $cprojectXml.SelectSingleNode($configXPath)
if ($null -eq $configNode) {
    throw "Khong tim thay cau hinh $Configuration trong .cproject"
}

$cubeIdeRootPath = Get-CubeIdeRoot -ExplicitRoot $CubeIdeRoot
$gccPath = Get-CubePluginExecutable -CubeIdeRoot $cubeIdeRootPath -PluginFilter 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*' -RelativePath 'tools\bin\arm-none-eabi-gcc.exe'
$gdbPath = Join-Path (Split-Path -Parent $gccPath) 'arm-none-eabi-gdb.exe'
$armToolchainBin = Split-Path -Parent $gccPath
$cubeProgrammerCli = Get-CubePluginExecutable -CubeIdeRoot $cubeIdeRootPath -PluginFilter 'com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*' -RelativePath 'tools\bin\STM32_Programmer_CLI.exe'
$cubeProgrammerBin = Split-Path -Parent $cubeProgrammerCli
$stlinkGdbServer = Get-CubePluginExecutable -CubeIdeRoot $cubeIdeRootPath -PluginFilter 'com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.*' -RelativePath 'tools\bin\ST-LINK_gdbserver.exe'

$artifactNameRaw = Get-XmlAttributeValue -Node $configNode -AttributeName 'artifactName'
$artifactName = Resolve-ArtifactName -RawValue $artifactNameRaw -ProjectName $projectName

$deviceName = $iocProperties['ProjectManager.DeviceId']
if ([string]::IsNullOrWhiteSpace($deviceName)) {
    $deviceName = Get-XmlNodeAttributeByXPath -RootNode $configNode -XPath './folderInfo/toolChain/option[contains(@superClass,"option.target_mcu")]' -AttributeName 'value'
}

$familyName = $iocProperties['Mcu.Family']
$cpu = Get-CortexCpuFromFamily -Family $familyName -DeviceName $deviceName
$fpuValue = Convert-EnumeratedOptionValue -RawValue (Get-XmlNodeAttributeByXPath -RootNode $configNode -XPath './folderInfo/toolChain/option[contains(@superClass,"option.fpu")]' -AttributeName 'value')
$floatAbiValue = Convert-EnumeratedOptionValue -RawValue (Get-XmlNodeAttributeByXPath -RootNode $configNode -XPath './folderInfo/toolChain/option[contains(@superClass,"option.floatabi")]' -AttributeName 'value')

if ($fpuValue -eq 'none') {
    $fpuValue = ''
}
if ([string]::IsNullOrWhiteSpace($floatAbiValue)) {
    $floatAbiValue = 'soft'
}

$debugLevel = Get-DebugLevelFlag -RawValue (Get-XmlNodeAttributeByXPath -RootNode $configNode -XPath './/tool[contains(@superClass,"tool.c.compiler")]/option[contains(@superClass,"tool.c.compiler.option.debuglevel")]' -AttributeName 'value')
$optimization = Get-OptimizationFlag -RawValue (Get-XmlNodeAttributeByXPath -RootNode $configNode -XPath './/tool[contains(@superClass,"tool.c.compiler")]/option[contains(@superClass,"tool.c.compiler.option.optimization.level")]' -AttributeName 'value')

$includeDirs = Get-XmlListAttributeByXPath -RootNode $configNode -XPath './/tool[contains(@superClass,"tool.c.compiler")]/option[contains(@superClass,"tool.c.compiler.option.includepaths")]/listOptionValue' -AttributeName 'value'
$includeDirs = Get-UniqueStrings -Values ($includeDirs | ForEach-Object { Resolve-CubeProjectValue -Value $_ -ProjectName $projectName })

$cDefines = Get-XmlListAttributeByXPath -RootNode $configNode -XPath './/tool[contains(@superClass,"tool.c.compiler")]/option[contains(@superClass,"tool.c.compiler.option.definedsymbols")]/listOptionValue' -AttributeName 'value'
$asmDefines = Get-XmlListAttributeByXPath -RootNode $configNode -XPath './/tool[contains(@superClass,"tool.assembler")]/option[contains(@superClass,"tool.assembler.option.definedsymbols")]/listOptionValue' -AttributeName 'value'
$defines = Get-UniqueStrings -Values ($cDefines + $asmDefines)

$sourceDirs = Get-SourceDirectories -ConfigNode $configNode -ProjectName $projectName
if ($sourceDirs.Count -eq 0) {
    $sourceDirs = Get-UniqueStrings -Values @('Core', 'Drivers', 'Middlewares')
}

$librarySearchDirs = Get-XmlListAttributeByXPath -RootNode $configNode -XPath './/tool[contains(@superClass,"tool.c.linker")]/option[contains(@superClass,"tool.c.linker.option.directories")]/listOptionValue' -AttributeName 'value'
$librarySearchDirs = Get-UniqueStrings -Values ($librarySearchDirs | ForEach-Object { Resolve-CubeProjectValue -Value $_ -ProjectName $projectName })

$projectLibraries = Get-XmlListAttributeByXPath -RootNode $configNode -XPath './/tool[contains(@superClass,"tool.c.linker")]/option[contains(@superClass,"tool.c.linker.option.libraries")]/listOptionValue' -AttributeName 'value'
$projectLibraries = Get-UniqueStrings -Values ($projectLibraries | ForEach-Object { Normalize-LinkerLibraryValue -Value $_ })
$linkerScript = Find-LinkerScript -ProjectRoot $projectRootPath -DeclaredValue (Get-XmlNodeAttributeByXPath -RootNode $configNode -XPath './/tool[contains(@superClass,"tool.c.linker")]/option[contains(@superClass,"tool.c.linker.option.script")]' -AttributeName 'value') -ProjectName $projectName
$startupFile = Find-StartupFile -ProjectRoot $projectRootPath -DeviceName $deviceName
$svdFile = Get-SvdFile -CubeIdeRoot $cubeIdeRootPath -DeviceName $deviceName

$linkerSymbols = New-Object System.Collections.Generic.List[string]
if ((Get-XmlNodeAttributeByXPath -RootNode $configNode -XPath './folderInfo/toolChain/option[contains(@superClass,"option.nanoprintffloat")]' -AttributeName 'value') -eq 'true') {
    $linkerSymbols.Add('_printf_float')
}
if ((Get-XmlNodeAttributeByXPath -RootNode $configNode -XPath './folderInfo/toolChain/option[contains(@superClass,"option.nanoscanffloat")]' -AttributeName 'value') -eq 'true') {
    $linkerSymbols.Add('_scanf_float')
}

$settingsData = [ordered]@{
    'C_Cpp.default.compileCommands' = '${workspaceFolder}/compile_commands.json'
    'C_Cpp.default.compilerPath' = $gccPath.Replace('\', '/')
    'C_Cpp.default.intelliSenseMode' = 'windows-gcc-x64'
    'C_Cpp.default.cStandard' = 'gnu11'
    'C_Cpp.default.cppStandard' = 'gnu++17'
    'C_Cpp.intelliSenseEngineFallback' = 'Enabled'
    'clangd.arguments' = @(
        '--compile-commands-dir=${workspaceFolder}',
        ('--query-driver=' + (($armToolchainBin.Replace('\', '/')) + '/arm-none-eabi-*'))
    )
    'stm32.tools.armToolchainBin' = $armToolchainBin.Replace('\', '/')
    'stm32.tools.gcc' = $gccPath.Replace('\', '/')
    'stm32.tools.gdb' = $gdbPath.Replace('\', '/')
    'stm32.tools.stlinkGdbServer' = $stlinkGdbServer.Replace('\', '/')
    'stm32.tools.cubeProgrammerBin' = $cubeProgrammerBin.Replace('\', '/')
    'stm32.tools.cubeProgrammerCli' = $cubeProgrammerCli.Replace('\', '/')
    'stm32.project.artifactName' = $artifactName
    'stm32.project.buildDir' = 'build/vscode'
    'stm32.project.cpu' = $cpu
    'stm32.project.fpu' = $fpuValue
    'stm32.project.floatAbi' = $floatAbiValue
    'stm32.project.cStandard' = 'gnu11'
    'stm32.project.debugLevel' = $debugLevel
    'stm32.project.optimization' = $optimization
    'stm32.project.device' = $deviceName
    'stm32.project.runToEntryPoint' = 'main'
    'stm32.project.svdFile' = $svdFile.Replace('\', '/')
    'stm32.project.linkerScript' = (Get-ProjectRelativePath -ProjectRoot $projectRootPath -PathValue $linkerScript)
    'stm32.project.startupFile' = (Get-ProjectRelativePath -ProjectRoot $projectRootPath -PathValue $startupFile)
    'stm32.project.flashAddress' = '0x08000000'
    'stm32.project.sourceDirs' = $sourceDirs
    'stm32.project.includeDirs' = $includeDirs
    'stm32.project.defines' = $defines
    'stm32.project.compilerExtraArgs' = @()
    'stm32.project.librarySearchDirs' = $librarySearchDirs
    'stm32.project.linkerSpecs' = @(
        '--specs=nosys.specs',
        '--specs=nano.specs'
    )
    'stm32.project.linkerSymbols' = $linkerSymbols.ToArray()
    'stm32.project.linkerExtraArgs' = @(
        '-static'
    )
    'stm32.project.libraries' = @(
        '-lc',
        '-lm'
    ) + $projectLibraries
    'stm32.debug.interface' = 'swd'
    'stm32.flash.connectArgs' = @(
        'port=SWD',
        'mode=UR',
        'reset=HWrst'
    )
    'stm32.flash.extraArgs' = @(
        '-v',
        '-rst'
    )
    'files.exclude' = [ordered]@{
        '**/.validate_build' = $true
    }
}

$vscodeDir = Join-Path $projectRootPath '.vscode'
$settingsPath = Join-Path $vscodeDir 'settings.json'
$tasksPath = Join-Path $vscodeDir 'tasks.json'
$launchPath = Join-Path $vscodeDir 'launch.json'
$cppPropsPath = Join-Path $vscodeDir 'c_cpp_properties.json'
$extensionsPath = Join-Path $vscodeDir 'extensions.json'

$tasksData = [ordered]@{
    version = '2.0.0'
    tasks = @(
        [ordered]@{
            label = 'STM32: Clean'
            type = 'shell'
            command = 'powershell'
            args = @('-ExecutionPolicy', 'Bypass', '-File', '${workspaceFolder}/tools/vscode/stm32.ps1', '-Action', 'clean')
            options = [ordered]@{ cwd = '${workspaceFolder}' }
            problemMatcher = @()
        },
        [ordered]@{
            label = 'STM32: Build'
            type = 'shell'
            command = 'powershell'
            args = @('-ExecutionPolicy', 'Bypass', '-File', '${workspaceFolder}/tools/vscode/stm32.ps1', '-Action', 'build')
            options = [ordered]@{ cwd = '${workspaceFolder}' }
            group = [ordered]@{
                kind = 'build'
                isDefault = $true
            }
            problemMatcher = @('$gcc')
        },
        [ordered]@{
            label = 'STM32: Rebuild'
            dependsOrder = 'sequence'
            dependsOn = @('STM32: Clean', 'STM32: Build')
            problemMatcher = @()
        },
        [ordered]@{
            label = 'STM32: Flash'
            dependsOrder = 'sequence'
            dependsOn = @('STM32: Build')
            type = 'shell'
            command = 'powershell'
            args = @('-ExecutionPolicy', 'Bypass', '-File', '${workspaceFolder}/tools/vscode/stm32.ps1', '-Action', 'flash')
            options = [ordered]@{ cwd = '${workspaceFolder}' }
            problemMatcher = @()
        },
        [ordered]@{
            label = 'STM32: Update compile_commands'
            type = 'shell'
            command = 'powershell'
            args = @('-ExecutionPolicy', 'Bypass', '-File', '${workspaceFolder}/tools/vscode/stm32.ps1', '-Action', 'compile-commands')
            options = [ordered]@{ cwd = '${workspaceFolder}' }
            problemMatcher = @()
        }
    )
}

$launchData = [ordered]@{
    version = '0.2.0'
    configurations = @(
        [ordered]@{
            name = 'STM32: Debug (Cortex-Debug + ST-LINK)'
            type = 'cortex-debug'
            request = 'launch'
            cwd = '${workspaceFolder}'
            preLaunchTask = 'STM32: Build'
            servertype = 'stlink'
            serverpath = '${config:stm32.tools.stlinkGdbServer}'
            stm32cubeprogrammer = '${config:stm32.tools.cubeProgrammerBin}'
            armToolchainPath = '${config:stm32.tools.armToolchainBin}'
            gdbPath = '${config:stm32.tools.gdb}'
            executable = '${workspaceFolder}/${config:stm32.project.buildDir}/${config:stm32.project.artifactName}.elf'
            device = '${config:stm32.project.device}'
            interface = '${config:stm32.debug.interface}'
            runToEntryPoint = '${config:stm32.project.runToEntryPoint}'
            svdFile = '${config:stm32.project.svdFile}'
            showDevDebugOutput = 'raw'
        }
    )
}

$cppPropertiesData = [ordered]@{
    version = 4
    configurations = @(
        [ordered]@{
            name = 'STM32'
            compileCommands = '${workspaceFolder}/compile_commands.json'
            intelliSenseMode = 'windows-gcc-x64'
            cStandard = 'gnu11'
            cppStandard = 'gnu++17'
        }
    )
}

$extensionsData = [ordered]@{
    recommendations = @(
        'ms-vscode.cpptools',
        'marus25.cortex-debug',
        'ms-vscode.powershell'
    )
}

Write-JsonFile -Path $settingsPath -Data $settingsData -AllowOverwrite:$Force
Write-JsonFile -Path $tasksPath -Data $tasksData -AllowOverwrite:$Force
Write-JsonFile -Path $launchPath -Data $launchData -AllowOverwrite:$Force
Write-JsonFile -Path $cppPropsPath -Data $cppPropertiesData -AllowOverwrite:$Force
Write-JsonFile -Path $extensionsPath -Data $extensionsData -AllowOverwrite:$Force

$stm32ScriptPath = Join-Path $projectRootPath 'tools\vscode\stm32.ps1'
if (-not (Test-Path -LiteralPath $stm32ScriptPath)) {
    throw "Khong tim thay script build: $stm32ScriptPath"
}

& $stm32ScriptPath -Action 'compile-commands'

if ($BuildAfterSetup) {
    & $stm32ScriptPath -Action 'build'
}

Write-Output ''
Write-Output 'Hoan tat khoi tao VS Code cho project STM32.'
Write-Output ("Project root : " + $projectRootPath)
Write-Output ("Configuration: " + $Configuration)
Write-Output ("Device       : " + $deviceName)
Write-Output ("Artifact     : " + $artifactName)
Write-Output ("CPU/FPU      : " + $cpu + ' / ' + $(if ([string]::IsNullOrWhiteSpace($fpuValue)) { 'none' } else { $fpuValue }))
Write-Output ("SVD          : " + $svdFile)
