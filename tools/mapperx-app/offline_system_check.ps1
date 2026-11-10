# Generated copies receive build-time hashes. No URLs or downloads are used here.
$ErrorActionPreference = 'Stop'
if (-not [Environment]::Is64BitOperatingSystem -or [Environment]::OSVersion.Version.Major -lt 10) {
    throw 'Shuffleboard requires Windows 10/11 x64.'
}
function Install-EmbeddedPrerequisite([string]$name, [string]$expected) {
    $folder = Join-Path $env:LOCALAPPDATA ('Shuffleboard\prerequisites\' + $expected.Substring(0, 16))
    [IO.Directory]::CreateDirectory($folder) | Out-Null
    $target = Join-Path $folder $name
    if (-not (Test-Path -LiteralPath $target) -or (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -ne $expected) {
        $begin = '# ==== SHUFFLEBOARD PREREQUISITE ' + $name + ' BEGIN ===='
        $end = '# ==== SHUFFLEBOARD PREREQUISITE ' + $name + ' END ===='
        $out = [IO.File]::Create($target + '.tmp')
        $buffer = New-Object Text.StringBuilder
        $inside = $false
        $complete = $false
        try {
            foreach ($line in [IO.File]::ReadLines($env:PYFILE)) {
                if ($line -eq $begin) { $inside = $true; continue }
                if ($inside -and $line -eq $end) { $complete = $true; break }
                if ($inside -and $line.StartsWith('# ') -and -not $line.StartsWith('# meta ')) {
                    [void]$buffer.Append($line.Substring(2))
                    if ($buffer.Length -ge 1048576) {
                        $bytes = [Convert]::FromBase64String($buffer.ToString())
                        $out.Write($bytes, 0, $bytes.Length)
                        [void]$buffer.Clear()
                    }
                }
            }
            if ($buffer.Length) {
                $bytes = [Convert]::FromBase64String($buffer.ToString())
                $out.Write($bytes, 0, $bytes.Length)
            }
        } finally { $out.Close() }
        if (-not $complete -or (Get-FileHash -LiteralPath ($target + '.tmp') -Algorithm SHA256).Hash -ne $expected) {
            throw ('Damaged offline prerequisite: ' + $name)
        }
        Move-Item -LiteralPath ($target + '.tmp') -Destination $target -Force
    }
    Write-Host ('Installing bundled prerequisite, no internet: ' + $name)
    $process = Start-Process -FilePath $target -Wait -PassThru
    if ($process.ExitCode -eq 3010 -or $process.ExitCode -eq 1641) {
        Write-Host 'Restart Windows, then launch Shuffleboard again.'
        exit 3010
    }
    if ($process.ExitCode -ne 0) { throw ('Installer returned ' + $process.ExitCode) }
}
$net = Get-ItemProperty -LiteralPath 'HKLM:\SOFTWARE\Microsoft\NET Framework Setup\NDP\v4\Full' -ErrorAction SilentlyContinue
if (-not $net -or $net.Release -lt 528040) {
    Install-EmbeddedPrerequisite 'NET-Framework-4.8.exe' '@@NET_HASH@@'
}
$guid = '{F3017226-FE2A-4295-8BDF-00C3A9A7E4C5}'
$edge = @('HKCU:\Software\Microsoft\EdgeUpdate\Clients\', 'HKLM:\SOFTWARE\WOW6432Node\Microsoft\EdgeUpdate\Clients\') |
    ForEach-Object { Get-ItemProperty -LiteralPath ($_ + $guid) -ErrorAction SilentlyContinue } |
    Where-Object { $_.pv -and ([version]$_.pv).Major -ge 86 }
if (-not $edge) { Install-EmbeddedPrerequisite 'WebView2-x64.exe' '@@WEBVIEW_HASH@@' }
exit 0
