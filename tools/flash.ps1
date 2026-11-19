# Compile and (optionally) upload a sketch to the ESP32-C6 using the Arduino IDE's bundled CLI.
#   .\tools\flash.ps1                         # compile turret_probe only
#   .\tools\flash.ps1 -Port COM5              # compile + upload
#   .\tools\flash.ps1 -Sketch turret_probe -Port COM5
# USB CDC On Boot is Enabled, so Serial goes to the board's native "USB" connector.
param(
    [string]$Sketch = "turret_probe",
    [string]$Port = ""
)
$ErrorActionPreference = "Stop"
$cli  = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
$cfg  = "$env:USERPROFILE\.arduinoIDE\arduino-cli.yaml"
$fqbn = "esp32:esp32:esp32c6:CDCOnBoot=cdc"
$root = Split-Path -Parent $PSScriptRoot
$path = Join-Path $root "firmware\$Sketch"
$build = Join-Path $env:TEMP "build_$Sketch"

& $cli --config-file $cfg compile -b $fqbn --build-path $build $path
if ($LASTEXITCODE -ne 0) { throw "compile failed" }
if ($Port) {
    & $cli --config-file $cfg upload -b $fqbn -p $Port --input-dir $build $path
    if ($LASTEXITCODE -ne 0) { throw "upload failed" }
}
