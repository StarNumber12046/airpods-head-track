# Query Bluetooth SDP records for AirPods at 90:5F:7A:F2:19:B4
# Uses WinRT Bluetooth APIs via PowerShell

[Windows.Devices.Bluetooth.BluetoothDevice, Windows.Devices.Bluetooth, ContentType = WindowsRuntime] | Out-Null
[Windows.Devices.Bluetooth.BluetoothLEDevice, Windows.Devices.Bluetooth, ContentType = WindowsRuntime] | Out-Null
[Windows.Devices.Bluetooth.GenericAttributeProfile.GattDeviceService, Windows.Devices.Bluetooth.GenericAttributeProfile, ContentType = WindowsRuntime] | Out-Null
[Windows.Devices.Bluetooth.GenericAttributeProfile.GattCommunicationStatus, Windows.Devices.Bluetooth.GenericAttributeProfile, ContentType = WindowsRuntime] | Out-Null
[Windows.Devices.Bluetooth.GenericAttributeProfile.GattDeviceServicesResult, Windows.Devices.Bluetooth.GenericAttributeProfile, ContentType = WindowsRuntime] | Out-Null

$asTaskGeneric = ([System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object { $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]

function Await($WinRtTask, $ResultType) {
    $asTask = $asTaskGeneric.MakeGenericMethod($ResultType)
    $netTask = $asTask.Invoke($null, @($WinRtTask))
    $netTask.Wait(-1) | Out-Null
    $netTask.Result
}

# Parse MAC bytes
$macBytes = "90:5F:7A:F2:19:B4".Split(':') | ForEach-Object { [Convert]::ToByte($_, 16) }
$btAddr = 0
for ($i = 0; $i -lt 6; $i++) {
    $btAddr = $btAddr -bor (([UInt64]$macBytes[$i]) -shl ($i * 8))
}
Write-Host "BTH_ADDR: 0x$($btAddr.ToString('X16'))"

# Try classic Bluetooth device
try {
    Write-Host "`n=== Classic Bluetooth Device ==="
    $device = Await ([Windows.Devices.Bluetooth.BluetoothDevice]::FromBluetoothAddressAsync($btAddr)) ([Windows.Devices.Bluetooth.BluetoothDevice])
    Write-Host "Name: $($device.Name)"
    Write-Host "ConnectionStatus: $($device.ConnectionStatus)"
    Write-Host "BluetoothAddress: 0x$($device.BluetoothAddress.ToString('X16'))"
} catch {
    Write-Host "Classic BT device failed: $_"
}

# Try BLE device
try {
    Write-Host "`n=== Bluetooth LE Device ==="
    $bleDevice = Await ([Windows.Devices.Bluetooth.BluetoothLEDevice]::FromBluetoothAddressAsync($btAddr)) ([Windows.Devices.Bluetooth.BluetoothLEDevice])
    Write-Host "Name: $($bleDevice.Name)"
    Write-Host "ConnectionStatus: $($bleDevice.ConnectionStatus)"
    Write-Host "BluetoothAddress: 0x$($bleDevice.BluetoothAddress.ToString('X16'))"
    
    $gattResult = Await $bleDevice.GetGattServicesAsync() ([Windows.Devices.Bluetooth.GenericAttributeProfile.GattDeviceServicesResult])
    Write-Host "GATT Services (Status: $($gattResult.Status)):"
    foreach ($svc in $gattResult.Services) {
        Write-Host "  - $($svc.Uuid) [Handle=$($svc.AttributeHandle)]"
    }
} catch {
    Write-Host "BLE device failed: $_"
}

Write-Host "`nDone."
pause
