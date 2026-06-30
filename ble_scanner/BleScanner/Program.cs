using Windows.Devices.Bluetooth;
using Windows.Devices.Enumeration;
using Windows.Networking.Sockets;

ulong btAddr = 0;
string[] parts = "90:5F:7A:F2:19:B4".Split(':');
for (int i = 0; i < 6; i++)
    btAddr |= (ulong)Convert.ToByte(parts[i], 16) << (i * 8);

Console.WriteLine($"BTH_ADDR: 0x{btAddr:X16}");

// Enumerate paired devices
Console.WriteLine();
Console.WriteLine("=== Paired Bluetooth Devices ===");
try
{
    var pairedSelector = BluetoothDevice.GetDeviceSelectorFromPairingState(true);
    var devices = await DeviceInformation.FindAllAsync(pairedSelector);
    Console.WriteLine($"Found {devices.Count} paired devices:");
    foreach (var d in devices)
    {
        Console.WriteLine($"  \"{d.Name}\" Id={d.Id}");
    }
}
catch (Exception ex)
{
    Console.WriteLine($"Enum error: {ex.Message}");
}

// Get classic device
Console.WriteLine();
Console.WriteLine("=== Classic Bluetooth Device ===");
try
{
    var device = await BluetoothDevice.FromBluetoothAddressAsync(btAddr);
    if (device == null)
    {
        Console.WriteLine("BluetoothDevice is null");
    }
    else
    {
        Console.WriteLine($"Name: \"{device.Name}\"");
        Console.WriteLine($"ConnectionStatus: {device.ConnectionStatus}");
        Console.WriteLine($"WasSecureConnectionUsed: {device.WasSecureConnectionUsedForPairing}");

        // Check RFCOMM services
        var rfcommResult = await device.GetRfcommServicesAsync();
        Console.WriteLine($"RFCOMM services: {rfcommResult.Services.Count}");
        foreach (var s in rfcommResult.Services)
        {
            Console.WriteLine($"  ServiceId: {s.ServiceId}");
        }
    }
}
catch (Exception ex)
{
    Console.WriteLine($"Classic device error: {ex.Message}");
}

Console.WriteLine();
Console.WriteLine("Done.");
