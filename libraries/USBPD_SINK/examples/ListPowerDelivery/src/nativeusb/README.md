The CH32X035 native USB CDC device code is adapted from WCH's
[SimulateCDC example](https://github.com/openwch/ch32x035/tree/main/EVT/EXAM/USB/USBFS/DEVICE/SimulateCDC).
The WCH source notices and restrictions apply. The UART bridge has been
replaced with minimal CDC state because this example sends data directly.

On CH32X035, select the 48 MHz clock and open the board's native USB serial
port at 115200 baud. The capability list and numeric `USBPDStatus` are
reported on status changes and when the monitor opens. This example
still requests the first advertised capability at 1 A; it does not request
9 V. Other chips continue to use their configured hardware `Serial` port.
