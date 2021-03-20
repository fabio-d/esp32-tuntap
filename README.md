# esp32-tuntap: ESP32 as a wireless dongle

This project consists of an ESP32 firmware and a Linux application that
transform an ESP32 board into a wireless (WiFi) dongle for a Linux PC, acting
either as a wireless client (STA) or an access point (AP).

## The Linux application

The `esp32-tuntap.py` python script initializes the ESP32's network module and
then tunnels its packets to/from a Linux system, connected over the ESP32
board's built-in USB serial.

The [pyserial](https://pypi.org/project/pyserial) and
[python-pytun](https://pypi.org/project/python-pytun) modules are required. They
can be installed with `pip` as usual; for instance, the following shell commands
create a python virtual environment and install them:

```shell script
python3 -m venv esp32-tuntap-venv
esp32-tuntap-venv/bin/pip install -r requirements.txt
```

The `esp32-tuntap.py` script can then be launched as follows:

```shell script
esp32-tuntap-venv/bin/python esp32-tuntap.py --help
```

### Examples

*Note*: root privileges are needed to use serial ports and create TUN/TAP
interfaces!

#### Client (STA) mode

To connect to an existing network with static IP `192.168.1.123` (DHCP client is
not supported yet), assuming your router's IP address is `192.168.1.1`:

```shell script
# Run the following commands as root (with either "sudo -s" or "su")

# Configure DNS server (8.8.8.8 is Google's DNS server)
echo nameserver 8.8.8.8 > /etc/resolv.conf

# Run esp32-tuntap agent
esp32-tuntap-venv/bin/python esp32-tuntap.py sta \
  --ssid YourNetworkName \
  --password YourNetworkPassword \
  --local-address 192.168.1.123/24 \
  --gateway-address 192.168.1.1 \
  --up
```

#### Access Point (AP) mode, sharing an existing connection with NAT

To create a NAT wireless network sharing your existing `eth0` connection:

```shell script
# Run the following commands as root (with either "sudo -s" or "su")

# Configure NAT
iptables -t nat -A POSTROUTING -s 192.168.199.0/24 -o eth0 -j MASQUERADE
sysctl -w net.ipv4.ip_forward=1

# Run esp32-tuntap agent
esp32-tuntap-venv/bin/python esp32-tuntap.py ap \
  --ssid ANewNetworkName \
  --password ANewNetworkPassword \
  --local-address 192.168.199.1/24 \
  --up
```

Then, open another shell and run the following commands to run a DHCP+DNS server
on your new wireless network:

```shell script
# Run the following commands as root (with either "sudo -s" or "su")

# Ensure dnsmasq is installed
apt install dnsmasq

# Launch it ("tap0" is the network interface created by esp32-tuntap.py)
dnsmasq --interface=tap0 --no-daemon --dhcp-range=192.168.199.2,192.168.199.254,255.255.255.0
```

Alternatively, if you do not need a DHCP server, clients can be configured with
static IP addresses and the following parameters:
 - IP address: any address in range `192.168.199.2`-`192.168.199.254`
 - Default gateway: `192.168.199.1`
 - DNS server: `8.8.8.8` (to use Google's) or your own

#### Access Point (AP) mode, attached to a bridge

To create a wireless network attached to an existing bridge:

```shell script
# Run the following commands as root (with either "sudo -s" or "su")

# Run esp32-tuntap agent
esp32-tuntap-venv/bin/python esp32-tuntap.py ap \
  --ssid ANewNetworkName \
  --password ANewNetworkPassword \
  --add-to-bridge br0 \
  --up
```

## The firmware

The `esp32-tuntap` firmware is just a regular ESP32 IDF-based application.

This section summarizes the steps needed to build and flash it. Further details
can be found in the
[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/get-started/index.html).

### Preparing the build environment

By default, IDF will download the ESP32 toolchain into the `.espressif` folder
in your home directory. You can *optionally* set a different path with the
`IDF_TOOLS_PATH` environment variable. For instance:

```shell script
export IDF_TOOLS_PATH=/home/fabio/esp32-tuntap/.espressif
```

Clone the git repository and its submodules with:

```shell script
git submodule init
git submodule update --recursive
```
Then, initialize the ESP-IDF environment with:

```shell script
modules/esp-idf/install.sh
```

### Building and flashing the ESP32 board

If a custom `IDF_TOOLS_PATH` value was set when preparing the environment, it
must be set while building too:

```shell script
export IDF_TOOLS_PATH=/home/fabio/esp32-tuntap/.espressif
```

We also need to source a script provided by IDF that will set further
environment variables:

```shell script
. modules/esp-idf/export.sh
```

Then, build the firmware with:

```shell script
cd firmware
idf.py build
```

Lastly, it can be flashed to the connected board with:

```shell script
idf.py flash
```

## Tested boards

### DOIT ESP32 DEVKIT V1 ([link](https://circuits4you.com/2018/12/31/esp32-devkit-esp32-wroom-gpio-pinout))

This is this project's main development board. Everything works out of the box.

### ESP32-CAM ([link](https://randomnerdtutorials.com/esp32-cam-video-streaming-face-recognition-arduino-ide))

Modify `firmware/main/main.cpp` before building the firmware, to select the
proper LED pin and polarity (by commenting the default values and
uncommenting those for ESP32-CAM):

```C++
// ESP32-CAM
static constexpr gpio_num_t ledPin = GPIO_NUM_33;
static constexpr int ledOffValue = 1;
```

Also, in this board the UART cannot reset the ESP32. Therefore, it will be
necessary to reset it manually every time `esp32-tuntap.py` is started (by
pressing the reset button on the back of the board).

### Other boards

Other ESP32 boards are likely to work too, either out of the box or with
minimal modifications to pin mappings.
