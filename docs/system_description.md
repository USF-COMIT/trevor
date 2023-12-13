# TREVOR Hardware Systems Description

## Communications schematic
![](/home/kris/ros/trevor_ws/src/trevor/docs/media/trevor_comms_description.drawio.svg)

## Power Schematic
![](/home/kris/ros/trevor_ws/src/trevor/docs/media/trevor_power_diagram.drawio.svg)

## Onboard Computer
 The onboard computer is a Premio Fanless mini pc part no:  RCO-1000-EHL-J6413-20-P.   It is running ubuntu 20.04 with ROS1 Noetic and ROS2 Foxy.
 
The full configuration can be found here:  [Premio Edge PC configuration](media/premio_description.pdf)

NOTE:   at this time we have not been able to communicate through COM3 and COM4.   As a temporary fix we have been using a USB-Serial adapter for additional serial ports.

## Navigation

The navigation system used is the Microstrain 3DM-GQ7.  This device requires some custom wiring not fully described in the diagram above.   Please see the [Microstrain Configuration](microstrain_configuration.md) page for details.

## Shore Station

The shore station consists of Ubiquiti Bullet, an Edge Router, a Field Laptop, and a Logitech gamepad.  Additional PCs or devices may be connected to ports ETH 2 and ETH 3 on the shore EdgeRoutter.

## Network

**The network devices used are:** 
* [Ubiquiti Edge Rounter x (2)](https://store.ui.com/us/en/products/er-x)
* [Ubiquiti airMAX Bullet AC IP67 (2)](https://store.ui.com/us/en/pro/category/all-wireless/products/bulletac-ip67)

**The devices are configured as follows:**

* The shore Edge Router is currently configured as a gateway and administers DHCP
  * This device maps static IPs to all the devices on the system
* The TREVOR Edge Router is in switch mode with POE out turned on for port 4
* The Shore Bullet is in bridge mode and is functioning as the access point for the WLAN
* The TREVOR Bullet is also in bridge mode and functions as a client

## Motor Controller

## Batteries

## Hulls

## 