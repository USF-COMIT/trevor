# TREVOR Network Configuration

## Passwords

**username:**   trevor

password:  bathyopossum&#x20;

## Device Documentation

* Edge Router: [https://dl.ubnt.com/guides/edgemax/EdgeRouter\_ER-X\_QSG.pdf](https://dl.ubnt.com/guides/edgemax/EdgeRouter\_ER-X\_QSG.pdf)
*

## IP Address Scheme

* 10.0.0.0/16:      Infrastructure/shore Subnet
  * 10.0.0.0/24:     backhaul network gear (wifi stuff)
* 10.1.0.0/16:   Shore Gear  (laptops RTK stations)
  * 10.1.1.0/24:    shore station 1   (Default Route)
* 10.2.0.0/16:       Vehicle subnet
  * 10.2.1.0/24:     TREVOR1 network
  * 10.2.n.0/24:    TREVOR(n) network

### Fixed IP addresses

Shore Router: [10.1.1.1](http://10.1.1.1)

TREVOR1 Router:  [10.2.1.1](http://10.2.1.1)

shore-rocket: [10.0.0.2](http://10.0.0.2)

trevor1-rocket [10.0.0.3](http://10.0.0.3)

## Airmax

ssid:  COMIT5G

pw:  bathyopossum

