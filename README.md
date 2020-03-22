# Components

* [Celestron NexStar GoTo Mount](https://www.ebay.com/itm/Celestron-Astro-Fi-Computerized-GoTo-Mount-Complete-Mount-NEW/402029171407?_trkparms=aid%3D111001%26algo%3DREC.SEED%26ao%3D1%26asc%3D20160811114145%26meid%3Dac0b70c81d164dd9bf6b6775530718f0%26pid%3D100667%26rk%3D2%26rkt%3D8%26mehot%3Dnone%26sd%3D303235523326%26itm%3D402029171407%26pmt%3D0%26noa%3D1%26pg%3D2334524&_trksid=p2334524.c100667.m2042)
* [Celestron compatible dovetail mount](https://www.amazon.com/gp/product/B07LGN4K6L/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1)
* [RPLiDAR A1/A2/A3](https://www.dfrobot.com/search-RPLIDAR.html) depending your needs and budge
* 3D Print the model on the `models/` folder

## Drivers

The `Celestron` driver sends [serial commands to the Celestron HandController](http://www.nexstarsite.com/download/manuals/NexStarCommunicationProtocolV1.2.zip) though the Mini-USB port at the bottom of it.

# Installation

```bash
sudo usermod -a -G dialout $USER
```