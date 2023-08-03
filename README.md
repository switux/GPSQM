# GPSQM
Handheld Sky Quality Meter with GPS and SDCard data storage. This is a byproduct of my AstroWeatherStation as people asked me if I could isolate the SQM feature and bundle it separately.

# Status

This is still under development. Most of the code base comes from my weather station but additional code needs to be written (SDcard, data retrieval, operation modes, ...). I will upload the code when it is good enough.
I am also still looking for the right housing to accomodate all the parts, so the hw layout is not done yet. Protoyping on breadboard is done, though.

A first set of features work:

  - SQM
  - GPS
  - Led display of NELM or MSAS value
  - Last MSAS/NELM/GPS data retrieval via web server

Features to come:

  - Continuous vs. Manually triggered MSAS measurements
  - Data storage on SDCard
  - Data retrieval via web server
  - On/Off web server

# Variants

It can be made standalone by having a waterproof housing and cable gland to provide power (USB). A solar panel version can also be made but would require a few changes in the power supply as the TP4056 requires a different setup to deal with fluctuating solar panel voltage, conversely a DD04CVSA could be used as it is designed to do load sharing. The GPS could also be removed if it is standalone to make some room and reduce the cost.

# Parts

Draft BOM with avg. prices in CHF:
  - Wemos D1 mini ESP32 (3.50)
  - Neo-8m GPS (10.-)
  - TFCard reader + Card (5.-)
  - 4 red digits HT16K33 (3.50)
  - TP4056 charger (4.50)
  - FQP27P06 mosfet (0.50)
  - SR240 diode (0.10)
  - TSL2591 (4.50)
  - Si7021 (1.50)
  - 2000mAh Li-ion battery (4.-)
  - Cables (1.-)
  - Perfboard(s) (1.50)
  - Housing (4.-)?
  - Some switches (1.50)
  - 20°FoV lens (0.25)
  - Some screws (0.50)
  - JST XH (0.10)
  - Various headers (2.-)
  - Female micro usb (0.60)
  - Male micro usb (0.2)
  - Optional external GPS antenna (3.-)

I think that the overall cost of material should be around CHF 50.- if you do it yourself, all parts sourced from aliexpress, which is way cheaper than a commercial SQM. With the pleasure of doing it yourself, of course :-)
