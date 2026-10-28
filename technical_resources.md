# Technical resources consulted

The resources below were identified by the author as development aids. Primary documents are used to verify hardware specifications; community discussions are preserved as practical background, not experimental proof.

| Resource | URL | Interpretation |
|---|---|---|
| RS Raspberry Pi 3 document | https://us.rs-online.com/m/d/4252b1ecd92888dbb9d8a39b536e7bf2.pdf | This is Model B, not Model B+. Retained as a consulted resource. |
| Pinout.xyz | https://pinout.xyz/ | Practical BCM/physical pin guide; final mapping checked against official schematic. |
| Raspberry Pi forum | https://forums.raspberrypi.com/viewtopic.php?t=278276#p1685229 | ECHO-divider advice appears in a thread originally about a detached transducer. |
| Arduino Stack Exchange | https://arduino.stackexchange.com/questions/92011/using-multiple-ultrasonic-sensors-simultanously-with-the-same-trigger | Shared simultaneous triggering discussion; controller implementation uses sequential triggering. |
| Official B+ product brief | https://datasheets.raspberrypi.com/rpi3/raspberry-pi-3-b-plus-product-brief.pdf | Model-specific primary document, published October 2025. |
| Official B+ reduced schematic | https://datasheets.raspberrypi.com/rpi3/raspberry-pi-3-b-plus-reduced-schematics.pdf | Primary header mapping; revision dated 19 March 2018. |
| HC-SR04 sheet | https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf | Recommends cycles longer than 60 ms; rapid 25 ms recheck pause is not a measured cycle or interference characterization. |

Access date: 13 October 2026. Resources are linked, not reproduced in full. No source code was changed in this review.

## Additional resources

- Arduino-ESP32 migration guide: https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html (API-family evidence, not installed release).
- LEDC API: https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html (existing reference retained).
- SN754410: https://www.ti.com/lit/ds/symlink/sn754410.pdf (existing reference retained; tracking parameters removed).
- Legacy ESP-WROOM-32 datasheet v2.4: https://www.mouser.com/datasheet/2/891/esp-wroom-32_datasheet_en-1223836.pdf (consulted background; not WROOM-32E specifications).
- esptool troubleshooting: https://docs.espressif.com/projects/esptool/en/latest/esp32/troubleshooting.html (upload guidance; no particular fault asserted).

Materials were provided by the supervisors. No acquisition-cost measurement or dimensional survey is claimed.
