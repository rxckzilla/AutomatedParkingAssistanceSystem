# Supplementary material

Complete code copies correspond to the analyzed controller v37, Robot v44 and Car 2 v12. Original filenames and SHA-256 hashes are recorded in source_manifest.json. The original files are preserved. Wi-Fi SSID and password assignments in the supplied firmware are configuration placeholders; configure them before use. The controller address and RFID associations must be verified for the target setup.

Firmware folders match their .ino filenames for opening in the Arduino environment. Required code libraries and hardware interfaces are discussed in chapter 4. The controller requests Python 3. Both firmware programs used the same Arduino environment with the ESP32 Dev Module board profile for compilation and upload, as confirmed by the author; both used Espressif Systems Arduino-ESP32 core 3.2.1. The controller ran under Raspbian Stretch (GNU/Linux 9) on the Raspberry Pi 3 Model B+. Project revisions v37/v44/v12 are not toolchain releases. These supplied files were not flashed or physically re-tested during appendix preparation.

The excerpts reproduce the original addressing function, Robot pause/resume functions, and the initial Car 2 recipient-validation portion. The Car 2 excerpt is a partial function and cannot be compiled by itself. PDF line numbers refer to the analyzed original source.

evaluation_results.json contains the author's aggregate outcomes from 20 attempts per scenario variant under a fixed configuration: 180 successes and 20 failures. It is not a fabricated collection of individual trial records. The chapter 5 acceptance criteria and limitations apply.

The photographs remain in ../images under the filenames supplied by the author. They document the hardware and physical arrangement; a photograph is not a measurement of success rate, alignment error or the cause of a failure.
