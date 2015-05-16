# README #

Manager for HOLUX GPSport 260 (gr260). Works on Linux.

With the program you can:

 * View track list on device

* Download selected track (to one or several files)

* UPLOAD track to device (for track follow feature)

* Clear memory

In addition it writes speed, height (barometer data) and heartrate


Attention! For the latest version of the program please visit https://bitbucket.org/Shura0/260babel

### Short help ###

```
#!bash

Usage: 260babel [options]
Options:
	-b                       Also save raw data in .bin file
	-B <filename>            Read binary file instead of device
	-d <tracklist>           List tracks to get, comma separated.
                             Special values: a - all tracks
                                             l - last track
                                             Example: -d 1,2,3,5,6 or -d l
	-f <filename>            Base name for saved files (.bin and .gpx)
	-h                       Print this message and exit
	-l                       Print track list
	-m                       Save tracks in one file
	-p <port>                Communication port, default: /dev/ttyUSB0
	-r                       Remove all data
	-s <speed>               Serial port speed, default 38400 baud
	-u <filename>            Upload GPX file to device.

Example:
	Download last track and waypoints from GPS device

	260babel -f gpsdata -d l

```

### Installation ###

1. 
- On deb-based distros install 'libdevice-serialport-perl' package
- On rpm-based distros install 'perl-Device-SerialPort' package

2. Download 260babel.pl and run.

Author is Alexander Zaitsev <shura0@yandex.ru>
