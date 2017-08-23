## Building kernel module dm9601 USB LAN for RHEL6.x/CentOS6.x

#### Building 
```bash
shell$ sudo make 
```

#### Installation

```bash
shell$ sudo make install
````

#### Clean the source 
```bash
shell$ sudo make clean
```


##### NOTE:

This dm9601.c is a modified version of original code from linux kernel branch tree
tailor fit to work on any USB LAN chipset DM9601 (CDR-KING model in the Philippines) 
I do not guarranty any of it to work on your USB LAN device

PROBLEM/LIMITATION
* this works only at USB 1.1 compatibility and speed 
* Placing two USB LAN (CDR-KING) in one server/pc will cause MAC address problem (I have 2 of this with same MAC)
* Speed is terrible slow sometimes disconnect
* Reliability is a big question (not to be used in production)
