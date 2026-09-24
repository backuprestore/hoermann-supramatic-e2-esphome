# hoermann-supramatic-e2-esphome
connect your old hörmann supramatic e2 with esphome to homeassistant


all credits to https://github.com/stephan192/hoermann_door

just some tweaks for e2 drive, because status just know 

00 stopped/moving
01 open
02 closed

there is not "moving upward" like with e3 motors

see here: https://github.com/stephan192/hoermann_door/issues/7

also if moving upward the stop impulse 0x1004 is not working, instead you have to send "down" signal to stop

*** hardware ***
MAX485
ESP32
Lm2596

6pin RJ12 cable

![alt text](image.png)
![alt text](image-1.png)
![alt text](image-2.png)
![alt text](image-3.png)



*** protocoll ***
00 00 01 02 CA -status  |
00 8C 02 01 80 AA - bus scan slave 8C   |
00 00 01 02 CA -status   |
00 8B 02 01 80 C8 - bus scan slave 8B  -> decreasing further and starts again until slave found |


