# Neurosky EEG Serial reader
Initially based on [kitschpatrol's Brain library for Arduino](https://github.com/kitschpatrol/Brain), was then adapted on a higher spec board, the MAX32630FTHR. This library was made to work with reading serial off of Neurosky EEG chips. It is setup for working as drag and drop into MBED libraries.

The main difference between this library and the one for Arduino is the overhaul of `Brain::update()` to use a switch statement and properly handle NOP messages from the module. `Brain.cpp` and `Brain.h` should be drag and drop into most projects as long as `RawSerial` is updated to whatever main serial class your board/firmware/platform uses.

Also, this API is meant to function with a serial interrupt routine to quickly get the character, then send it off to update.