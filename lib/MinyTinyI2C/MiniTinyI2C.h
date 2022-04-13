/*
MiniTinyI2C
Copyright (C) 2022

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Author: Robert Alm, almrobert@gmail.com
*/
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdio.h>

void initMiniTinyI2C(const uint16_t baud);
uint8_t readMiniTinyI2C(bool stop);
bool writeMiniTinyI2C(uint8_t data);
bool startMiniTinyI2C(uint8_t address, bool read);
void stopMiniTinyI2C();

#ifdef __cplusplus
}
#endif
