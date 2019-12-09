/**
 * Copyright (c) 2017 Tara Keeling
 * 
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include <stdint.h>
//#include "config.h"
#include "sdkconfig.h"
#include "font.h"

#ifdef CONFIG_FONT_LIBERATION_SANS_15x16
//WARNING: This Font Require X-GLCD Lib.
//         You can not use it with MikroE GLCD Lib.

//Font Generated by MikroElektronika GLCD Font Creator 1.2.0.0
//MikroElektronika 2011
//http://www.mikroe.com

//GLCD FontName : Liberation_Sans15x16
//GLCD FontSize : 15 x 16

static const uint8_t Liberation_Sans15x16_Data[ ] = {
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char
    0x03, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char !
    0x05, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char "
    0x09, 0x10, 0x02, 0x10, 0x1E, 0xF0, 0x03, 0x1E, 0x02, 0x10, 0x02, 0x10, 0x1E, 0xF8, 0x03, 0x16, 0x02, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char #
    0x09, 0x00, 0x08, 0x3C, 0x18, 0x66, 0x10, 0x42, 0x10, 0xFF, 0x3F, 0xC2, 0x10, 0x82, 0x10, 0x84, 0x0F, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char $
    0x0D, 0x00, 0x00, 0x7E, 0x00, 0x42, 0x00, 0x42, 0x10, 0x7E, 0x0C, 0x00, 0x06, 0x80, 0x01, 0x60, 0x00, 0x38, 0x00, 0x8C, 0x1F, 0x82, 0x10, 0x80, 0x10, 0x80, 0x1F, 0x00, 0x00, 0x00, 0x00, // Code for char %
    0x0B, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x19, 0xF8, 0x10, 0xC4, 0x11, 0x44, 0x13, 0x64, 0x0E, 0x38, 0x0C, 0x00, 0x1B, 0x80, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char &
    0x02, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char '
    0x05, 0x00, 0x00, 0xE0, 0x0F, 0x3C, 0x78, 0x06, 0xC0, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char (
    0x04, 0x02, 0x80, 0x06, 0xC0, 0x3C, 0x78, 0xE0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char )
    0x06, 0x08, 0x00, 0x48, 0x00, 0x38, 0x00, 0x3E, 0x00, 0x48, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char *
    0x08, 0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0xF0, 0x07, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char +
    0x03, 0x00, 0x00, 0x00, 0x18, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char ,
    0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char -
    0x03, 0x00, 0x00, 0x00, 0x18, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char .
    0x04, 0x00, 0x18, 0x80, 0x07, 0x78, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char /
    0x08, 0x00, 0x00, 0xF8, 0x07, 0x04, 0x0C, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x04, 0x0C, 0xF8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 0
    0x08, 0x00, 0x00, 0x00, 0x10, 0x04, 0x10, 0x06, 0x10, 0xFE, 0x1F, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 1
    0x08, 0x00, 0x00, 0x0C, 0x18, 0x06, 0x1C, 0x02, 0x12, 0x02, 0x11, 0x82, 0x10, 0xEE, 0x10, 0x3C, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 2
    0x08, 0x00, 0x00, 0x0C, 0x0C, 0x06, 0x18, 0x42, 0x10, 0x42, 0x10, 0x62, 0x10, 0xF6, 0x18, 0x9C, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 3
    0x09, 0x00, 0x02, 0x80, 0x03, 0xC0, 0x02, 0x30, 0x02, 0x1C, 0x02, 0x06, 0x02, 0xFE, 0x1F, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 4
    0x08, 0x00, 0x00, 0x7E, 0x08, 0x4E, 0x18, 0x22, 0x10, 0x22, 0x10, 0x22, 0x10, 0x62, 0x08, 0xC2, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 5
    0x08, 0x00, 0x00, 0xF8, 0x07, 0xCC, 0x0C, 0x22, 0x10, 0x22, 0x10, 0x22, 0x10, 0x66, 0x18, 0xC4, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 6
    0x08, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x1E, 0xC2, 0x07, 0x72, 0x00, 0x1A, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 7
    0x08, 0x00, 0x00, 0xBC, 0x0F, 0xA6, 0x18, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0xA6, 0x18, 0xBC, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 8
    0x08, 0x00, 0x00, 0xFC, 0x08, 0x86, 0x19, 0x02, 0x11, 0x02, 0x11, 0x02, 0x11, 0x8C, 0x0C, 0xF8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char 9
    0x03, 0x00, 0x00, 0x30, 0x18, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char :
    0x03, 0x00, 0x00, 0x30, 0x18, 0x30, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char ;
    0x08, 0x00, 0x00, 0xC0, 0x01, 0x40, 0x01, 0x40, 0x01, 0x60, 0x03, 0x20, 0x02, 0x20, 0x02, 0x10, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char <
    0x08, 0x00, 0x00, 0x20, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char =
    0x08, 0x00, 0x00, 0x10, 0x04, 0x20, 0x02, 0x20, 0x02, 0x60, 0x03, 0x40, 0x01, 0x40, 0x01, 0xC0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char >
    0x08, 0x00, 0x00, 0x0C, 0x00, 0x06, 0x00, 0x02, 0x18, 0x02, 0x1B, 0x82, 0x00, 0xC6, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char ?
    0x0F, 0x00, 0x00, 0x80, 0x07, 0x70, 0x1C, 0x18, 0x30, 0x84, 0x27, 0xE4, 0x4C, 0x32, 0x48, 0x12, 0x48, 0x12, 0x44, 0x32, 0x42, 0xE2, 0x2F, 0x34, 0x28, 0x0C, 0x0C, 0x38, 0x07, 0xE0, 0x00, // Code for char @
    0x0B, 0x00, 0x10, 0x00, 0x1E, 0x80, 0x03, 0xF0, 0x01, 0x1E, 0x01, 0x02, 0x01, 0x1E, 0x01, 0xF0, 0x01, 0x80, 0x03, 0x00, 0x1E, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char A
    0x0A, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0x66, 0x10, 0xBC, 0x0C, 0x80, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char B
    0x0B, 0x00, 0x00, 0xF0, 0x03, 0x1C, 0x0F, 0x04, 0x08, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x04, 0x08, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char C
    0x0B, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x02, 0x18, 0x04, 0x08, 0x1C, 0x0E, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char D
    0x0A, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0x42, 0x10, 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char E
    0x09, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char F
    0x0B, 0x00, 0x00, 0xF0, 0x03, 0x1C, 0x0E, 0x04, 0x08, 0x02, 0x10, 0x02, 0x10, 0x82, 0x10, 0x82, 0x10, 0x86, 0x18, 0x8C, 0x0F, 0x88, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char G
    0x0A, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char H
    0x02, 0x00, 0x00, 0xFE, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char I
    0x07, 0x00, 0x04, 0x00, 0x0C, 0x00, 0x10, 0x02, 0x10, 0x02, 0x10, 0xFE, 0x0F, 0xFE, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char J
    0x0A, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0xC0, 0x00, 0x60, 0x00, 0xF0, 0x01, 0x18, 0x03, 0x0C, 0x06, 0x06, 0x0C, 0x02, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char K
    0x08, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char L
    0x0C, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x3C, 0x00, 0xE0, 0x00, 0x00, 0x07, 0x00, 0x18, 0x00, 0x0F, 0xE0, 0x01, 0x3C, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char M
    0x0A, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x1C, 0x00, 0x30, 0x00, 0xC0, 0x00, 0x00, 0x03, 0x00, 0x0C, 0xFE, 0x1F, 0xFE, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char N
    0x0B, 0x00, 0x00, 0xF8, 0x03, 0x1C, 0x0E, 0x04, 0x08, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x04, 0x08, 0x1C, 0x0E, 0xF8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char O
    0x0A, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x6C, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char P
    0x0B, 0x00, 0x00, 0xF8, 0x03, 0x1C, 0x0E, 0x04, 0x08, 0x02, 0x10, 0x02, 0x10, 0x02, 0x70, 0x02, 0xD0, 0x04, 0x88, 0x1C, 0x8E, 0xF8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char Q
    0x0A, 0x00, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x82, 0x01, 0x82, 0x07, 0x7C, 0x1C, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char R
    0x0A, 0x00, 0x00, 0x18, 0x0C, 0x3C, 0x08, 0x66, 0x10, 0x42, 0x10, 0x42, 0x10, 0xC2, 0x10, 0x82, 0x10, 0x8C, 0x09, 0x0C, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char S
    0x09, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0xFE, 0x1F, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char T
    0x0A, 0x00, 0x00, 0xFE, 0x03, 0xFE, 0x0F, 0x00, 0x18, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x18, 0xFE, 0x0F, 0xFE, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char U
    0x0B, 0x02, 0x00, 0x1E, 0x00, 0x78, 0x00, 0xC0, 0x03, 0x00, 0x0E, 0x00, 0x10, 0x00, 0x0E, 0xC0, 0x03, 0x78, 0x00, 0x1E, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char V
    0x0F, 0x06, 0x00, 0x3E, 0x00, 0xF0, 0x03, 0x00, 0x1F, 0x00, 0x1C, 0xC0, 0x07, 0x7C, 0x00, 0x06, 0x00, 0x7C, 0x00, 0xC0, 0x07, 0x00, 0x1C, 0x00, 0x1E, 0xE0, 0x03, 0x3E, 0x00, 0x06, 0x00, // Code for char W
    0x0A, 0x00, 0x00, 0x02, 0x18, 0x0E, 0x0C, 0x18, 0x07, 0xB0, 0x01, 0xC0, 0x00, 0xF0, 0x01, 0x18, 0x07, 0x0E, 0x0C, 0x02, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char X
    0x08, 0x00, 0x00, 0x06, 0x00, 0x1C, 0x00, 0x70, 0x00, 0xC0, 0x1F, 0x70, 0x00, 0x1C, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char Y
    0x09, 0x00, 0x10, 0x02, 0x1C, 0x02, 0x16, 0x82, 0x13, 0xE2, 0x10, 0x32, 0x10, 0x1E, 0x10, 0x06, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char Z
    0x04, 0x00, 0x00, 0xFE, 0xFF, 0x02, 0x80, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char [
    0x04, 0x06, 0x00, 0x78, 0x00, 0x80, 0x07, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char BackSlash
    0x03, 0x02, 0x80, 0x02, 0x80, 0xFE, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char ]
    0x07, 0x40, 0x00, 0x70, 0x00, 0x0E, 0x00, 0x02, 0x00, 0x0E, 0x00, 0x70, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char ^
    0x09, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char _
    0x04, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char `
    0x09, 0x00, 0x00, 0x20, 0x0E, 0x30, 0x13, 0x10, 0x11, 0x10, 0x11, 0x10, 0x09, 0xF0, 0x0F, 0xC0, 0x1F, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char a
    0x08, 0x00, 0x00, 0xFE, 0x1F, 0xEE, 0x0F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x30, 0x18, 0xE0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char b
    0x07, 0x00, 0x00, 0xE0, 0x0F, 0x30, 0x18, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x20, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char c
    0x08, 0x00, 0x00, 0xE0, 0x0F, 0x30, 0x18, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFE, 0x0F, 0xFE, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char d
    0x08, 0x00, 0x00, 0xE0, 0x0F, 0x30, 0x19, 0x10, 0x11, 0x10, 0x11, 0x10, 0x11, 0x30, 0x19, 0xE0, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char e
    0x04, 0x10, 0x00, 0xFC, 0x1F, 0xFE, 0x1F, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char f
    0x08, 0x00, 0x00, 0xE0, 0x4F, 0x30, 0xD8, 0x10, 0x90, 0x10, 0x90, 0x30, 0x88, 0xC0, 0x7F, 0xF0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char g
    0x07, 0x00, 0x00, 0xFE, 0x1F, 0x60, 0x00, 0x10, 0x00, 0x10, 0x00, 0x30, 0x00, 0xE0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char h
    0x03, 0x00, 0x00, 0xF2, 0x1F, 0xF2, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char i
    0x02, 0x00, 0x80, 0xF2, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char j
    0x08, 0x00, 0x00, 0xFE, 0x1F, 0x00, 0x03, 0x80, 0x01, 0xC0, 0x03, 0x60, 0x0E, 0x30, 0x18, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char k
    0x02, 0x00, 0x00, 0xFE, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char l
    0x0C, 0x00, 0x00, 0xF0, 0x1F, 0x60, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0xE0, 0x1F, 0x20, 0x00, 0x10, 0x00, 0x10, 0x00, 0x30, 0x00, 0xE0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char m
    0x07, 0x00, 0x00, 0xF0, 0x1F, 0x60, 0x00, 0x10, 0x00, 0x10, 0x00, 0x30, 0x00, 0xE0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char n
    0x08, 0x00, 0x00, 0xE0, 0x0F, 0x30, 0x18, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x30, 0x18, 0xE0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char o
    0x08, 0x00, 0x00, 0xF0, 0xFF, 0xE0, 0xEF, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x30, 0x18, 0xE0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char p
    0x08, 0x00, 0x00, 0xE0, 0x0F, 0x30, 0x18, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xE0, 0xEF, 0xF0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char q
    0x05, 0x00, 0x00, 0xF0, 0x1F, 0x60, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char r
    0x07, 0x00, 0x00, 0xE0, 0x08, 0x90, 0x11, 0x10, 0x11, 0x10, 0x11, 0x10, 0x13, 0x20, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char s
    0x04, 0x10, 0x00, 0xFC, 0x1F, 0xFC, 0x1F, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char t
    0x07, 0x00, 0x00, 0xF0, 0x0F, 0x00, 0x18, 0x00, 0x10, 0x00, 0x10, 0x00, 0x0C, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char u
    0x07, 0x30, 0x00, 0xF0, 0x01, 0x00, 0x1F, 0x00, 0x10, 0x00, 0x0F, 0xF0, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char v
    0x0B, 0x30, 0x00, 0xE0, 0x03, 0x00, 0x1E, 0x00, 0x1E, 0xE0, 0x03, 0x10, 0x00, 0xE0, 0x03, 0x00, 0x1C, 0x00, 0x1E, 0xE0, 0x03, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char w
    0x07, 0x00, 0x10, 0x30, 0x18, 0xE0, 0x06, 0x80, 0x01, 0xE0, 0x06, 0x30, 0x18, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char x
    0x07, 0x10, 0x80, 0xF0, 0x81, 0x80, 0xCF, 0x00, 0x78, 0x00, 0x0F, 0xF0, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char y
    0x06, 0x00, 0x00, 0x10, 0x1C, 0x10, 0x16, 0x90, 0x13, 0xF0, 0x10, 0x30, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char z
    0x05, 0x00, 0x01, 0x80, 0x03, 0xFE, 0xFE, 0x02, 0x80, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char {
    0x02, 0x00, 0x00, 0xFE, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char |
    0x05, 0x02, 0x80, 0x02, 0x80, 0xFE, 0xFE, 0x80, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char }
    0x08, 0x00, 0x00, 0xC0, 0x00, 0x40, 0x00, 0x40, 0x00, 0x80, 0x00, 0x00, 0x01, 0x00, 0x01, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Code for char ~
    0x03, 0xFC, 0x0F, 0x04, 0x08, 0xFC, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Code for char 
};

struct FontDef Font_Liberation_Sans_15x16 = {
    Liberation_Sans15x16_Data,
    15,
    2,
    ' ',
    '\x7F'
};

#endif
