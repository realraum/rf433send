/*
 *  rf433send
 *
 *
 *  Copyright (C) 2013-2014 Christian Pointner <equinox@spreadspace.org>
 *                          Othmar Gsenger <otti@gsenger.com>
 *
 *  This file is part of rf433send.
 *
 *  rf433send is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  rf433send is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rf433send. If not, see <http://www.gnu.org/licenses/>.
 */


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <stdio.h>

#include "util.h"
#include "led.h"
#include "usbio.h"

/*
  this program listens on a usb serial/acm interface. When the send command ('s')
  is received the send buffer gets filled with the next 3 bytes. These bites are
  sent out serialized to a specified port using PWM. Before the data is sent, a
  sync bit is sent. The data is repeated multiple times.

  This version of the program is optimized for 433MHz radio controlled power plugs.
  To control them you need to add a ASK modulator to the output pin (default F0).
*/

// RF433 Sender is Connected to PF0
#define SEND_PORT PORTF
#define SEND_DDR DDRF
#define SEND_SERIAL 0
// Logical Zero (Sender turned off) is when output is low
#define SEND_ZERO 1
// rf433 command length is 3 bytes
#define SEND_CMD_LENGTH 3
// pwm is SEND_PWM_BASE timers high and SEND_PWN_LONG_MULT * SEND_PWM_BASE timers low for a low bit (low bit = low is longer)
#define SEND_PWM_BASE 1
#define SEND_PWN_LONG_MULT 3
#define SEND_PWN_SYNC_MULT 31

//bits are read LSB from first byte first
#define SEND_CURRENT_BIT (send_cmd_buffer[send_cmd_buffer_bit_pos/8] & (1<< (send_cmd_buffer_bit_pos % 8 )))

static char send_cmd_buffer[SEND_CMD_LENGTH];
static int8_t send_cmd_buffer_bit_pos=0;
static uint8_t send_pwm_remain=0;
static uint8_t send_pwm_output=0;
static uint8_t send_repeat=0;
static uint8_t send_sync=0;



static inline void sender_on(void)
{
  if(SEND_ZERO)
    SEND_PORT |= (1 << SEND_SERIAL);
  else
    SEND_PORT &= ~(1 << SEND_SERIAL);
  send_pwm_output=1;
}

static inline void sender_off(void)
{
  if(SEND_ZERO)
    SEND_PORT &= ~(1 << SEND_SERIAL);
  else
    SEND_PORT |= (1 << SEND_SERIAL);
  send_pwm_output=0;
}

inline void sender_toggle(void)
{
  SEND_PORT ^= (1 << SEND_SERIAL);
}

void sender_timer_enable(void)
{
    TCCR0A = 1<<WGM01;
    OCR0A = 22;        // (1+17)*16us = 352us
    TCCR0B = 1<<CS02 | 0<<CS01| 0<<CS00;   //Teiler 256
    TCNT0 = 0;
    TIMSK0 |= (1<<OCIE0A);
}

inline void sender_timer_disable(void)
{
    TCCR0B =0;
    TIMSK0 &= ~(1<<OCIE0A);
}

inline uint8_t sender_timer_status(void)
{
  return (TIMSK0 & (1<<OCIE0A))>>OCIE0A;
}

ISR(TIMER0_COMPA_vect)
{
  PORTF^=2; //debug;
  if(send_cmd_buffer_bit_pos < SEND_CMD_LENGTH * 8) {
    if(send_pwm_remain)
    {
      //NOTHING
    } else if (send_sync) {
      sender_off();
      send_pwm_remain=SEND_PWM_BASE*SEND_PWN_SYNC_MULT;
      send_sync=0;
      send_cmd_buffer_bit_pos=-1;
    } else if (send_pwm_output) {
      sender_off();
      if(SEND_CURRENT_BIT)
      {
        send_pwm_remain=SEND_PWM_BASE;
      } else {
        send_pwm_remain=SEND_PWM_BASE*SEND_PWN_LONG_MULT;
      }
    } else {
      send_cmd_buffer_bit_pos++;
      sender_on();
      if(SEND_CURRENT_BIT)
      {
        send_pwm_remain=SEND_PWM_BASE*SEND_PWN_LONG_MULT;
      } else {
        send_pwm_remain=SEND_PWM_BASE;
      }
    }
    send_pwm_remain--;
  } else {
    send_cmd_buffer_bit_pos=0;
    send_pwm_remain=0;
    if(send_repeat)
    {
      send_repeat--;
      send_sync=1;
      send_pwm_remain=SEND_PWM_BASE;
      sender_on();
    } else {
      sender_timer_disable();
      sender_off();
    }
  }
}

void pins_init(void)
{
    sender_off();
    SEND_DDR |= 1 << SEND_SERIAL;
    DDRF |= 2; //DEBUG
}

void timer_init(void)
{
}

static uint8_t command_pos=0;

void handle_cmd(uint8_t cmd)
{
  if (!command_pos)
  {
    switch(cmd) {
    case '0': led_off(); break;
    case '1': led_on(); break;
    case 't': led_toggle(); break;
    case 'r': reset2bootloader(); break;
    case 's':
      printf("Expecting multibyte command now\n\r");
      command_pos=1;
      break;
    default: printf("error\n\r"); return;
    }
    printf("ok\n\r");
  } else {
    send_cmd_buffer[command_pos-1]=cmd;
    if (command_pos==SEND_CMD_LENGTH)
    {
      printf("Enabling Timer\n\r");
      command_pos=0;
      send_sync=1;
      send_pwm_remain=SEND_PWM_BASE;
      send_repeat=15;
      sender_on();
      sender_timer_enable();
    }
    else
      command_pos++;
  }
}



int main(void)
{
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  cpu_init();
  led_init();
  usbio_init();
  pins_init();
  timer_init();
  sei();

  for(;;) {
    int16_t BytesReceived = usbio_bytes_received();
    while(BytesReceived > 0) {
      int16_t ReceivedByte = fgetc(stdin);
      if(ReceivedByte != EOF) {
        handle_cmd(ReceivedByte);
      }
      BytesReceived--;
    }

    usbio_task();
  }
}
