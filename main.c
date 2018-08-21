/* Modified from ....
 * Copyright (C) 2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use with
 * Infineon's microcontrollers.
 * This file can be freely distributed within development tools that are
 * supporting such microcontrollers.
 *
 * (This version is for use with the Curtis 1355 test fixture) Martin Wibert 07/13/18
 * File name main.c
*/

/*
This code is used for performing a (go-no-go) bench test for a model # 1355-5002 motor controller
using an Infineon XMC1400 Boot Kit uP board as the master CAN bus controller.

Infineon XMC BOOT KIT:
Ports P4.7 and P4.6 are inputs, if P4.7 goes HIGH, start executing code and sending CAN messages(see below)
A total of X PDO (MISO) TX messages and X SDO (MISO) TX messages are sent.

NOTE: Add 1.5 seconds of delay between the XMC 1400 power_on and sending CAN messages to the controller.
This is to ensure controller enough time to initiate during poweron reset and is ready to accept CAN messages.

;
; Columns descriptions: "For reference only"
; ~~~~~~~~~~~~~~~~~~~~~

*/


#include <xmc_common.h>
#include "xmc_can.h"
#include "xmc_gpio.h"

#define XMC_SCU_IRQCTRL_7_CAN0_SR3 ( (7U << 8U) | 1U )

#define TIME_FOR_BOOTUP_MS 1500  // add startup delay 1.5 sec to ensure UUT 1355-5002 is ready first

#define TICKS_PER_SECOND 1000
#define TICKS_WAIT_250 250      //250
#define TICKS_WAIT_32 32

#define CAN1_TXD P4_9
#define CAN1_RXD P4_8

// Define LEDs (verified 08/16/18)
#define LED P4_0      //uP on-board LED
#define LED_TX P4_1   //CAN TX indication
#define LED_RX P4_3   //CAN_RX indication
#define LED_TST P3_0  //Blue "Testing" LED
#define LED_FF P3_2   //Red "Fault/Fail" LED
#define LED_A1 P1_8   //Actuator #1 Green LED
#define LED_A2 P3_3   //Actuator #2 Green LED
#define LED_A3 P0_13  //Actuator #3 Green LED
#define LED_M1 P4_5   //Motor M1 Green LED
#define LED_M2 P4_4   //Motor M2 Green LED
#define LED_M3 P3_1   //Motor M3 Green LED
#define LED_M4 P0_12  //Motor M4 Green LED
#define LED_M5 P3_4   //Motor M5 Green LED

//Configure I/O pins 07/16/18
XMC_GPIO_CONFIG_t led_config;
XMC_GPIO_CONFIG_t led_TX_config;
XMC_GPIO_CONFIG_t led_RX_config;
XMC_GPIO_CONFIG_t Pin4_7;       //analog IN or digital IN = Start test button
XMC_GPIO_CONFIG_t Pin4_6;       //analog IN or digital IN = Serial Switch
XMC_GPIO_CONFIG_t Pin3_0;       //Blue "Testing" LED (was led_30_config;)
XMC_GPIO_CONFIG_t Pin3_2;       //Red Fault/Fail LED
XMC_GPIO_CONFIG_t Pin1_8;       //Green Actuator #1 LED
XMC_GPIO_CONFIG_t Pin3_3;       //Green Actuator #2 LED
XMC_GPIO_CONFIG_t Pin0_13;      //Green Actuator #3 LED
XMC_GPIO_CONFIG_t Pin4_5;       //Green Motor M1 LED
XMC_GPIO_CONFIG_t Pin4_4;       //Green Motor M2 LED
XMC_GPIO_CONFIG_t Pin3_1;       //Green Motor M3 LED
XMC_GPIO_CONFIG_t Pin0_12;      //Green Motor M4 LED
XMC_GPIO_CONFIG_t Pin3_4;       //Green Motor M5 LED


#define SMALL_TEXT_SIZE 6
#define LARGE_TEXT_SIZE 3
//#define P4_7_read (PORT4->IN & (1 << 7))  //was P2_8_read (PORT2->IN & (1 << 8))
//#define P4_6_read (PORT4->IN & (1 << 6))  //added 07/13/18
// #define P3_4_write (PORT3->OUT & (1 << 4)) //added 7/16/18

#define CAN_FREQUENCY 8000000

/*CAN Bit time*/
XMC_CAN_NODE_NOMINAL_BIT_TIME_CONFIG_t baud = 
{
  .can_frequency = CAN_FREQUENCY,
  .baudrate = 500000, // default to 250000
  .sample_point = 6000,
  .sjw=3,
};

//CAN Message 0 dedicated to TX CAN msgs
XMC_CAN_MO_t CAN_message0 = 
{
  .can_mo_ptr = CAN_MO0,
  .can_priority = XMC_CAN_ARBITRATION_MODE_IDE_DIR_BASED_PRIO_2,
  .can_identifier = 0x000, 
  .can_id_mask = 0x7ff,
  .can_id_mode = XMC_CAN_FRAME_TYPE_STANDARD_11BITS,
  .can_ide_mask = 1,
  .can_data_length = 2,
  .can_data = {0, 0}, 
  .can_mo_type = XMC_CAN_MO_TYPE_TRANSMSGOBJ
};


//CAN Message 2 dedicated to RX CAN msgs
XMC_CAN_MO_t CAN_message2 = 
{
  .can_mo_ptr = CAN_MO2,
  .can_priority = XMC_CAN_ARBITRATION_MODE_IDE_DIR_BASED_PRIO_2,
  .can_identifier = 0x5A0,
  .can_id_mask = 0x7ff,
  .can_id_mode = XMC_CAN_FRAME_TYPE_STANDARD_11BITS,
  .can_ide_mask = 1,
  .can_data_length = 8,
  .can_mo_type = XMC_CAN_MO_TYPE_RECMSGOBJ
};

//CAN Message 3 & 4 dedicated to 3140T gauge display
XMC_CAN_MO_t CAN_message3 = 
{
  .can_mo_ptr = CAN_MO3,
  .can_priority = XMC_CAN_ARBITRATION_MODE_IDE_DIR_BASED_PRIO_2,
  .can_identifier = 0x271,
  .can_id_mask = 0x7ff,
  .can_id_mode = XMC_CAN_FRAME_TYPE_STANDARD_11BITS,
  .can_ide_mask = 1,
  .can_data_length = 5,
  .can_data = {0, 0},
  .can_mo_type= XMC_CAN_MO_TYPE_TRANSMSGOBJ
};

XMC_CAN_MO_t CAN_message4 = 
{
  .can_mo_ptr = CAN_MO4,
  .can_priority = XMC_CAN_ARBITRATION_MODE_IDE_DIR_BASED_PRIO_2,
  .can_identifier = 0x371,
  .can_id_mask = 0x7ff,
  .can_id_mode = XMC_CAN_FRAME_TYPE_STANDARD_11BITS,
  .can_ide_mask = 1,
  .can_data_length = 6,
  .can_data = {0, 0},
  .can_mo_type = XMC_CAN_MO_TYPE_TRANSMSGOBJ
};

XMC_CAN_MO_t CAN_message1;
XMC_CAN_MO_t CAN_message5;


bool send_messages = false;
uint32_t tick_startup = 0;

uint32_t ticks = 0;
void SysTick_Handler(void)
{
  static uint32_t ticks250ms = 0; 
  static uint32_t ticks32ms = 0;
  ticks++;
  if(tick_startup++ < TIME_FOR_BOOTUP_MS)
      return;
  
  ticks250ms++;
  ticks32ms++;
  
  
  if (ticks250ms == TICKS_WAIT_250)
  {
    ticks250ms = 0;
  }  
  
  if (ticks32ms == TICKS_WAIT_32)
  {
      XMC_CAN_MO_Transmit(&CAN_message4);
      XMC_CAN_MO_Transmit(&CAN_message3);
      ticks32ms = 0;
  }
  
}

XMC_SCU_CLOCK_CONFIG_t clock_config =
{
	.fdiv = 0,  //Fractional divider
	.idiv = 1,   //MCLK = 48MHz
	.dclk_src = XMC_SCU_CLOCK_DCLKSRC_DCO1,
	.oschp_mode = XMC_SCU_CLOCK_OSCHP_MODE_DISABLED,
        .osclp_mode = XMC_SCU_CLOCK_OSCLP_MODE_OSC,
	.pclk_src = XMC_SCU_CLOCK_PCLKSRC_DOUBLE_MCLK,
        .rtc_src = XMC_SCU_CLOCK_RTCCLKSRC_DCO2,
};

uint32_t mode = 0;
void application_state_machine();

int main(void)
{
  XMC_SCU_CLOCK_Init(&clock_config);
  //led_config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  //XMC_GPIO_Init(LED, &led_config);
  
  /*Led on Board Initialization*/
  Pin4_7.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
  XMC_GPIO_Init(P4_7, &Pin4_7);
  Pin4_6.mode = XMC_GPIO_MODE_INPUT_TRISTATE; 
  XMC_GPIO_Init(P4_6, &Pin4_6);
  Pin3_0.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P3_0, &Pin3_0);
  Pin3_2.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P3_2, &Pin3_2);
  Pin3_3.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P3_3, &Pin3_3);
  Pin0_13.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P0_13, &Pin0_13);
  Pin1_8.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P1_8, &Pin1_8);
  Pin3_4.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P3_4, &Pin3_4);
  Pin0_12.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P0_12, &Pin0_12);
  Pin3_1.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P3_1, &Pin3_1);
  Pin4_5.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P4_5, &Pin4_5);
  Pin4_4.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  XMC_GPIO_Init(P4_4, &Pin4_4);
  
  
  //added 7/16/18 9:19
  XMC_GPIO_SetOutputLow(LED_TST);
  XMC_GPIO_SetOutputLow(LED_A1);
  XMC_GPIO_SetOutputLow(LED_A2);
  XMC_GPIO_SetOutputLow(LED_A3);
  XMC_GPIO_SetOutputLow(LED_M1);
  XMC_GPIO_SetOutputLow(LED_M2);
  XMC_GPIO_SetOutputLow(LED_M3);
  XMC_GPIO_SetOutputLow(LED_M4);
  XMC_GPIO_SetOutputLow(LED_M5);
  

  /* System timer configuration */
  SysTick_Config(SystemCoreClock / TICKS_PER_SECOND);
  NVIC_SetPriority(SysTick_IRQn,0);

  while(tick_startup < TIME_FOR_BOOTUP_MS); // added 1.5sec delay to wait for UUT ready
    
  XMC_GPIO_CONFIG_t CAN1_TXD_config;
  XMC_GPIO_CONFIG_t CAN1_RXD_config;
  CAN1_TXD_config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT9;
  CAN1_RXD_config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
  XMC_GPIO_Init(CAN1_TXD, &CAN1_TXD_config);
  XMC_GPIO_Init(CAN1_RXD, &CAN1_RXD_config);

  /*Configure CAN Module*/
  XMC_CAN_Init(CAN, XMC_CAN_CANCLKSRC_MCLK, CAN_FREQUENCY);

  XMC_CAN_NODE_EnableConfigurationChange(CAN_NODE1);
  XMC_CAN_NODE_SetReceiveInput(CAN_NODE1, XMC_CAN_NODE_RECEIVE_INPUT_RXDCC);
  XMC_CAN_NODE_DisableConfigurationChange(CAN_NODE1);

  /*NODE 1 initialization*/
  XMC_CAN_NODE_EnableConfigurationChange(CAN_NODE1);
  XMC_CAN_NODE_NominalBitTimeConfigure(CAN_NODE1, &baud);
  XMC_CAN_NODE_DisableConfigurationChange(CAN_NODE1);

  led_TX_config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  led_TX_config.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
  XMC_GPIO_Init(LED_TX, &led_TX_config);
  led_RX_config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  led_RX_config.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
  XMC_GPIO_Init(LED_RX, &led_RX_config);

  /*Message 4 Configuration - Transmit*/
  XMC_CAN_MO_Config(&CAN_message0);
  XMC_CAN_MO_Config(&CAN_message1);
  XMC_CAN_MO_Config(&CAN_message2);
  XMC_CAN_MO_Config(&CAN_message3);
  XMC_CAN_MO_Config(&CAN_message4);
  XMC_CAN_MO_Config(&CAN_message5); 


  /* Message 2 Configuration - Receive */
  XMC_CAN_MO_Config(&CAN_message2); 

  /* Enable receive event */
  XMC_CAN_MO_EnableEvent(&CAN_message2,XMC_CAN_MO_EVENT_RECEIVE); //08/14 was 2
  XMC_CAN_NODE_EnableEvent(CAN_NODE1, XMC_CAN_NODE_EVENT_TX_INT);

  /* Set receive interrupt Service request number*/
  XMC_SCU_SetInterruptControl(7, (XMC_SCU_IRQCTRL_t)XMC_SCU_IRQCTRL_7_CAN0_SR3);
  XMC_CAN_MO_SetEventNodePointer(&CAN_message2,XMC_CAN_MO_POINTER_EVENT_RECEIVE,3); 
  NVIC_SetPriority(IRQ7_IRQn,1);
  NVIC_EnableIRQ(IRQ7_IRQn);

  /* Allocate MO in Node List*/
  XMC_CAN_AllocateMOtoNodeList(CAN,1,0);
  XMC_CAN_AllocateMOtoNodeList(CAN,1,1);
  XMC_CAN_AllocateMOtoNodeList(CAN,1,2);
  XMC_CAN_AllocateMOtoNodeList(CAN,1,3);
  XMC_CAN_AllocateMOtoNodeList(CAN,1,4);
  XMC_CAN_AllocateMOtoNodeList(CAN,1,5);
  XMC_CAN_NODE_ResetInitBit(CAN_NODE1);

  while(1){
    application_state_machine();
  }
}

void delay_ms(uint32_t cycles)
{ 
  ticks = 0;
  while(ticks <= cycles);
}

  /*This function is the Interrupt Event Handler for the CAN Node*/
void  IRQ7_Handler(void)
{
  /* Toggle LED Pin 4.3  to indicate that the requested message is received*/
  XMC_CAN_MO_Receive(&CAN_message2); //08/14 was 2
  //send_messages = true;
  //XMC_GPIO_ToggleOutput(LED_RX);
  NVIC_ClearPendingIRQ(IRQ7_IRQn);
}

void SetGaugeSmallText(const char * msg)
{
    memcpy(&CAN_message4.can_data_byte[0],msg,SMALL_TEXT_SIZE);
    XMC_CAN_MO_UpdateData(&CAN_message4);
}

void SetGaugeLargeText(const char * msg)
{
    CAN_message3.can_data_byte[0] = 0x22;
    CAN_message3.can_data_byte[1] = 0;
    memcpy(&CAN_message3.can_data_byte[2],msg,LARGE_TEXT_SIZE);
    XMC_CAN_MO_UpdateData(&CAN_message3);
}

uint64_t flip_endian64(uint64_t d)
{
    volatile uint64_t temp = 0;
    for(int i = 0; i < 8; i++)
    {
        temp |= d & 0xFF;
        if(i != 7)
        {      
          temp <<= 8;     
          d >>= 8;
        }
    }
    return temp;
}

//helper function to send CAN message,
//Format  Example,  SendCanMessage(0x371,8,0x0011223344556677);
//This will send a CAN message on filter 371 that is 8 bytes long
//with a payload of 0x0011223344556677
void SendCanMessage(uint16_t filter, uint8_t len, uint64_t data)
{
    CAN_message0.can_data_length = len;
    if(8 - len)
      data <<= (8 - len) * 8;
    CAN_message0.can_identifier = filter;
    CAN_message0.can_data_long = flip_endian64(data);
    XMC_CAN_MO_UpdateData(&CAN_message0);
    XMC_CAN_MO_Config(&CAN_message0);
    XMC_CAN_MO_Transmit(&CAN_message0);
}

void sdo_write(int32_t object, uint32_t data)
{
    volatile uint64_t packet = data << 8;
    packet |= object & 0xFF;
    packet <<= 8;
    packet |= (object & 0xFF00) >> 0x8;
    packet <<= 8;
    packet |= (object & 0xFF0000) >> 0x10;
    packet <<= 8;
    packet |= 0x22;
    packet = flip_endian64(packet);
    SendCanMessage(0x620,8,packet);
}

uint32_t sdo_read(int32_t object)
{
    volatile uint64_t packet = 0;
    uint32_t data_callback = 0;
    packet |= object & 0xFF;
    packet <<= 8;
    packet |= (object & 0xFF00) >> 0x8;
    packet <<= 8;
    packet |= (object & 0xFF0000) >> 0x10;
    packet <<= 8;
    packet |= 0x42;
    packet = flip_endian64(packet);
    SendCanMessage(0x620,8,packet);
    delay_ms(2);
    data_callback |= CAN_message2.can_data_byte[7];
    data_callback <<= 8;
    data_callback |= CAN_message2.can_data_byte[6];
    data_callback <<= 8;
    data_callback |= CAN_message2.can_data_byte[5];
    data_callback <<= 8;
    data_callback |= CAN_message2.can_data_byte[4];
    return data_callback;
}

enum state_t
{
  STARTUP,
  SEND_GO_OPERATIONAL,
  CLEAR_GAUGES,
  SDO_SEND,
  FINISHED,
};


int state = STARTUP;
void application_state_machine()
{
    switch(state)
    {
      case STARTUP:
        state = SEND_GO_OPERATIONAL;
        break;
      case SEND_GO_OPERATIONAL:
        SendCanMessage(0,2,0x8000);
        state = SDO_SEND;
        break;
      case SDO_SEND:
        
        SetGaugeSmallText("      ");
        SetGaugeLargeText("   ");
     
        sdo_write(0x1F3100,0x10); //set M2 RUN ON
        state = CLEAR_GAUGES;
        break;
      case CLEAR_GAUGES:
        delay_ms(100);
        int vendor_id = sdo_read(0x181001);
        if(vendor_id == 0x154)
        {
            //do something
        }
        
        break;
      case FINISHED:
        break;
      default:
        break;
      
    }
}
