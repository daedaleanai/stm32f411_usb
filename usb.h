#pragma once
/*
	Small USB driver for STM32F4xx, providing 1 device with 1 configuration
	with 1 interface with 1 setting, providing endpoint 0 for control and
	endpoint 0x1/0x81 with bidirectional bulk transfers, with a buffer
	size of 64 bytes in both directions. 

	Linux clients can use 

		echo 0x0483 0x5722 >/sys/bus/usb-serial/drivers/generic/new_id

	or

		sudo modprobe usbserial vendor=0x0483 product=0x5722

	The device has VID 0483 (STMicroelectronics) and PID 5722 (Bulk demo).
*/
#include <stddef.h>  // for size_t
#include <stdint.h>  // for uintX_t

// usb_init() (re-)initializes the usb device.
//
// To enable handling of the protocol messages exchanged with the usb host,
// the user must call NVIC_EnableIRQ(OTG_FS_IRQn) and provide an
// implementation of OTG_FS_Handler(void) that calls usb_recv()
// or call usb_recv() repeatedly by other means, e.g. in the main loop.
void usb_init();


// Cf. the Visible States Universal Serial Bus Specification Revision 2.0 Section 9.1.1
// In the DEFAULT state, the device address is zero and only endpoint 0 is configured.
// On a resume(wakeup) event, the SUSPENDED state returns to whatever the state was before.
enum usb_state_t { 
	USB_UNATTACHED,   // The built-in USB peripheral not yet communicating with the host.
	USB_DEFAULT,      // the host has issued a bus-reset to the peripheral.
	USB_ADDRESS,      // the host has assigned a device address to the peripheral
	USB_CONFIGURED,   // the host has issued a successful configuration request to the peripheral
	USB_SUSPENDED,    // the peripheral is in the suspended state due to bus inactivity
};

// usb_state() returns the current state of the built-in USB peripheral.
enum usb_state_t usb_state();

const char* usb_state_str(enum usb_state_t s);

// usb_recv() tries to copy up to sz bytes from the last received message from
// the USB receive buffer to buf. This function is meant to be called from
// OTG_FS_IRQ_Handler() or the main loop, as it drives the entire protocol stack. If
// more than sz bytes are available, the unread portion of the buffer is
// discarded.  To avoid this call usb_recv with sz >= 64.
size_t usb_recv(uint8_t* buf, size_t sz);

// usb_send tries to copy buf[:min(len,64)] to a free USB transmit buffer and
// schedules it for transmission. If no buffer is available usb_send returns
// zero, otherwise it returns the number of bytes that were be copied. 
size_t usb_send(const uint8_t* buf, size_t len);

