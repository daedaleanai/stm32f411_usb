/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../STM32F411.h"

#include <string.h>
#include "usbd.h"
#include "usb_private.h"

#include "../serial.h"

/* Receive FIFO size in 32-bit words. */
enum { RX_FIFO_SIZE = 128 };

struct Endpoint_Type {
	volatile uint32_t CTL;         // @256 OTG_FS device control IN endpoint 0 control register (OTG_FS_DIEPCTL0)
	uint8_t       RESERVED9[4];    // @260
	volatile uint8_t  INT;         // @264 device endpoint-x interrupt register
	uint8_t       RESERVED10[7];   // @265
	volatile uint32_t TSIZ;        // @272 device endpoint-0 transfer size register
	uint8_t       RESERVED11[4];   // @276
	volatile uint16_t  DTXFSTS;    // @280 OTG_FS device IN endpoint transmit FIFO status register
	uint8_t       RESERVED12[6];   // @282
};

static inline struct Endpoint_Type* DIEP(int idx) { return ((struct Endpoint_Type*)(&OTG_FS_DEVICE.DIEPCTL0)) + idx; }
static inline struct Endpoint_Type* DOEP(int idx) { return ((struct Endpoint_Type*)(&OTG_FS_DEVICE.DOEPCTL0)) + idx; }

extern struct Fifo_Type {
	volatile uint32_t data[1024]; // 4KB 
} OTG_FS_FIFO[5];


static void dwc_set_address(usbd_device *usbd_dev, uint8_t addr)
{
	(void)usbd_dev;
	otg_fs_device_dcfg_set_dad(&OTG_FS_DEVICE, addr);
}

static void dwc_ep_setup(usbd_device *usbd_dev, uint8_t addr, uint8_t type,	uint16_t max_size,
			void (*callback) (usbd_device *usbd_dev, uint8_t ep))
{
	(void)usbd_dev;
	/*
	 * Configure endpoint address and type. Allocate FIFO memory for
	 * endpoint. Install callback function.
	 */
	uint8_t dir = addr & 0x80;
	addr &= 0x7f;

	if (addr == 0) { /* For the default control endpoint */
		/* Configure IN part. */
		if (max_size >= 64) {
			otg_fs_device_diepctl0_set_mpsiz(&OTG_FS_DEVICE, 0);
		} else if (max_size >= 32) {
			otg_fs_device_diepctl0_set_mpsiz(&OTG_FS_DEVICE,1);
		} else if (max_size >= 16) {
			otg_fs_device_diepctl0_set_mpsiz(&OTG_FS_DEVICE,2);
		} else {
			otg_fs_device_diepctl0_set_mpsiz(&OTG_FS_DEVICE,3);
		}

		OTG_FS_DEVICE.DIEPTSIZ0 = 0;
		otg_fs_device_dieptsiz0_set_xfrsiz(&OTG_FS_DEVICE, max_size);
		OTG_FS_DEVICE.DIEPCTL0 |= OTG_FS_DEVICE_DIEPCTL0_EPENA | OTG_FS_DEVICE_DIEPCTL0_SNAK;

		/* Configure OUT part. */
		OTG_FS_DEVICE.DOEPTSIZ0 |= OTG_FS_DEVICE_DIEPTSIZ0_PKTCNT;
		otg_fs_device_doeptsiz0_set_stupcnt(&OTG_FS_DEVICE, 1);
		otg_fs_device_doeptsiz0_set_xfrsiz(&OTG_FS_DEVICE, max_size);
		usbd_dev->doeptsiz[0] = OTG_FS_DEVICE.DOEPTSIZ0;
		OTG_FS_DEVICE.DOEPCTL0 |= OTG_FS_DEVICE_DOEPCTL0_EPENA | OTG_FS_DEVICE_DOEPCTL0_SNAK;

		otg_fs_global_gnptxfsiz_device_set_tx0fd(&OTG_FS_GLOBAL, max_size / 4);
		otg_fs_global_gnptxfsiz_device_set_tx0fsa(&OTG_FS_GLOBAL,usbd_dev->driver->rx_fifo_size);
		usbd_dev->fifo_mem_top += max_size / 4;
		usbd_dev->fifo_mem_top_ep0 = usbd_dev->fifo_mem_top;

		return;
	}

	if (dir) {
		(&OTG_FS_GLOBAL.DIEPTXF1)[addr-1] = ((max_size / 4) << 16) | usbd_dev->fifo_mem_top;
		usbd_dev->fifo_mem_top += max_size / 4;

		DIEP(addr)->TSIZ = (max_size & OTG_FS_DEVICE_DIEPTSIZ0_XFRSIZ);
		DIEP(addr)->CTL |=  OTG_FS_DEVICE_DIEPCTL0_EPENA | OTG_FS_DEVICE_DIEPCTL0_SNAK | (type << 18)| OTG_FS_DEVICE_DIEPCTL0_USBAEP | OTG_FS_DEVICE_DIEPCTL1_SD0PID_SEVNFRM | (addr << 22) | max_size;

		if (callback) {
			usbd_dev->user_callback_ctr[addr][USB_TRANSACTION_IN] = (void *)callback;
		}
	} else {
		usbd_dev->doeptsiz[addr] = OTG_FS_DEVICE_DOEPTSIZ0_PKTCNT | (max_size & OTG_FS_DEVICE_DOEPTSIZ0_XFRSIZ);
		DOEP(addr)->TSIZ = usbd_dev->doeptsiz[addr];
		DOEP(addr)->CTL |= OTG_FS_DEVICE_DOEPCTL0_EPENA | OTG_FS_DEVICE_DOEPCTL0_USBAEP | OTG_FS_DEVICE_DOEPCTL0_CNAK | OTG_FS_DEVICE_DOEPCTL1_SD0PID_SEVNFRM | (type << 18) | max_size;

		if (callback) {
			usbd_dev->user_callback_ctr[addr][USB_TRANSACTION_OUT] = (void *)callback;
		}
	}
}

static void dwc_endpoints_reset(usbd_device *usbd_dev)
{
	int i;
	/* The core resets the endpoints automatically on reset. */
	usbd_dev->fifo_mem_top = usbd_dev->fifo_mem_top_ep0;

	/* Disable any currently active endpoints */
	for (i = 1; i < 4; i++) {
		if (DOEP(i)->CTL & OTG_FS_DEVICE_DOEPCTL0_EPENA) {
			DOEP(i)->CTL |= OTG_FS_DEVICE_DOEPCTL0_EPDIS;
		}
		if (DIEP(i)->CTL & OTG_FS_DEVICE_DIEPCTL0_EPENA) {
			DIEP(i)->CTL |= OTG_FS_DEVICE_DIEPCTL0_EPDIS;
		}
	}

	/* Flush all tx/rx fifos */
	OTG_FS_GLOBAL.GRSTCTL = OTG_FS_GLOBAL_GRSTCTL_TXFFLSH | (1<<10) | OTG_FS_GLOBAL_GRSTCTL_RXFFLSH;
}

static void dwc_ep_stall_set(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)
{
	(void)usbd_dev;
	if (addr == 0) {
		if (stall) {
			DIEP(addr)->CTL |= OTG_FS_DEVICE_DIEPCTL0_STALL;
		} else {
			DIEP(addr)->CTL &= ~OTG_FS_DEVICE_DIEPCTL0_STALL;
		}
	}

	if (addr & 0x80) {
		addr &= 0x7F;

		if (stall) {
			DIEP(addr)->CTL |= OTG_FS_DEVICE_DIEPCTL0_STALL;
		} else {
			DIEP(addr)->CTL &= ~OTG_FS_DEVICE_DIEPCTL0_STALL;
			DIEP(addr)->CTL |= OTG_FS_DEVICE_DIEPCTL1_SD0PID_SEVNFRM;
		}
	} else {
		if (stall) {
			DOEP(addr)->CTL |= OTG_FS_DEVICE_DOEPCTL0_STALL;
		} else {
			DOEP(addr)->CTL &= ~OTG_FS_DEVICE_DOEPCTL0_STALL;
			DOEP(addr)->CTL |= OTG_FS_DEVICE_DOEPCTL1_SD0PID_SEVNFRM;
		}
	}
}

static uint8_t dwc_ep_stall_get(usbd_device *usbd_dev, uint8_t addr)
{
	(void)usbd_dev;
	/* Return non-zero if STALL set. */
	if (addr & 0x80) {
		return (DIEP(addr&0x7f)->CTL & OTG_FS_DEVICE_DIEPCTL0_STALL) ? 1 : 0;
	} else {
		return (DOEP(addr)->CTL & OTG_FS_DEVICE_DOEPCTL0_STALL) ? 1 : 0;
	}
}

static void dwc_ep_nak_set(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)
{
	/* It does not make sense to force NAK on IN endpoints. */
	if (addr & 0x80) {
		return;
	}

	usbd_dev->force_nak[addr] = nak;

	DOEP(addr)->CTL |= nak ? OTG_FS_DEVICE_DOEPCTL1_SNAK : OTG_FS_DEVICE_DOEPCTL1_CNAK;
}

static uint16_t dwc_ep_write_packet(usbd_device *usbd_dev, uint8_t addr,
			      const void *buf, uint16_t len)
{
	(void)usbd_dev;

	const uint32_t *buf32 = buf;
#if defined(__ARM_ARCH_6M__)
	const uint8_t *buf8 = buf;
	uint32_t word32;
#endif /* defined(__ARM_ARCH_6M__) */
	int i;

	addr &= 0x7F;

	/* Return if endpoint is already enabled. */
	if (DIEP(addr)->TSIZ & OTG_FS_DEVICE_DOEPTSIZ0_PKTCNT) {
		return 0;
	}

	/* Enable endpoint for transmission. */
	DIEP(addr)->TSIZ = OTG_FS_DEVICE_DOEPTSIZ0_PKTCNT | len;
	DIEP(addr)->CTL |= OTG_FS_DEVICE_DIEPCTL0_EPENA | OTG_FS_DEVICE_DIEPCTL0_CNAK;

	/* Copy buffer to endpoint FIFO, note - memcpy does not work.
	 * ARMv7M supports non-word-aligned accesses, ARMv6M does not. */
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
	for (i = len; i > 0; i -= 4) {
		OTG_FS_FIFO[addr].data[0] = *buf32++;
	}
#endif /* defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) */

#if defined(__ARM_ARCH_6M__)
	/* Take care of word-aligned and non-word-aligned buffers */
	if (((uint32_t)buf8 & 0x3) == 0) {
		for (i = len; i > 0; i -= 4) {
			OTG_FS_FIFO[addr].data[0] = *buf32++;
		}
	} else {
		for (i = len; i > 0; i -= 4) {
			memcpy(&word32, buf8, 4);
			OTG_FS_FIFO[addr].data[0] = word32;
			buf8 += 4;
		}
	}
#endif /* defined(__ARM_ARCH_6M__) */

	return len;
}

static uint16_t dwc_ep_read_packet(usbd_device *usbd_dev, uint8_t addr, void *buf, uint16_t len)
{
	int i;
	uint32_t *buf32 = buf;
#if defined(__ARM_ARCH_6M__)
	uint8_t *buf8 = buf;
	uint32_t word32;
#endif /* defined(__ARM_ARCH_6M__) */
	uint32_t extra;

	/* We do not need to know the endpoint address since there is only one
	 * receive FIFO for all endpoints.
	 */
	(void) addr;
	len = MIN(len, usbd_dev->rxbcnt);

	/* ARMv7M supports non-word-aligned accesses, ARMv6M does not. */
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
	for (i = len; i >= 4; i -= 4) {
		*buf32++ = OTG_FS_FIFO[0].data[0];
		usbd_dev->rxbcnt -= 4;
	}
#endif /* defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) */

#if defined(__ARM_ARCH_6M__)
	/* Take care of word-aligned and non-word-aligned buffers */
	if (((uint32_t)buf8 & 0x3) == 0) {
		for (i = len; i >= 4; i -= 4) {
			*buf32++ = OTG_FS_FIFO[0].data[0];
			usbd_dev->rxbcnt -= 4;
		}
	} else {
		for (i = len; i >= 4; i -= 4) {
			word32 = OTG_FS_FIFO[0].data[0];
			memcpy(buf8, &word32, 4);
			usbd_dev->rxbcnt -= 4;
			buf8 += 4;
		}
		/* buf32 needs to be updated as it is used for extra */
		buf32 = (uint32_t *)buf8;
	}
#endif /* defined(__ARM_ARCH_6M__) */

	if (i) {
		extra = OTG_FS_FIFO[0].data[0];
		/* we read 4 bytes from the fifo, so update rxbcnt */
		if (usbd_dev->rxbcnt < 4) {
			/* Be careful not to underflow (rxbcnt is unsigned) */
			usbd_dev->rxbcnt = 0;
		} else {
			usbd_dev->rxbcnt -= 4;
		}
		memcpy(buf32, &extra, i);
	}

	return len;
}

static void dwc_flush_txfifo(usbd_device *usbd_dev, int ep)
{
	(void)usbd_dev;
	uint32_t fifo;
	/* set IN endpoint NAK */
	DIEP(ep)->CTL |= OTG_FS_DEVICE_DIEPCTL0_SNAK;
	/* wait for core to respond */
	while (!(DIEP(ep)->INT & OTG_FS_DEVICE_DIEPINT0_INEPNE)) {
		/* idle */
	}
	/* get fifo for this endpoint */
	fifo = (DIEP(ep)->CTL  >> 22) & 0xf;
	/* wait for core to idle */
	while (!(OTG_FS_GLOBAL.GRSTCTL & OTG_FS_GLOBAL_GRSTCTL_AHBIDL)){}
	/* flush tx fifo */
	OTG_FS_GLOBAL.GRSTCTL  = (fifo << 6) | OTG_FS_GLOBAL_GRSTCTL_TXFFLSH;
	/* reset packet counter */
	DIEP(ep)->TSIZ = 0;
	while (OTG_FS_GLOBAL.GRSTCTL  & OTG_FS_GLOBAL_GRSTCTL_TXFFLSH) {}
}

static void dwc_poll(usbd_device *usbd_dev)
{
	/* Read interrupt status register. */
	uint32_t intsts = OTG_FS_GLOBAL.GINTSTS;
	int i;

	if (intsts & OTG_FS_GLOBAL_GINTSTS_ENUMDNE) {
		/* Handle USB RESET condition. */
		serial_printf(&USART2, "usb irq reset\n");
		OTG_FS_GLOBAL.GINTSTS = OTG_FS_GLOBAL_GINTSTS_ENUMDNE;
		usbd_dev->fifo_mem_top = usbd_dev->driver->rx_fifo_size;
		_usbd_reset(usbd_dev);
		return;
	}

	/*
	 * There is no global interrupt flag for transmit complete.
	 * The XFRC bit must be checked in each OTG_DIEPINT(x).
	 */
	for (i = 0; i < 4; i++) { /* Iterate over endpoints. */
		if (DIEP(i)->INT & OTG_FS_DEVICE_DIEPINT0_XFRC) {
			/* Transfer complete. */
			if (usbd_dev->user_callback_ctr[i][USB_TRANSACTION_IN]) {
				usbd_dev->user_callback_ctr[i][USB_TRANSACTION_IN](usbd_dev, i);
			}
			DIEP(i)->INT = OTG_FS_DEVICE_DIEPINT0_XFRC;
		}
	}

	/* Note: RX and TX handled differently in this device. */
	if (intsts & OTG_FS_GLOBAL_GINTSTS_RXFLVL) {
		/* Receive FIFO non-empty. */
		uint32_t grxstsr = OTG_FS_GLOBAL.GRXSTSP_Device;
		uint32_t pktsts =  (grxstsr & OTG_FS_GLOBAL_GRXSTSR_DEVICE_PKTSTS) >> 17;
		uint32_t bcnt   =  (grxstsr & OTG_FS_GLOBAL_GRXSTSR_DEVICE_BCNT) >> 4;
		uint8_t ep      =  (grxstsr & OTG_FS_GLOBAL_GRXSTSR_DEVICE_EPNUM) >> 0;

		serial_printf(&USART2, "rx: %ld %d %ld\n", pktsts, ep, bcnt);

		if (pktsts == 4 /*OTG_GRXSTSP_PKTSTS_SETUP_COMP*/) {
			usbd_dev->user_callback_ctr[ep][USB_TRANSACTION_SETUP] (usbd_dev, ep);
		}

		if (pktsts == 3 /*OTG_GRXSTSP_PKTSTS_OUT_COMP*/ || pktsts == 4 /*OTG_GRXSTSP_PKTSTS_SETUP_COMP*/)  {
			DOEP(ep)->TSIZ = usbd_dev->doeptsiz[ep];
			DOEP(ep)->CTL |= OTG_FS_DEVICE_DOEPCTL0_EPENA |	(usbd_dev->force_nak[ep] ? OTG_FS_DEVICE_DOEPCTL1_SNAK : OTG_FS_DEVICE_DOEPCTL1_CNAK); 
			return;
		}

		if ((pktsts != 2 /*OTG_GRXSTSP_PKTSTS_OUT*/) && (pktsts != 6 /*OTG_GRXSTSP_PKTSTS_SETUP*/)) {
			return;
		}

		uint8_t type;
		if (pktsts == 6 /*OTG_GRXSTSP_PKTSTS_SETUP*/) {
			type = USB_TRANSACTION_SETUP;
		} else {
			type = USB_TRANSACTION_OUT;
		}

		if (type == USB_TRANSACTION_SETUP && (DIEP(ep)->TSIZ & OTG_FS_DEVICE_DOEPTSIZ0_PKTCNT)) {
			/* SETUP received but there is still something stuck in the transmit fifo.  Flush it. */
			dwc_flush_txfifo(usbd_dev, ep);
		}

		/* Save packet size for dwc_ep_read_packet(). */
		usbd_dev->rxbcnt = bcnt;

		if (type == USB_TRANSACTION_SETUP) {
			dwc_ep_read_packet(usbd_dev, ep, &usbd_dev->control_state.req, 8);
		} else if (usbd_dev->user_callback_ctr[ep][type]) {
			usbd_dev->user_callback_ctr[ep][type] (usbd_dev, ep);
		}

		/* Discard unread packet data. */
		for (i = 0; i < usbd_dev->rxbcnt; i += 4) {
			/* There is only one receive FIFO, so use OTG_FIFO(0) */
			(void)OTG_FS_FIFO[0].data[0];
		}

		usbd_dev->rxbcnt = 0;
	}

	if (intsts & OTG_FS_GLOBAL_GINTSTS_USBSUSP) {
		if (usbd_dev->user_callback_suspend) {
			usbd_dev->user_callback_suspend();
		}
		OTG_FS_GLOBAL.GINTSTS = OTG_FS_GLOBAL_GINTSTS_USBSUSP;
	}

	if (intsts & OTG_FS_GLOBAL_GINTSTS_WKUPINT) {
		serial_printf(&USART2, "usb irq wkup\n");
		if (usbd_dev->user_callback_resume) {
			usbd_dev->user_callback_resume();
		}
		OTG_FS_GLOBAL.GINTSTS = OTG_FS_GLOBAL_GINTSTS_WKUPINT;
	}

	if (intsts & OTG_FS_GLOBAL_GINTSTS_SOF) {
		serial_printf(&USART2, "usb irq SOF\n");
		if (usbd_dev->user_callback_sof) {
			usbd_dev->user_callback_sof();
		}
		OTG_FS_GLOBAL.GINTSTS = OTG_FS_GLOBAL_GINTSTS_SOF;
	}

	if (usbd_dev->user_callback_sof) {
		OTG_FS_GLOBAL.GINTMSK |= OTG_FS_GLOBAL_GINTMSK_SOFM;
	} else {
		OTG_FS_GLOBAL.GINTMSK &= ~OTG_FS_GLOBAL_GINTMSK_SOFM;
	}
}

static void dwc_disconnect(usbd_device *usbd_dev, int disconnected)
{
	(void)usbd_dev;
	if (disconnected) {
		OTG_FS_DEVICE.DCTL |= OTG_FS_DEVICE_DCTL_SDIS;
	} else {
		OTG_FS_DEVICE.DCTL &= ~OTG_FS_DEVICE_DCTL_SDIS;
	}
}

static struct _usbd_device usbd_dev;

/** Initialize the USB device controller hardware of the STM32. */
static usbd_device *stm32f107_usbd_init(void)
{
	OTG_FS_GLOBAL.GUSBCFG |= OTG_FS_GLOBAL_GUSBCFG_PHYSEL;

	/* Wait for AHB idle. */
	while (!(OTG_FS_GLOBAL.GRSTCTL & OTG_FS_GLOBAL_GRSTCTL_AHBIDL));
	/* Do core soft reset. */
	OTG_FS_GLOBAL.GRSTCTL |= OTG_FS_GLOBAL_GRSTCTL_CSRST;
	while (OTG_FS_GLOBAL.GRSTCTL & OTG_FS_GLOBAL_GRSTCTL_CSRST){}

#if 0 // version >= 0x2000
	/* Enable VBUS detection in device mode and power up the PHY. */
	OTG_FS_GLOBAL.GCCFG |= OTG_FS_GLOBAL_GCCFG_VBDEN | OTG_FS_GLOBAL_GCCFG_PWRDWN;
#else
//	OTG_FS_GLOBAL.GCCFG |= OTG_FS_GLOBAL_GCCFG_VBUSBSEN | OTG_FS_GLOBAL_GCCFG_PWRDWN;
	OTG_FS_GLOBAL.GCCFG |= OTG_FS_GLOBAL_GCCFG_PWRDWN;
#endif


	/* Explicitly enable DP pullup (not all cores do this by default) */
	OTG_FS_DEVICE.DCTL &= ~OTG_FS_DEVICE_DCTL_SDIS;

	/* Force peripheral only mode. */
	OTG_FS_GLOBAL.GUSBCFG |= OTG_FS_GLOBAL_GUSBCFG_FDMOD | OTG_FS_GLOBAL_GUSBCFG_TRDT;

	OTG_FS_GLOBAL.GINTSTS = OTG_FS_GLOBAL_GINTSTS_MMIS;

	/* Full speed device. */
	OTG_FS_DEVICE.DCFG |= OTG_FS_DEVICE_DCFG_DSPD;

	/* Restart the PHY clock. */
	OTG_FS_PWRCLK.PCGCCTL = 0;

	OTG_FS_GLOBAL.GRXFSIZ = RX_FIFO_SIZE;
	usbd_dev.fifo_mem_top = RX_FIFO_SIZE;

	/* Unmask interrupts for TX and RX. */
	OTG_FS_GLOBAL.GAHBCFG |= OTG_FS_GLOBAL_GAHBCFG_GINT;
	OTG_FS_GLOBAL.GINTMSK = OTG_FS_GLOBAL_GINTMSK_ENUMDNEM | OTG_FS_GLOBAL_GINTMSK_RXFLVLM | OTG_FS_GLOBAL_GINTMSK_IEPINT | OTG_FS_GLOBAL_GINTMSK_USBSUSPM | OTG_FS_GLOBAL_GINTMSK_WUIM;
	OTG_FS_DEVICE.DAINTMSK = 0xF;
	OTG_FS_DEVICE.DIEPMSK = OTG_FS_DEVICE_DIEPMSK_XFRCM;

	return &usbd_dev;
}

const struct _usbd_driver stm32f107_usb_driver = {
	.init 			= stm32f107_usbd_init,
	.set_address 	= dwc_set_address,
	.ep_setup 		= dwc_ep_setup,
	.ep_reset 		= dwc_endpoints_reset,
	.ep_stall_set 	= dwc_ep_stall_set,
	.ep_stall_get 	= dwc_ep_stall_get,
	.ep_nak_set 	= dwc_ep_nak_set,
	.ep_write_packet = dwc_ep_write_packet,
	.ep_read_packet = dwc_ep_read_packet,
	.poll 			= dwc_poll,
	.disconnect 	= dwc_disconnect,
	.set_address_before_status = 1,
	.rx_fifo_size 	= RX_FIFO_SIZE,
};

