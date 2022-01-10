//
// This is an implementation of the usb api defined in usb.h.
// It uses the Synopsis OTG_FS found in stm32f4xx and others.
// (An implementation for the stm32f103 with the same api is in usb.c)
//
//
#include "usb.h"

#include "stm32f411.h"

static enum usb_state_t _usb_state       = USB_UNATTACHED;
//static enum usb_state_t _usb_state_saved = USB_UNATTACHED; // when state is suspended, the state to return to on wakeup

enum usb_state_t usb_state() { return _usb_state; }

static const char* _sstr[] = {
    "UNATTACHED", "DEFAULT", "ADDRESS", "CONFIGURED", "SUSPENDED",
};

const char* usb_state_str(enum usb_state_t s) {
    size_t ss = s;
    if (ss < sizeof _sstr)
        return _sstr[ss];
    return "UNDEFINED";
}

struct Endpoint_Type {
    volatile uint32_t CTL;        
    const    uint32_t reserved9;  
    volatile uint32_t INT;        
    const    uint32_t reserved10; 
    volatile uint32_t TSIZ;       
    const    uint32_t reserved11; 
    volatile uint32_t DTXFSTS;    
    const    uint32_t reserved12; 
};

static inline struct Endpoint_Type* DIEP(uint8_t ep) { return ((struct Endpoint_Type*)(&OTG_FS_DEVICE.DIEPCTL0)) +(ep & 0x7f); }
static inline struct Endpoint_Type* DOEP(uint8_t ep) { return ((struct Endpoint_Type*)(&OTG_FS_DEVICE.DOEPCTL0)) + ep; }

static inline uint16_t read_le16(const uint8_t* src) { return ((uint16_t)(src[0])) | (((uint16_t)(src[1])) << 8); }
static inline uint32_t read_le32(const uint8_t* src) { return ((uint32_t)(src[0])) | (((uint32_t)(src[1])) << 8) | (((uint32_t)(src[2])) << 16)| (((uint32_t)(src[3])) << 24); }

extern struct Fifo_Type {
    volatile uint32_t data;
    const    uint32_t reserved[1023];
} OTG_FS_FIFO[5];


// read a packet from the rx fifo and store up to sz bytes in buf
static size_t read_packet(size_t len, uint8_t* buf, size_t sz) {

    size_t wordcount = (len + 3)/4; // we must pop this many words
    if (sz < len)
        len = sz; // but we must not write more than this many bytes

    size_t i = 0;
    while(i+3 < len) {
        uint32_t word = OTG_FS_FIFO[0].data;
        --wordcount;
        buf[i++] = word;
        buf[i++] = word >> 8;
        buf[i++] = word >> 16;
        buf[i++] = word >> 24;
    }

    if (wordcount) {
        uint32_t word = OTG_FS_FIFO[0].data;
        --wordcount;

        for (; i < len; i++) {
            buf[i] = word;
            word >>= 8;
        }
    }

    while(wordcount) {
        uint32_t word = OTG_FS_FIFO[0].data;
        --wordcount;
        (void)word;
    }

    return len;
}

// copy buf[:sz] to the tx packet buffer of endpoint ep
static size_t write_packet(uint8_t ep, const uint8_t* buf, size_t len) {

    if (DIEP(ep)->TSIZ & (1<<19)) {
        return 0;
    }

    DIEP(ep)->TSIZ = (1<<19) | len;  // 1 packet, of len bytes
    DIEP(ep)->CTL |= OTG_FS_DEVICE_DIEPCTL0_EPENA | OTG_FS_DEVICE_DIEPCTL0_CNAK;

    size_t wc = (len + 3)/4; // we must push this many words
    for (size_t i = 0; i < wc; ++i)
        OTG_FS_FIFO[ep].data = read_le32(buf+4*i); // ok if we read over boundary

    return len;
}

size_t usb_send(const uint8_t* buf, size_t len) { return write_packet(1, buf, len); }

void usb_init() {

    _usb_state = USB_UNATTACHED;

    // Wait for AHB idle.
    while (!(OTG_FS_GLOBAL.GRSTCTL & OTG_FS_GLOBAL_GRSTCTL_AHBIDL))
        __NOP();

     // core soft reset
    OTG_FS_GLOBAL.GRSTCTL |= OTG_FS_GLOBAL_GRSTCTL_CSRST;
    while (OTG_FS_GLOBAL.GRSTCTL & OTG_FS_GLOBAL_GRSTCTL_CSRST)
        __NOP();

    OTG_FS_GLOBAL.GINTSTS = OTG_FS_GLOBAL_GINTSTS_MMIS;  // Clear mode-mismatch interrupt, 

    // Set up for hardwired full speed device, no fancy stuff. 
    OTG_FS_GLOBAL.GCCFG |= OTG_FS_GLOBAL_GCCFG_NOVBUSSENS | OTG_FS_GLOBAL_GCCFG_PWRDWN;
    OTG_FS_DEVICE.DCTL &= ~OTG_FS_DEVICE_DCTL_SDIS;      // clear soft disconnect; enable DP pullup 
    OTG_FS_GLOBAL.GUSBCFG |= OTG_FS_GLOBAL_GUSBCFG_FDMOD | OTG_FS_GLOBAL_GUSBCFG_TRDT;  // Peripheral-only mode
    OTG_FS_DEVICE.DCFG |= OTG_FS_DEVICE_DCFG_DSPD;     // Full speed device

    // Restart the PHY clock.
    OTG_FS_PWRCLK.PCGCCTL = 0;

    /* Disable any currently active endpoints 1/2/3 out/in */
    for (int i = 1; i < 4; i++)
        if (DOEP(i)->CTL & OTG_FS_DEVICE_DOEPCTL1_EPENA)
            DOEP(i)->CTL |= OTG_FS_DEVICE_DOEPCTL1_EPDIS;

    for (int i = 1; i < 4; i++)   
        if (DIEP(i)->CTL & OTG_FS_DEVICE_DIEPCTL1_EPENA) 
            DIEP(i)->CTL |= OTG_FS_DEVICE_DIEPCTL1_EPDIS;

    // Allocate fifos (in units of uint32s)
    OTG_FS_GLOBAL.GRXFSIZ = 128;                                    // RX fifo 4x128 = 512 Bytes
    otg_fs_global_gnptxfsiz_device_set_tx0fsa(&OTG_FS_GLOBAL, 128); // start address
    otg_fs_global_gnptxfsiz_device_set_tx0fd(&OTG_FS_GLOBAL,  16);  // ep0 tx 4x16 = 64 bytes 
    otg_fs_global_dieptxf1_set_ineptxsa(&OTG_FS_GLOBAL, 128 + 16);  // start address
    otg_fs_global_dieptxf1_set_ineptxfd(&OTG_FS_GLOBAL, 128);       // ep1 tx 4x128 = 512 bytes

    OTG_FS_GLOBAL.DIEPTXF2 = 0;
    OTG_FS_GLOBAL.DIEPTXF3 = 0;

    /* Flush all tx/rx fifos */
    OTG_FS_GLOBAL.GRSTCTL = OTG_FS_GLOBAL_GRSTCTL_TXFFLSH | (1<<10) | OTG_FS_GLOBAL_GRSTCTL_RXFFLSH;

    /* Unmask interrupts for TX and RX. */
    OTG_FS_GLOBAL.GAHBCFG |= OTG_FS_GLOBAL_GAHBCFG_GINT;  // enable interrupts
    OTG_FS_GLOBAL.GINTMSK = OTG_FS_GLOBAL_GINTMSK_ENUMDNEM | OTG_FS_GLOBAL_GINTMSK_RXFLVLM | OTG_FS_GLOBAL_GINTMSK_IEPINT;
    OTG_FS_DEVICE.DAINTMSK = 0xF; // all In endpoints (rx)
    OTG_FS_DEVICE.DIEPMSK = OTG_FS_DEVICE_DIEPMSK_XFRCM; // all in endpints: notify on completed transmission

}

static void flush_txfifo(uint8_t ep) {
    DIEP(ep)->CTL |= OTG_FS_DEVICE_DIEPCTL1_SNAK;
    while (!(DIEP(ep)->INT & OTG_FS_DEVICE_DIEPINT1_INEPNE))
        __NOP();

    uint32_t fifo = (DIEP(ep)->CTL  >> 22) & 0xf;

    while (!(OTG_FS_GLOBAL.GRSTCTL & OTG_FS_GLOBAL_GRSTCTL_AHBIDL))
        __NOP();

    OTG_FS_GLOBAL.GRSTCTL  = (fifo << 6) | OTG_FS_GLOBAL_GRSTCTL_TXFFLSH;

    DIEP(ep)->TSIZ = 0;
    while (OTG_FS_GLOBAL.GRSTCTL & OTG_FS_GLOBAL_GRSTCTL_TXFFLSH)
        __NOP();
}


enum {
    REQ_TYPE_TX = 1 << 7, // bit 7 direction: 1: device->host
                          //    REQ_TYPE_VENDOR         = 1<<6, // bits 6..5 : type
                          //    REQ_TYPE_CLASS          = 1<<5, //  00 = standard, 11 is reserved

    //  REQ_TYPE_DEVICE         =  0x00,
    REQ_TYPE_INTERFACE = 0x01,
    REQ_TYPE_ENDPOINT  = 0x02,
    REQ_TYPE_OTHER     = 0x03,

    // USB Standard Request Codes - Table 9-4
    REQ_GET_STATUS     = (0 << 8) | REQ_TYPE_TX, // return 1: self-powered, 2: remote wakeup
    REQ_CLR_FEATURE    = (1 << 8),               // 1 remote wakeup-enable, 2: test mode (high-speed only)
    REQ_SET_FEATURE    = (3 << 8),
    REQ_SET_ADDRESS    = (5 << 8),               // device only
    REQ_GET_DESCRIPTOR = (6 << 8) | REQ_TYPE_TX, // device only
                                                 // REQ_SET_DESCRIPTOR      = (7<<8),                // device only       // we don't support
    REQ_GET_CONFIGURATION = (8 << 8) | REQ_TYPE_TX, // device only       // return state == USB_CONFIGURED ? 1 : 0
    REQ_SET_CONFIGURATION = (9 << 8),               // device only       // 0-> state to ADDRESS,  1 -> state to CONFIGURED (and configure)
    REQ_GET_INTERFACE     = (10 << 8) | REQ_TYPE_TX | REQ_TYPE_INTERFACE, // interface only
    REQ_SET_INTERFACE     = (11 << 8) | REQ_TYPE_INTERFACE,               // interface only
    //  REQ_SYNC_FRAME          = (12<<8)               | REQ_TYPE_ENDPOINT,  // endpoint, synch mode only, not supported

    REQ_GET_STATUS_INTERFACE  = REQ_GET_STATUS | REQ_TYPE_INTERFACE,  // return 0x0000
    REQ_CLR_FEATURE_INTERFACE = REQ_CLR_FEATURE | REQ_TYPE_INTERFACE, // noop
    REQ_SET_FEATURE_INTERFACE = REQ_SET_FEATURE | REQ_TYPE_INTERFACE, // noop

    REQ_GET_STATUS_ENDPOINT  = REQ_GET_STATUS | REQ_TYPE_ENDPOINT,  // return 0x1 if feature 'HALT' is set
    REQ_CLR_FEATURE_ENDPOINT = REQ_CLR_FEATURE | REQ_TYPE_ENDPOINT, // clear HALT (wvalue = 0), windex = 0x008f (dir/epnr)
    REQ_SET_FEATURE_ENDPOINT = REQ_SET_FEATURE | REQ_TYPE_ENDPOINT, // set HALT
};

// The most recently received SETUP request
struct {
    uint16_t req; // lower byte: Type, upper byte request code
    uint16_t val;
    uint16_t idx;
    uint16_t len;
} _ctrl_req = {0, 0, 0, 0};

static int handle_get_request(void);
static int handle_set_request(void);

size_t usb_recv(uint8_t* buf, size_t sz) {

    uint32_t intsts = OTG_FS_GLOBAL.GINTSTS;

    // clear all the unhandled interrupt flags
    OTG_FS_GLOBAL.GINTSTS = intsts & (OTG_FS_GLOBAL_GINTSTS_SRQINT|OTG_FS_GLOBAL_GINTSTS_USBSUSP|OTG_FS_GLOBAL_GINTSTS_WKUPINT|OTG_FS_GLOBAL_GINTSTS_SOF);

    // Handle USB reset done condition
    if (intsts & OTG_FS_GLOBAL_GINTSTS_ENUMDNE) {

        OTG_FS_GLOBAL.GINTSTS = OTG_FS_GLOBAL_GINTSTS_ENUMDNE;

        // Flush all tx/rx fifos
        OTG_FS_GLOBAL.GRSTCTL = OTG_FS_GLOBAL_GRSTCTL_TXFFLSH | (1<<10) | OTG_FS_GLOBAL_GRSTCTL_RXFFLSH;

        // setup endpoint zero
        otg_fs_device_diepctl0_set_mpsiz(&OTG_FS_DEVICE, 0); // packet size 64
        OTG_FS_DEVICE.DIEPTSIZ0 = 0;
        otg_fs_device_dieptsiz0_set_xfrsiz(&OTG_FS_DEVICE, 64);  // ?
        OTG_FS_DEVICE.DIEPCTL0 |= OTG_FS_DEVICE_DIEPCTL0_EPENA | OTG_FS_DEVICE_DIEPCTL0_SNAK;

        OTG_FS_DEVICE.DOEPTSIZ0 |= OTG_FS_DEVICE_DIEPTSIZ0_PKTCNT;
        otg_fs_device_doeptsiz0_set_stupcnt(&OTG_FS_DEVICE, 1);
        otg_fs_device_doeptsiz0_set_xfrsiz(&OTG_FS_DEVICE, 64); 
        OTG_FS_DEVICE.DOEPCTL0 |= OTG_FS_DEVICE_DOEPCTL0_EPENA | OTG_FS_DEVICE_DOEPCTL0_SNAK;

        otg_fs_device_dcfg_set_dad(&OTG_FS_DEVICE, 0); // zero device address

        _usb_state = USB_DEFAULT;
        return 0;
    }

    // Handle IN (tx) transfer completions

    if (DIEP(0)->INT & OTG_FS_DEVICE_DIEPINT0_XFRC) {
        // TX done on EP 0
        if (_ctrl_req.len != 0) {
            // last request was a GET, so we are here because we sent the reply
            // next thing should be the RX of the host's STATUS_OUT zero lenght packet
            DOEP(0)->CTL |= OTG_FS_DEVICE_DOEPCTL0_CNAK;
        }
    }

    for (int i = 0; i < 4; i++)
        if (DIEP(i)->INT & OTG_FS_DEVICE_DIEPINT0_XFRC)
            DIEP(i)->INT = OTG_FS_DEVICE_DIEPINT0_XFRC;

    // Handle OUT and SETUP (rx) transfers

    if (intsts & OTG_FS_GLOBAL_GINTSTS_RXFLVL) {

        uint32_t rxstsp = OTG_FS_GLOBAL.GRXSTSP_Device; // reading once pops the value
        uint32_t pktsts = (rxstsp & OTG_FS_GLOBAL_GRXSTSR_DEVICE_PKTSTS) >> 17;
        uint32_t bcnt   = (rxstsp & OTG_FS_GLOBAL_GRXSTSR_DEVICE_BCNT) >> 4;
        uint8_t  ep     = (rxstsp & OTG_FS_GLOBAL_GRXSTSR_DEVICE_EPNUM) >> 0;

        /*
           1 0001: Global OUT NAK (triggers an interrupt)
           2 0010: OUT data packet received  -> drain fifo and return buf[:len]
           3 0011: OUT transfer completed (triggers an interrupt) 
           4 0100: SETUP transaction completed (triggers an interrupt) 
           6 0110: SETUP data packet received  -> read request 8 bytes data
        */
        if (ep == 0) {

            switch (pktsts) {
            case 6: // setup packet received
                if (bcnt != 8)
                    break;

                if (DIEP(0)->TSIZ & OTG_FS_DEVICE_DOEPTSIZ1_PKTCNT)
                     flush_txfifo(0);

                uint32_t word = OTG_FS_FIFO[0].data;
                _ctrl_req.req = word;
                _ctrl_req.val = word >> 16;
                word = OTG_FS_FIFO[0].data;
                _ctrl_req.idx = word;
                _ctrl_req.len = word >> 16;

                return 0;

            case 4:  // setup completed -> handle ep0
                // if non-zero length request and direction is OUT
                // there's no request we can handle so bail out straightaway
                if ((_ctrl_req.len > 0) && !(_ctrl_req.req & REQ_TYPE_TX))
                    break;

                if (_ctrl_req.len == 0) {
                    if (!handle_set_request())
                        break;

                    // ZLP status-in reply
                    DIEP(0)->TSIZ = (1<<19);  // 1 packet, of zero bytes
                    DIEP(0)->CTL |= OTG_FS_DEVICE_DIEPCTL0_EPENA | OTG_FS_DEVICE_DIEPCTL0_CNAK;

                } else {
                    if (!handle_get_request()) // writes reply buffer
                        break;
                }
    
                // fallthrough                
            case 3:
                // out completed
                // this can only be the ZLP Status after we sent a reply
                if (bcnt != 0)
                    break;

                DOEP(0)->TSIZ = (1<<19) | 64;  // 1 packet, up to 64 bytes
                DOEP(0)->CTL |= OTG_FS_DEVICE_DOEPCTL1_EPENA | OTG_FS_DEVICE_DOEPCTL1_CNAK;
                return 0;

            }

        } else {
            // not endpoint 0

            switch (pktsts) {
            case 2: // out packet received
                return read_packet(bcnt, buf, sz);

            case 3: // out completed
                DOEP(ep)->TSIZ = (1<<19) | 64;  // 1 packet, up to 64 bytes
                DOEP(ep)->CTL |= OTG_FS_DEVICE_DOEPCTL1_EPENA | OTG_FS_DEVICE_DOEPCTL1_CNAK;
                return 0;
            }

        }

        // unhandled RX: set STALL on ep rx/tx
        DIEP(ep)->CTL |= OTG_FS_DEVICE_DIEPCTL0_STALL;
        DOEP(ep)->CTL |= OTG_FS_DEVICE_DOEPCTL0_STALL;
    }

    return 0;
}


// Setup and standard request handling

#if 1
static uint8_t _deviceDescriptor[] = {
    18,            // length of this descriptor
    0x01,          // DEVICE Descriptor Type
    0x00, 0x02,    // USB version 2.00
    0,             // Device Class per interface
    0,    0,       // subclass, protocol 0,0
    64,            //  Max Packet Size ep0
    0x83, 0x04,    // VendorID  = 0x0483 (STMicroelectronics)
    0x22, 0x57,    // ProductID = 0x5722 (Bulk demo)
    0x00, 0x02,    // Device Version 2.0
    0,    0,    0, // Manufacturer/Product/SerialNumber strings not set
    1,             // NumConfigurations
};

static uint8_t _configDescriptor[] = {
    // Config 0 header
    9,                                //  Length
    0x02,                             //  CONFIGURATION Descriptor Type
    9 + 9 + 7 + 7, 0, //  TotalLength
    1,                                //  NumInterfaces
    1,                                //  ConfigurationValue
    0,                                //  Configuration string not set
    0x80,                             //  Attributes 0x80 for historical reasons
    50,                               //  MaxPower 100mA

    // interface 0
    9,    // Length
    0x04, // INTERFACE Descriptor Type
    0, 0, // Interface Number, Alternate Setting
    2,    // Num Endpoints
    0x0A, // InterfaceClass: USB_CLASS_DATA
    0,    // InterfaceSubClass
    0,    // InterfaceProtocol
    0,    // Interface string not set

    // endpoint 0x1
    7,     //  Length
    0x05,  //  ENDPOINT Descriptor Type
    0x01,  //  Endpoint Address: 1-OUT
    0x02,  //  Attributes: BULK
    64, 0, //  MaxPacketSize
    0,     //  Interval, ignored for BULK

    // endpoint 0x81
    7,     //  Length
    0x05,  //  ENDPOINT Descriptor Type
    0x81,  //  Endpoint Address 1-IN
    0x02,  //  Attributes: BULK
    64, 0, //  MaxPacketSize
    0,     //  Interval, ignored for BULK
};
#else
static uint8_t _deviceDescriptor[] = {
    18,            // length of this descriptor
    0x01,          // DEVICE Descriptor Type
    0x00, 0x02,    // USB version 2.00
    2,             // Device Class = CDC
    0,    0,       // subclass, protocol 0,0
    64,            //  Max Packet Size ep0
    0x83, 0x04,    // VendorID  = 0x0483 (STMicroelectronics)
    0x40, 0x57,    // ProductID = 0x5740 (Virtual COM Port)
    0x00, 0x02,    // Device Version 2.0
    0,    0,    0, // Manufacturer/Product/SerialNumber strings not set
    1,             // NumConfigurations
};

static uint8_t _configDescriptor[] = {
    // Config 0 header
    9,                                //  Length
    0x02,                             //  CONFIGURATION Descriptor Type
    9 + 9 + 5 + 4 + 5 + 9 + 7 + 7, 0, //  TotalLength
    2,                                //  NumInterfaces
    1,                                //  ConfigurationValue
    0,                                //  Configuration string not set
    0x80,                             //  Attributes 0x80 for historical reasons
    50,                               //  MaxPower 100mA

    // interface 0
    9,    // Length
    0x04, // INTERFACE Descriptor Type
    0, 0, // Interface Number, Alternate Setting
    0,    // Num Endpoints
    0x02, // InterfaceClass:    CDC
    0x02, // InterfaceSubClass: ACM
    0,    // InterfaceProtocol: NONE
    0,    // Interface string not set

    // CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
    5,          // bFunctionLength
    0x24,       // bDescriptorType    CS_INTERFACE
    0x00,       // bDescriptorSubtype USB_CDC_TYPE_HEADER
    0x10, 0x01, // bcdCDC version 1.10

    // Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
    4,    // bFunctionLength
    0x24, // bDescriptorType  CS_INTERFACE
    0x02, // bDescriptorSubtype USB_CDC_TYPE_ACM
    0x00, // bmCapabilities: none

    // Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
    5,    // bFunctionLength
    0x24, // bDescriptorType  CS_INTERFACE
    0x06, // bDescriptorSubtype USB_CDC_TYPE_UNION
    0,    // bMasterInterface
    1,    // bSlaveInterface0

    // interface 1
    9,    // Length
    0x04, // INTERFACE Descriptor Type
    1, 0, // Interface Number, Alternate Setting
    2,    // Num Endpoints
    0x0A, // InterfaceClass: USB_CLASS_DATA
    0,    // InterfaceSubClass
    0,    // InterfaceProtocol
    0,    // Interface string not set

    // endpoint 0x1
    7,     //  Length
    0x05,  //  ENDPOINT Descriptor Type
    0x01,  //  Endpoint Address: 1-OUT
    0x02,  //  Attributes: BULK
    64, 0, //  MaxPacketSize
    0,     //  Interval, ignored for BULK

    // endpoint 0x81
    7,     //  Length
    0x05,  //  ENDPOINT Descriptor Type
    0x81,  //  Endpoint Address 1-IN
    0x02,  //  Attributes: BULK
    64, 0, //  MaxPacketSize
    0,     //  Interval, ignored for BULK
};
#endif

// false on failure, true on success
static int handle_set_request(void) {
    switch (_ctrl_req.req) {
    case REQ_SET_ADDRESS:
        if ((_usb_state == USB_CONFIGURED) || (_ctrl_req.val > 127) || (_ctrl_req.idx != 0))
            return 0;
        // this isnt supposed to take effect until afther the STATUS-IN but apparently the hardware handles that
        otg_fs_device_dcfg_set_dad(&OTG_FS_DEVICE, _ctrl_req.val);
         _usb_state = (_ctrl_req.val == 0) ? USB_DEFAULT : USB_ADDRESS;
        return 1;

    case REQ_SET_CONFIGURATION:
        switch (_usb_state) {
        case USB_ADDRESS:
            switch (_ctrl_req.val) {
            case 1:
                // configure our endpoint 0x01/0x81
                DIEP(0x81)->TSIZ = 0;
                DIEP(0x81)->CTL = OTG_FS_DEVICE_DIEPCTL0_EPENA | OTG_FS_DEVICE_DIEPCTL0_SNAK | (2 << 18)| OTG_FS_DEVICE_DIEPCTL0_USBAEP | OTG_FS_DEVICE_DIEPCTL1_SD0PID_SEVNFRM | (1 << 22) | 64;

                DOEP(0x1)->TSIZ = (1<<19) | 64;
                DOEP(0x1)->CTL = OTG_FS_DEVICE_DOEPCTL0_EPENA | OTG_FS_DEVICE_DOEPCTL0_USBAEP | OTG_FS_DEVICE_DOEPCTL0_CNAK | OTG_FS_DEVICE_DOEPCTL1_SD0PID_SEVNFRM | (2 << 18) | 64;

                _usb_state = USB_CONFIGURED;
                // fallthrough
            case 0:
                return 1;
            }
            return 0;

        case USB_CONFIGURED:
            switch (_ctrl_req.val) {
            case 0:
                // unconfigure our endpoints 0x01/0x81
                DIEP(0x81)->CTL |= OTG_FS_DEVICE_DIEPCTL1_EPDIS;
                DOEP(0x1)->CTL  |= OTG_FS_DEVICE_DOEPCTL1_EPDIS;

                _usb_state = USB_ADDRESS;
                // fallthrough
            case 1:
                return 1;
            }
        default:
            break;
        }
        return 0;

    case REQ_SET_INTERFACE:
        return (_usb_state == USB_CONFIGURED) && (_ctrl_req.idx == 0);
        // case REQ_CLR_FEATURE:
        // case REQ_SET_FEATURE:
        // case REQ_CLR_FEATURE_INTERFACE:
        // case REQ_SET_FEATURE_INTERFACE:
        //    return 0; // no features implemented at device or interface level

#if 0 // THESE ARE SUPPOSED TO BE MANDATORY BUT WE SEEM TO BE FINE WITHOUT THEM
    case REQ_CLR_FEATURE_ENDPOINT:
        if ((_usb_state != USB_CONFIGURED) || ((_ctrl_req.idx & 0xf) != 1))
            return 0;
        if (_ctrl_req.idx & 0x80) {
            usb_ep_set_stat_tx(1, USB_EP_STAT_NAK);
            usb_ep_clr_dtog_tx(1);
        } else {
            usb_ep_set_stat_rx(1, USB_EP_STAT_NAK);
            usb_ep_clr_dtog_rx(1);
        }
        return 1;

    case REQ_SET_FEATURE_ENDPOINT:
        if ((_usb_state != USB_CONFIGURED) || ((_ctrl_req.idx & 0xf) != 1))
            return 0;
        if (_ctrl_req.idx & 0x80) {
            usb_ep_set_stat_tx(1, USB_EP_STAT_STALL);
        } else {
            usb_ep_set_stat_rx(1, USB_EP_STAT_STALL);
        }
        return 1;
#endif
    }

    return 0;
}

// false on failure, true on success
static int handle_get_request(void) {
    uint8_t data[2] = {0, 0};
    size_t  len;
    switch (_ctrl_req.req) {
    case REQ_GET_DESCRIPTOR:
        if (_ctrl_req.idx != 0)
            return 0;
        switch (_ctrl_req.val) {
        case 0x0100:
            len = sizeof _deviceDescriptor;
            if (len > _ctrl_req.len)
                len = _ctrl_req.len;
            if(write_packet(0, _deviceDescriptor, len))
                return 1;
            break;
        case 0x0200:
            len = sizeof _configDescriptor;
            if (len > _ctrl_req.len)
                len = _ctrl_req.len;
            if(write_packet(0, _configDescriptor, len))
                return 1;
            break;
        }
        return 0;

    case REQ_GET_STATUS:
        if ((_usb_state == USB_DEFAULT) || (_ctrl_req.val != 0) || (_ctrl_req.idx != 0) || (_ctrl_req.len != 2))
            return 0;
        break;

    case REQ_GET_STATUS_INTERFACE:
        if ((_usb_state == USB_DEFAULT) || (_ctrl_req.val != 0) || (_ctrl_req.idx != 0) || (_ctrl_req.len != 2))
            return 0;
        break;

    case REQ_GET_STATUS_ENDPOINT:
        if ((_usb_state == USB_DEFAULT) || (_ctrl_req.val != 0) || (_ctrl_req.len != 2))
            return 0;
        switch (_ctrl_req.idx) {
        case 0x00:
            data[0] = (DOEP(0)->CTL & OTG_FS_DEVICE_DOEPCTL0_STALL) ? 1 : 0;
            break;
        case 0x80:
            data[0] = (DIEP(0)->CTL & OTG_FS_DEVICE_DIEPCTL0_STALL) ? 1 : 0;
            break;
        case 0x01:
            data[0] = (DOEP(1)->CTL & OTG_FS_DEVICE_DOEPCTL1_STALL) ? 1 : 0;
            break;
        case 0x81:
            data[0] = (DIEP(1)->CTL & OTG_FS_DEVICE_DIEPCTL1_STALL) ? 1 : 0;
            break;
        default:
            return 0;
        }
        break;

    case REQ_GET_CONFIGURATION:
        if ((_usb_state == USB_DEFAULT) || (_ctrl_req.len != 1))
            return 0;
        data[0] = (_usb_state == USB_CONFIGURED) ? 1 : 0;
        break;

    case REQ_GET_INTERFACE:
        if ((_usb_state != USB_CONFIGURED) || (_ctrl_req.len != 1))
            return 0;
        break;
    }

    return write_packet(0, data, _ctrl_req.len) == _ctrl_req.len;
}
