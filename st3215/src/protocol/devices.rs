// ST3215 USB identifiers
const ST3215_VID: u16 = 0x1a86;
const ST3215_PID: u16 = 0x55d3;

pub fn is_st3215_usbdevice(vid: u16, pid: u16) -> bool {
    vid == ST3215_VID && pid == ST3215_PID
}
