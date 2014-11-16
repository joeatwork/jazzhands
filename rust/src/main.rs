#![feature(macro_rules)]

extern crate libc;

use libc::{c_int, c_ulong, c_uchar, c_void, size_t};

use std::default::{Default};
use std::io::{IoError};
use std::os;

#[allow(non_camel_case_types)] type tcflag_t = libc::c_ulong;
#[allow(non_camel_case_types)] type speed_t = libc::c_ulong;
#[allow(non_camel_case_types)] type cc_t = libc::c_uchar;

// Values from fcntl.h on my Mac OS 10.10 system
const CSIZE: tcflag_t = 0x00000300;
const CS8: tcflag_t = 0x00000300;

const CSTOPB: tcflag_t = 0x00000400;	/* send 2 stop bits */
const CREAD: tcflag_t = 0x00000800;	/* enable receiver */
const CLOCAL: tcflag_t = 0x00008000;	/* ignore modem status lines */
const CCTS_OFLOW: tcflag_t = 0x00010000;	/* CTS flow control of output */
const CRTS_IFLOW: tcflag_t = 0x00020000;	/* RTS flow control of input */
const CRTSCTS: tcflag_t = (CCTS_OFLOW | CRTS_IFLOW);
const PARENB: tcflag_t = 0x00001000;	/* parity enable */

const IXON: tcflag_t = 0x00000200;	/* enable output flow control */
const IXOFF: tcflag_t = 0x00000400;	/* enable input flow control */
const IXANY: tcflag_t = 0x00000800;	/* any char will restart after stop */

const ICANON: tcflag_t = 0x00000100;	/* canonicalize input lines */
const ECHO: tcflag_t = 0x00000008;	/* enable echoing */
const ECHOE: tcflag_t = 0x00000002;	/* visually erase chars */
const ISIG: tcflag_t = 0x00000080;	/* enable signals INTR, QUIT, [D]SUSP */

const OPOST: tcflag_t = 0x00000001;	/* enable following output processing */

const VMIN: uint = 16;	/* !ICANON */
const VTIME: uint = 17;	/* !ICANON */

const TCSAFLUSH: libc::c_int = 2; /* drain output, flush input */

const O_NOCTTY: libc::c_int = 0x20000;
const O_NDELAY: libc::c_int = 0x0004;
const NCCS: uint = 20;

const B115200: speed_t = 115200;

#[repr(C)]
struct Termios {
    // From termios.h on my Mac OS 10.10 system
    c_iflag: tcflag_t,
    c_oflag: tcflag_t,
    c_cflag: tcflag_t,
    c_lflag: tcflag_t,
    c_cc: [cc_t, ..NCCS],
    c_ispeed: speed_t,
    c_ospeed: speed_t,
}

impl Default for Termios {
    fn default() -> Termios {
        Termios {
            c_iflag: 0,
            c_oflag: 0,
            c_cflag: 0,
            c_lflag: 0,
            c_cc: [0, ..NCCS],
            c_ispeed: 0,
            c_ospeed: 0,
        }
    }
}

const FAILED: c_int = -1;
const SUCCEEDED: c_int = 0;

// Calls a c function
macro_rules! checked_c_io(
    ($call:expr) => (
        match $call {
            FAILED => return Err(IoError::last_error()),
            SUCCEEDED => (),
            _ => unreachable!(),
        }
    )
)

#[link(name = "c")]
extern {
    fn cfsetispeed(termios: *mut Termios, speed: speed_t) -> c_int;
    fn cfsetospeed(termios: *mut Termios, speed: speed_t) -> c_int;
    fn tcgetattr(fd: c_int, termios: *mut Termios) -> c_int;
    fn tcsetattr(fd: c_int, optional_actions: c_int, termios: *const Termios) -> c_int;
}

const MESSAGE_LENGTH: uint = 50;

fn run_reader(device: Path) -> Result<(), IoError> {
    let mut scratch_buffer = [0u8, ..MESSAGE_LENGTH];

    let flags = libc::O_RDWR | O_NOCTTY | O_NDELAY;
    let fd_option = device.with_c_str(|path_str| unsafe {
        libc::open(path_str, flags, 0)
    });
    let fd = match fd_option {
        -1 => return Err(IoError::last_error()),
        fd => fd
    };

    let mut toptions: Termios = Default::default();
    unsafe {
        checked_c_io!(tcgetattr(fd, &mut toptions));
        checked_c_io!(cfsetispeed(&mut toptions, B115200));
        checked_c_io!(cfsetospeed(&mut toptions, B115200));
    }

    // 8 bits per character
    toptions.c_cflag &= !CSIZE;
    toptions.c_cflag |= CS8;

    toptions.c_cflag &= !PARENB; // no parity
    toptions.c_cflag &= !CSTOPB; // one stop bit

    toptions.c_cflag &= !CRTSCTS; // No hardwre flow control
    toptions.c_cflag |= CREAD; // Enable read
    toptions.c_cflag |= CLOCAL; // ignore modem control lines

    toptions.c_iflag &= !(IXON | IXOFF | IXANY); // No flow control
    toptions.c_iflag &= !(ICANON | ECHO | ECHOE | ISIG); // No echo, not in canonical mode

    toptions.c_oflag &= !OPOST; // No output processing

    toptions.c_cc[VMIN] = 24; // Buffer at least 24 characters before returning from read
    toptions.c_cc[VTIME] = 1; // OR return from read after 0.1 second of silence

    unsafe {
        // Drop everything that has happened, start fresh
        checked_c_io!(tcsetattr(fd, TCSAFLUSH, &toptions));
    }

    // TODO: There is very likely a much more rusty way to deal with this
    loop {
        let bytes = unsafe {
            let len = scratch_buffer.len() as size_t;
            let bytes = libc::read(fd, scratch_buffer.as_mut_ptr() as *mut c_void, len);
            if -1 == bytes {
                let err = os::errno();
                if err == libc::EAGAIN as uint {
                    continue;
                } else {
                    return Err(IoError::last_error());
                }
            }

            bytes
        };
        println!("Read {} bytes ok!", bytes);
        break;
    }

    Ok(())
}

fn main() {
    let args = os::args();
    if args.len() != 2 {
        println!("Usage: {} [serial port name]", args[0]);
        return;
    }

    let port_name = args[1].clone();
    let device = Path::new(&port_name);
    match run_reader(device) {
        Ok(_) => println!("It opened and closed"),
        Err(err) => println!("ERROR: \"{}\" {}", port_name, err.desc)
    };
}