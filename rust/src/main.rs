#![feature(macro_rules)]

use std::os;
use std::sync::Arc;
use std::sync::atomic::{AtomicOption, Acquire};

mod device;
mod message;

fn main() {
    let args = os::args();
    if args.len() != 2 {
        println!("Usage: {} [serial port name]", args[0]);
        return;
    }

    let port_name = args[1].clone();
    let device = Path::new(&port_name);

    let write_to = Arc::new(AtomicOption::empty());
    let read_from = write_to.clone();
    let (write_terminate, read_terminate) = sync_channel(0);

    spawn(proc() {
        loop {
            match read_from.take(Acquire) {
                None => (),
                Some(message) => {
                    println!("Got message {}", message);
                }
            }

            let terminate = read_terminate.try_recv();
            if terminate.is_ok() {
                break;
            }
        }
    });

    match device::run_reader(device, write_to) {
        Ok(_) => println!("Device closed"),
        Err(err) => println!("ERROR: \"{}\" {}", port_name, err.desc)
    };

    println!("Terminating osc process");
    write_terminate.send(true);
    println!("Bye!");
}