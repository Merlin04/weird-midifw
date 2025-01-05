//! The starter code slowly blinks the LED and sets up
//! USB logging. It periodically logs messages over USB.
//!
//! Despite targeting the Teensy 4.0, this starter code
//! should also work on the Teensy 4.1 and Teensy MicroMod.
//! You should eventually target your board! See inline notes.
//!
//! This template uses [RTIC v2](https://rtic.rs/2/book/en/)
//! for structuring the application.

#![no_std]
#![no_main]

mod hardware;

use core::arch::asm;
use teensy4_bsp::ral;

use teensy4_panic as _;

fn reboot_into_bootloader() -> ! {
    let cfg5 = unsafe { ral::read_reg!(ral::ocotp, ral::ocotp::OCOTP, CFG5) };
    if (cfg5 & 0x02) == 0 {
        unsafe {
            asm!("bkpt #251"); // invoke the ancient wisdom of Paul Stoffregen
            // https://github.com/PaulStoffregen/cores/blob/058d2808187a24e7db53803b7510e26827064c03/teensy4/usb.c#L214C3-L214C20
        }
        panic!("tried to jump to bootloader but failed!");
    } else {
        panic!("jump to bootloader when secure mode is enabled is unimplemented");
        // disable irq?? idk how
        // ?????
        // unsafe {
        //     ral::write_reg!(ral::usb, ral::usb::USB1, USBCMD, 0);
        //     ral::write_reg!(ral::iomuxc_gpr, ral::iomuxc_gpr::GPR16, 0x00200003);


        //     let first_address = core::ptr::read_volatile(0x0020001C as *const u32);
        //     let function_address = core::ptr::read_volatile((first_address + 8) as *const u32);
        //     let code: extern "C" fn(*const u32) = core::mem::transmute(function_address as *const ());
        //     (code)(5);
        //     panic!("yeah");
        // }
    }
}

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use critical_section::Mutex;
    use crate::hardware::i2c_mp::I2cBankBus;
    use static_cell::StaticCell;
    use teensy4_bsp::{board::{lpi2c, Lpi2c3}, hal::usbd::{
        gpt::{Instance::Gpt0, Mode},
        BusAdapter, EndpointMemory, EndpointState, Speed
    }};
    use teensy4_bsp::board;

    use usb_device::{
        bus::UsbBusAllocator,
        device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid}
    };
    use usbd_serial::{CustomControlRequestHandler, SerialPort};
    use usbd_midi::{data::{byte::{from_traits::FromClamped, u7::U7}, midi::{channel::Channel, message::{control_function::ControlFunction, Message}}, usb_midi::{cable_number::CableNumber, usb_midi_event_packet::UsbMidiEventPacket}}, midi_device::MidiClass};

    use defmt_bbq::DefmtConsumer;

    use rtic_monotonics::{systick::{Systick, *}, Monotonic};

    // sure
    use core::{cell::RefCell, sync::atomic::{AtomicU32, Ordering}};

    use crate::{hardware::{encoder::EncoderAS5600, i2c_mp::{I2cBank, I2cBus, MultiplexedI2c}}, reboot_into_bootloader};
    static COUNT: AtomicU32 = AtomicU32::new(0);
    defmt::timestamp!("{=u32:us}", COUNT.fetch_add(1, Ordering::Relaxed));

    const SPEED: Speed = Speed::LowFull;
    const VID_PID: UsbVidPid = UsbVidPid(0x16C0, 0x0477); // needs to be this for teensy rebootor

    struct RebootRequestHandler {}
    impl CustomControlRequestHandler<BusAdapter> for RebootRequestHandler {
        fn handle_request(&mut self, _req: &usb_device::class::ControlOut<'_,'_,'_,BusAdapter>) -> bool {
            reboot_soon::spawn().ok();
            true
        }
    }

    #[shared]
    struct Shared {
        midi_class: MidiClass<'static, BusAdapter>
    }

    #[local]
    struct Local {
        serial_class: SerialPort<'static, BusAdapter, RebootRequestHandler>,
        usb_device: UsbDevice<'static, BusAdapter>,
        defmt_consumer: DefmtConsumer,
        led: board::Led,
        ctrl_enc: EncoderAS5600<Lpi2c3>,
        param_encs: [EncoderAS5600<Lpi2c3>; 4]
    }

    #[init(local = [
        ep_memory: EndpointMemory<1024> = EndpointMemory::new(),
        ep_state: EndpointState = EndpointState::max_endpoints(),
        usb_bus: Option<UsbBusAllocator<BusAdapter>> = None
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
            usb,
            lpi2c3,
            ..
        } = board::t40(cx.device);
        let led = board::led(&mut gpio2, pins.p13);
        
        let bus_adapter = BusAdapter::with_speed(usb, cx.local.ep_memory, cx.local.ep_state, SPEED);
        bus_adapter.set_interrupts(true);
        bus_adapter.gpt_mut(Gpt0, |gpt| {
            gpt.stop();
            gpt.clear_elapsed();
            gpt.set_interrupt_enabled(true);
            gpt.set_mode(Mode::Repeat);
            gpt.set_load(10_000); // ms
            gpt.reset();
            gpt.run();
        });
        
        let usb_bus = cx.local.usb_bus.insert(UsbBusAllocator::new(bus_adapter));
        let serial_class = SerialPort::new(usb_bus, RebootRequestHandler {});
        let midi_class = MidiClass::new(usb_bus, 0, 1).unwrap();
        
        let usb_device = UsbDeviceBuilder::new(usb_bus, VID_PID)
            .device_class(usbd_serial::USB_CLASS_CDC)
            .device_sub_class(0)
            .max_packet_size_0(64) // ????
            .unwrap()
            .strings(&[StringDescriptors::default()
                .manufacturer("me")
                .product("weird-midi")
                .serial_number("12345678")])
            .unwrap()
            .build();

        let defmt_consumer = defmt_bbq::init().unwrap();

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        let i2c3: Lpi2c3 = lpi2c(lpi2c3, pins.p16, pins.p17, board::Lpi2cClockSpeed::KHz400);
        let mi2c = MultiplexedI2c::new(i2c3).unwrap();
        static I2C_BUS: StaticCell<I2cBus<Lpi2c3>> = StaticCell::new();
        let i2c_bus = I2C_BUS.init(Mutex::new(RefCell::new(mi2c)));

        static ENC_BANK: StaticCell<I2cBankBus<Lpi2c3>> = StaticCell::new();
        let enc_bank = ENC_BANK.init(Mutex::new(RefCell::new(I2cBank {
            start: 0,
            len: 5,
            active: 0,
            i2c: i2c_bus
        })));
        
        let param_encs: [_; 4] = core::array::from_fn(|i: usize| EncoderAS5600::new(enc_bank, i.try_into().unwrap(), None).unwrap());
        let ctrl_enc = EncoderAS5600::new(enc_bank, 4, Some(24)).unwrap();

        blink::spawn().unwrap();

        (Shared {
            midi_class
        }, Local {
            led,
            serial_class,
            usb_device,
            defmt_consumer,
            param_encs,
            ctrl_enc
        })
    }

    #[task(local = [led])]
    async fn blink(cx: blink::Context) {
        let mut count = 0u32;
        loop {
            cx.local.led.toggle();
            Systick::delay(500.millis()).await;

            defmt::info!("Hello from your Teensy 4! The count is {=u32}", count);
            if count % 7 == 0 {
                defmt::warn!("Here's a warning at count {=u32}", count);
            }
            if count % 23 == 0 {
                defmt::error!("Here's an error at count {=u32}", count);
            }

            count = count.wrapping_add(1);
        }
    }

    #[task(shared = [midi_class], local = [param_encs, ctrl_enc])]
    async fn process_param_encs(mut cx: process_param_encs::Context) {
        let process_param_encs::LocalResources {
            param_encs,
            ctrl_enc,
            ..
        } = cx.local;
        loop {
            let start = Systick::now();

            for e in param_encs.iter_mut() {
                match e.process() {
                    Ok(n) => {
                        if n == 0 {
                            continue;
                        }
                        let cable_number = CableNumber::Cable0;
                        let channel = Channel::Channel1;
                        // relative encoder: send 63 if decrement, 65 if increment
                        // todo: repeat messages
                        let message = Message::ControlChange(
                            channel,
                            ControlFunction(U7::from_clamped(20 + e.index)),
                            U7::from_clamped(if n > 0 { 65 } else { 63 })
                        );
                        let packet = UsbMidiEventPacket::from_midi(cable_number, message);
                        let _ = cx.shared.midi_class.lock(|midi_class| {
                            midi_class.send_message(packet)
                        }).inspect_err(|e| {
                            defmt::error!("Failed to send MIDI packet {:?}", e);
                        });
                    },
                    Err(_e) => defmt::error!("Failed to communicate with encoder")
                }
            }

            match ctrl_enc.process() {
                Ok(n) => {
                    if n == 0 { continue; }
                    defmt::info!("ctrl enc {=i16}", n);
                },
                Err(_e) => defmt::error!("Failed to communicate with ctrl encoder")
            }

            Systick::delay_until(start + 2.millis()).await;
        }
    }

    // remove defmt data from queue and send to usb host
    #[task(binds = USB_OTG1, shared = [midi_class], local = [
        serial_class, usb_device, defmt_consumer,
        configured: bool = false
    ])]
    fn usb_interrupt(mut cx: usb_interrupt::Context) {
        let usb_interrupt::LocalResources {
            serial_class,
            usb_device,
            defmt_consumer,
            configured,
            ..
        } = cx.local;

        usb_device.bus().gpt_mut(Gpt0, |gpt| {
            while gpt.is_elapsed() {
                gpt.clear_elapsed();
            }
        });

        cx.shared.midi_class.lock(|midi_class| {
            if usb_device.poll(&mut [serial_class, midi_class]) {
                if usb_device.state() == UsbDeviceState::Configured {
                    // first configuration?
                    if !*configured {
                        usb_device.bus().configure();
                    }
                    *configured = true;
    
                    // rebootor
                    let mut buffer = [0; 64];
                    match serial_class.read(&mut buffer) {
                        Ok(count) => {
                            // led.toggle();
                            serial_class.write(&buffer[..count]).ok();
                            serial_class.write(&buffer[..count]).ok();
                        },
                        Err(usb_device::UsbError::WouldBlock) => {}
                        Err(err) => defmt::error!("{:?}", err)
                    }
                } else {
                    // might have lost our configuration!
                    *configured = false;
                }
            }    
        });

        // TODO: midi
        if *configured {
            while let Ok(grant) = defmt_consumer.read() {
                if let Ok(written) = serial_class.write(&grant) {
                    grant.release(written);
                } else {
                    break;
                }
            }
        }
    }

    #[task(priority = 1)]
    async fn reboot_soon(_cx: reboot_soon::Context) {
        Systick::delay(10.millis()).await;
        reboot_into_bootloader();
    }
}
