#![no_std]
#![no_main]

mod hardware;
mod velocity;
mod bootloader;

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP, ENET2])]
mod app {
    use critical_section::Mutex;
    use fugit::Instant;
    use crate::{bootloader::reboot_into_bootloader, hardware::matrix_pins::KeyType};
    use rtic_sync::{channel::Receiver, make_channel};
    use crate::{hardware::{i2c_mp::I2cBankBus, debounce::Debounce, matrix_pins::{disable_matrix_interrupts, enable_matrix_interrupts, key_type, MatrixInPins, MatrixInPinsExt, MatrixOutPins, MatrixOutPinsExt, N_COLS, N_ROWS}}, velocity::{self, Key, KeyEvent, KEY_EVENT_CAPACITY}};
    use static_cell::StaticCell;
    use teensy4_bsp::{board::{lpi2c, Lpi2c3}, hal::{gpio::Port, usbd::{
        gpt::{Instance::Gpt0, Mode},
        BusAdapter, EndpointMemory, EndpointState, Speed
    }}, ral};
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
    use core::{array, cell::RefCell, sync::atomic::{AtomicU32, Ordering}};

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
        midi_class: MidiClass<'static, BusAdapter>,
        gpio1: Port<1>,
        gpio2: Port<2>,
        matrix_in_pins: MatrixInPins,
        matrix_out_pins: MatrixOutPins,
        matrix_velocity: MatrixVelocity,
        kscan_scan_time: Instant<u32, 1, 10000>
    }

    type MatrixDebounce = [[Debounce; N_ROWS]; N_COLS];
    type MatrixVelocity = [[velocity::Key; N_ROWS]; N_COLS];

    #[local]
    struct Local {
        serial_class: SerialPort<'static, BusAdapter, RebootRequestHandler>,
        usb_device: UsbDevice<'static, BusAdapter>,
        defmt_consumer: DefmtConsumer,
        led: board::Led,
        ctrl_enc: EncoderAS5600<Lpi2c3>,
        param_encs: [EncoderAS5600<Lpi2c3>; 4],
        matrix_debounce: MatrixDebounce,
        key_receiver: Receiver<'static, KeyEvent, KEY_EVENT_CAPACITY>
    }

    #[init(local = [
        ep_memory: EndpointMemory<1024> = EndpointMemory::new(),
        ep_state: EndpointState = EndpointState::max_endpoints(),
        usb_bus: Option<UsbBusAllocator<BusAdapter>> = None
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio1,
            mut gpio2,
            mut gpio3,
            mut gpio4,
            pins,
            usb,
            lpi2c3,
            ..
        } = board::t41(cx.device);
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

        // *screams*
        let matrix_in_pins = (
            gpio2.input(pins.p35), gpio2.input(pins.p36), gpio2.input(pins.p37),
            gpio1.input(pins.p38), gpio1.input(pins.p39), gpio1.input(pins.p40),
            gpio1.input(pins.p41), gpio1.input(pins.p14), gpio1.input(pins.p15),
            gpio1.input(pins.p25), gpio1.input(pins.p24),
            gpio2.input(pins.p12), gpio2.input(pins.p7)
        );

        enable_matrix_interrupts(&mut gpio1, &mut gpio2, &matrix_in_pins);

        let matrix_out_pins: MatrixOutPins = (
            gpio2.output(pins.p32),
            gpio3.output(pins.p31), gpio3.output(pins.p30),
            gpio4.output(pins.p29),
            gpio3.output(pins.p28),
            gpio1.output(pins.p27), gpio1.output(pins.p26),
            gpio4.output(pins.p33),
            gpio2.output(pins.p34)
        );

        blink::spawn().unwrap();

        let matrix_debounce: MatrixDebounce = array::from_fn(|_| array::from_fn(|_| Debounce(0)));

        let (key_sender, key_receiver) = make_channel!(KeyEvent, KEY_EVENT_CAPACITY);
        let start_timeout_callback = Mutex::new(RefCell::new(|row, col| {
            // TODO - timeout handler???
        }));
        let matrix_velocity: MatrixVelocity = array::from_fn(|col| array::from_fn(|row|
            velocity::Key::new(row.try_into().unwrap(), col.try_into().unwrap(), key_sender.clone(), &start_timeout_callback
        )));

        kscan_matrix_read::spawn(0).unwrap();

        (Shared {
            midi_class,
            gpio1,
            gpio2,
            matrix_in_pins,
            matrix_out_pins,
            matrix_velocity,
            kscan_scan_time: Systick::now()
        }, Local {
            led,
            serial_class,
            usb_device,
            defmt_consumer,
            param_encs,
            ctrl_enc,
            matrix_debounce,
            key_receiver
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

    #[task(priority = 1, shared = [midi_class], local = [key_receiver])]
    async fn process_key_events(cx: process_key_events::Context) {
        let process_key_events::LocalResources {
            key_receiver,
            ..
        } = cx.local;

        while let Ok(val) = key_receiver.recv().await {

        }
    }

    #[task(priority = 1, shared = [midi_class], local = [param_encs, ctrl_enc])]
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
                        let _ = (0..(n.abs())).fold::<Result<(), usb_device::UsbError>, _>(Ok(()), |acc, _| {
                            let message = Message::ControlChange(
                                channel,
                                ControlFunction(U7::from_clamped(20 + e.index)),
                                U7::from_clamped(if n > 0 { 65 } else { 63 })
                            );    
                            let packet = UsbMidiEventPacket::from_midi(cable_number, message);
                            cx.shared.midi_class.lock(|midi_class| {
                                midi_class.send_message(packet)
                            }).and(acc)
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

    const KSCAN_DEBOUNCE_SCAN_PERIOD_MS: u32 = 1;
    const KSCAN_COL_DELAY_US: u32 = 5;

    #[task(priority = 2, shared = [gpio1, gpio2, matrix_in_pins, matrix_out_pins, matrix_velocity, kscan_scan_time], local = [matrix_debounce])]
    async fn kscan_matrix_read(mut cx: kscan_matrix_read::Context, mut poll_counter: u8) {
        let kscan_matrix_read::LocalResources {
            matrix_debounce,
            ..
        } = cx.local;

        // Scan the matrix.

        loop {
            Systick::delay(5.micros()).await; // hardware is so bad

            let continue_scan = cx.shared.matrix_out_pins.lock(|matrix_out_pins| {
                cx.shared.matrix_in_pins.lock(|matrix_in_pins| async {
                    let outputs = matrix_out_pins.as_array();
                    let inputs = matrix_in_pins.as_array();

                    for out_idx in 0..outputs.len() {
                        outputs[out_idx].set();
                        
                        for in_idx in 0..inputs.len() {
                            if in_idx == 12 && out_idx < 2 {
                                continue;
                            }
                            
                            let active = inputs[in_idx].is_set(); // assume INPUT_PULLDOWN (active high)
                            matrix_debounce[out_idx][in_idx].update(active, KSCAN_DEBOUNCE_SCAN_PERIOD_MS.into());
                        }

                        outputs[out_idx].clear();
                        Systick::delay(KSCAN_COL_DELAY_US.micros()).await; // electron moment, I think this is waiting for the diode to switch?? unclear
                    }

                    // Process the new state.
                    let mut continue_scan = poll_counter > 0; // sometimes an interrupt will be triggered but the switch will jitter a bit and seem like it wasn't pressed
                    // but we know it was pressed, so continue even if the debouncer says nothing is active

                    // iterate through all rows/cols, find the keys where debounce_get_changed, then callback
                    // then continue_scan ||= debounce_is_active
                    for out_idx in 0..outputs.len() {
                        for in_idx in 0..inputs.len() {
                            let debounce = matrix_debounce[out_idx][in_idx];
                            if debounce.changed() {
                                let pressed = debounce.pressed();
                                // key event!

                                let k_type = key_type(in_idx.try_into().unwrap(), out_idx.try_into().unwrap());
                                if k_type == KeyType::Ctrl {
                                    // TODO: handle ctrl key press
                                } else {
                                    cx.shared.matrix_velocity.lock(|matrix_velocity| {
                                        use velocity::velocity_sense::Input;
                                        matrix_velocity[out_idx][in_idx].process_event(if k_type == KeyType::Upper {
                                            if pressed { Input::TopPress } else { Input::TopRelease }
                                        } else {
                                            if pressed { Input::BottomPress } else { Input::BottomRelease }
                                        });
                                    });
                                }
                            }
                            continue_scan = continue_scan || debounce.is_active();
                        }
                    }
                    continue_scan
                })
            }).await;

            if continue_scan {
                // At least one key is pressed or the debouncer has not yet decided if
                // it is pressed. Poll quickly until everything is released.
                // kscan_scan_time += KSCAN_DEBOUNCE_SCAN_PERIOD_MS * 1000; // microseconds
                let run_at = cx.shared.kscan_scan_time.lock(|ksc| {
                    *ksc += KSCAN_DEBOUNCE_SCAN_PERIOD_MS.millis();
                    *ksc
                });

                if poll_counter > 0 {
                    poll_counter -= 1;
                }

                Systick::delay_until(run_at).await;
                // matrix_scheduler.schedule_at(kscan_scan_time, micros(), poll_counter == 0 ? 0 : poll_counter - 1);
            } else {
                // All keys are released. Return to normal.
                // Return to waiting for an interrupt.
                cx.shared.gpio1.lock(|gpio1| {
                    cx.shared.gpio2.lock(|gpio2| {
                        cx.shared.matrix_in_pins.lock(|matrix_in_pins| {
                            enable_matrix_interrupts(gpio1, gpio2, matrix_in_pins);
                        })
                    })
                });
                break;
            }
        }
    }

    fn handle_gpio_interrupt(
        mut gpio1: shared_resources::gpio1_that_needs_to_be_locked,
        mut gpio2: shared_resources::gpio2_that_needs_to_be_locked,
        mut matrix_in_pins: shared_resources::matrix_in_pins_that_needs_to_be_locked,
        mut kscan_scan_time: shared_resources::kscan_scan_time_that_needs_to_be_locked
    ) {
        use rtic::Mutex;
        // Disable our interrupts temporarily to avoid re-entry while we scan.
        gpio1.lock(|gpio1| {
            gpio2.lock(|gpio2| {
                matrix_in_pins.lock(|matrix_in_pins| {
                    disable_matrix_interrupts(gpio1, gpio2, matrix_in_pins);
                });
                unsafe {
                    ral::write_reg!(ral::gpio, ral::gpio::GPIO1, ISR, 0);
                    ral::write_reg!(ral::gpio, ral::gpio::GPIO2, ISR, 0);
                }
            })
        });
        kscan_scan_time.lock(|ksc| {
            *ksc = Systick::now();
        });

        kscan_matrix_read::spawn(5).unwrap(); // start polling for a bit to try to catch everything
    }

    #[task(binds = GPIO1_COMBINED_0_15, shared = [gpio1, gpio2, matrix_in_pins, kscan_scan_time])]
    fn interrupt_gpio1_0_15(cx: interrupt_gpio1_0_15::Context) {
        handle_gpio_interrupt(cx.shared.gpio1, cx.shared.gpio2, cx.shared.matrix_in_pins, cx.shared.kscan_scan_time);
    }

    #[task(binds = GPIO1_COMBINED_16_31, shared = [gpio1, gpio2, matrix_in_pins, kscan_scan_time])]
    fn interrupt_gpio1_16_31(cx: interrupt_gpio1_16_31::Context) {
        handle_gpio_interrupt(cx.shared.gpio1, cx.shared.gpio2, cx.shared.matrix_in_pins, cx.shared.kscan_scan_time);
    }

    #[task(binds = GPIO2_COMBINED_0_15, shared = [gpio1, gpio2, matrix_in_pins, kscan_scan_time])]
    fn interrupt_gpio2_0_15(cx: interrupt_gpio2_0_15::Context) {
        handle_gpio_interrupt(cx.shared.gpio1, cx.shared.gpio2, cx.shared.matrix_in_pins, cx.shared.kscan_scan_time);
    }

    #[task(binds = GPIO2_COMBINED_16_31, shared = [gpio1, gpio2, matrix_in_pins, kscan_scan_time])]
    fn interrupt_gpio2_16_31(cx: interrupt_gpio2_16_31::Context) {
        handle_gpio_interrupt(cx.shared.gpio1, cx.shared.gpio2, cx.shared.matrix_in_pins, cx.shared.kscan_scan_time);
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
