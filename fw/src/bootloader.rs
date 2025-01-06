use core::arch::asm;

use teensy4_bsp::ral;

pub fn reboot_into_bootloader() -> ! {
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