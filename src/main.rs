mod cpu;
mod memoryManager;

fn main() {
    // Initialize the emulator with a sample ROM data and memory bank configuration
    let rom_data = vec![0u8; 0x8000]; // Example ROM data
    let eram_size = 0x2000; // Example External RAM size
    let mbc_type = memoryManager::MbcType::None; // Example MBC type

    // let mut emulator = Emulator::new(rom_data, eram_size, mbc_type);

    // Set the initial Program Counter (PC) to 0x0100, where the Game Boy's ROM starts execution.
    // emulator.cpu.pc = 0x0100;

    println!("Game Boy emulator initialized!");

    // Run the emulator
    // emulator.run();
}
