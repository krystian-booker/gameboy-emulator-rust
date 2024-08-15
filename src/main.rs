mod cpu;
mod memoryManager;

use cpu::CPU;

fn main() {
    // Initialize the CPU
    let mut cpu = CPU::new();

    // Set the initial Program Counter (PC) to 0x0100, where the Game Boy's ROM starts execution.
    cpu.pc = 0x0100;

    println!("Game Boy emulator initialized!");
}
