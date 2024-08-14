use std::fmt;

// Define a trait for memory operations
pub trait Memory {
    fn read_byte(&mut self, addr: u16) -> u8;
    fn write_byte(&mut self, addr: u16, value: u8);
}

// Define the CPU structure
pub struct CPU {
    // 8-bit Registers
    pub a: u8, // Accumulator
    pub f: u8, // Flags

    pub b: u8,
    pub c: u8,

    pub d: u8,
    pub e: u8,

    pub h: u8,
    pub l: u8,

    // 16-bit Registers
    pub sp: u16, // Stack Pointer
    pub pc: u16, // Program Counter

    // Internal state
    pub ime: bool, // Interrupt Master Enable
}

// Implement Display trait for debugging purposes
impl fmt::Debug for CPU {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "AF: {:02X}{:02X} BC: {:02X}{:02X} DE: {:02X}{:02X} HL: {:02X}{:02X} SP: {:04X} PC: {:04X} IME: {}",
            self.a, self.f,
            self.b, self.c,
            self.d, self.e,
            self.h, self.l,
            self.sp,
            self.pc,
            self.ime)
    }
}

// Implement CPU methods
impl CPU {
    pub fn new() -> CPU {
        CPU {
            a: 0x01,
            f: 0xB0,
            b: 0x00,
            c: 0x13,
            d: 0x00,
            e: 0xD8,
            h: 0x01,
            l: 0x4D,
            sp: 0xFFFE,
            pc: 0x0100,
            ime: false,
        }
    }

    // Combined 16-bit getters
    fn get_af(&self) -> u16 {
        ((self.a as u16) << 8) | (self.f as u16)
    }

    fn get_bc(&self) -> u16 {
        ((self.b as u16) << 8) | (self.c as u16)
    }

    fn get_de(&self) -> u16 {
        ((self.d as u16) << 8) | (self.e as u16)
    }

    fn get_hl(&self) -> u16 {
        ((self.h as u16) << 8) | (self.l as u16)
    }

    // Combined 16-bit setters
    fn set_af(&mut self, value: u16) {
        self.a = (value >> 8) as u8;
        self.f = (value & 0xF0) as u8; // Lower 4 bits of F are always zero
    }

    fn set_bc(&mut self, value: u16) {
        self.b = (value >> 8) as u8;
        self.c = (value & 0xFF) as u8;
    }

    fn set_de(&mut self, value: u16) {
        self.d = (value >> 8) as u8;
        self.e = (value & 0xFF) as u8;
    }

    fn set_hl(&mut self, value: u16) {
        self.h = (value >> 8) as u8;
        self.l = (value & 0xFF) as u8;
    }

    // Fetch next byte
    fn fetch_byte<M: Memory>(&mut self, memory: &mut M) -> u8 {
        let byte = memory.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        byte
    }

    // Fetch next word (2 bytes)
    fn fetch_word<M: Memory>(&mut self, memory: &mut M) -> u16 {
        let low = self.fetch_byte(memory) as u16;
        let high = self.fetch_byte(memory) as u16;
        (high << 8) | low
    }

    // Method to execute an instruction based on the opcode
    fn execute<M: Memory>(&mut self, opcode: u8, memory: &mut M) {
        match opcode {
            0x00 => self.nop(),
            0x01 => self.ld_bc_d16(memory),
            0x06 => self.ld_b_n(memory),
            _ => panic!("Unrecognized opcode: {:#X}", opcode),
        }
    }

    // Execute a single instruction
    pub fn step<M: Memory>(&mut self, memory: &mut M) {
        let opcode = self.fetch_byte(memory);
        self.execute(opcode, memory);
    }

    fn nop(&mut self) {
        // No operation (do nothing)
        println!("NOP");
    }

    fn ld_bc_d16<M: Memory>(&mut self, memory: &mut M) {
        // LD BC, d16 : Load 16-bit immediate value into BC register
        let value = self.fetch_word(memory);
        self.set_bc(value);
        println!("LD BC, 0x{:04X}", value);
    }

    fn ld_b_n<M: Memory>(&mut self, memory: &mut M) {
        // LD B, n : Load immediate 8-bit value into B register
        let value = self.fetch_byte(memory);
        self.b = value;
        println!("LD B, 0x{:02X}", value);
    }
}

// Define a simple memory structure for testing
struct TestMemory {
    data: [u8; 65536], // 64KB of memory
}

impl TestMemory {
    fn new() -> Self {
        TestMemory { data: [0; 65536] }
    }

    fn load(&mut self, start: u16, bytes: &[u8]) {
        let start = start as usize;
        let end = start + bytes.len();
        self.data[start..end].copy_from_slice(bytes);
    }
}

impl Memory for TestMemory {
    fn read_byte(&mut self, addr: u16) -> u8 {
        self.data[addr as usize]
    }

    fn write_byte(&mut self, addr: u16, value: u8) {
        self.data[addr as usize] = value;
    }
}

// Unit tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nop() {
        let mut cpu = CPU::new();
        let mut memory = TestMemory::new();

        memory.load(0x0100, &[0x00]); // Load NOP instruction at 0x0100
        cpu.step(&mut memory);

        // After executing NOP, PC should be incremented by 1
        assert_eq!(cpu.pc, 0x0101);
    }

    #[test]
    fn test_ld_bc_d16() {
        let mut cpu = CPU::new();
        let mut memory = TestMemory::new();

        memory.load(0x0100, &[0x01, 0x34, 0x12]); // LD BC, 0x1234
        cpu.step(&mut memory);

        // After executing LD BC, d16, BC should be set to 0x1234
        assert_eq!(cpu.get_bc(), 0x1234);
        assert_eq!(cpu.pc, 0x0103); // PC should be incremented by 3 (1 for opcode, 2 for immediate value)
    }

    #[test]
    fn test_ld_b_n() {
        let mut cpu = CPU::new();
        let mut memory = TestMemory::new();

        memory.load(0x0100, &[0x06, 0x42]); // LD B, 0x42
        cpu.step(&mut memory);

        // After executing LD B, n, B should be set to 0x42
        assert_eq!(cpu.b, 0x42);
        assert_eq!(cpu.pc, 0x0102); // PC should be incremented by 2 (1 for opcode, 1 for immediate value)
    }
}

