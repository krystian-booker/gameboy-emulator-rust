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
    pub ime: bool,            // Interrupt Master Enable
    pub interrupt_enable: u8, // IE Register
    pub interrupt_flag: u8,   // IF Register

    // Timing
    pub cycles: u64, // Cycle counter
}

// Flag bit constants
impl CPU {
    const FLAG_Z: u8 = 0b1000_0000; // Zero flag
    const FLAG_N: u8 = 0b0100_0000; // Subtract flag
    const FLAG_H: u8 = 0b0010_0000; // Half Carry flag
    const FLAG_C: u8 = 0b0001_0000; // Carry flag

    // Interrupt Flag and Enable Bits
    const INTERRUPT_VBLANK: u8 = 0b0000_0001;
    const INTERRUPT_LCD_STAT: u8 = 0b0000_0010;
    const INTERRUPT_TIMER: u8 = 0b0000_0100;
    const INTERRUPT_SERIAL: u8 = 0b0000_1000;
    const INTERRUPT_JOYPAD: u8 = 0b0001_0000;

    // Interrupt Vectors
    const VECTOR_VBLANK: u16 = 0x0040;
    const VECTOR_LCD_STAT: u16 = 0x0048;
    const VECTOR_TIMER: u16 = 0x0050;
    const VECTOR_SERIAL: u16 = 0x0058;
    const VECTOR_JOYPAD: u16 = 0x0060;
}

// Define the static lookup table for interrupts
static INTERRUPT_LOOKUP: &[(u8, u16)] = &[
    (CPU::INTERRUPT_VBLANK, CPU::VECTOR_VBLANK),
    (CPU::INTERRUPT_LCD_STAT, CPU::VECTOR_LCD_STAT),
    (CPU::INTERRUPT_TIMER, CPU::VECTOR_TIMER),
    (CPU::INTERRUPT_SERIAL, CPU::VECTOR_SERIAL),
    (CPU::INTERRUPT_JOYPAD, CPU::VECTOR_JOYPAD),
];

// Type alias for opcode handler functions
type OpcodeHandler<M> = fn(&mut CPU, &mut M);

// Implement Display trait for debugging purposes
impl fmt::Debug for CPU {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "AF: {:02X}{:02X} BC: {:02X}{:02X} DE: {:02X}{:02X} HL: {:02X}{:02X} SP: {:04X} PC: {:04X} IME: {} Cycles: {}",
            self.a, self.f,
            self.b, self.c,
            self.d, self.e,
            self.h, self.l,
            self.sp,
            self.pc,
            self.ime,
            self.cycles)
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
            interrupt_enable: 0x00,
            interrupt_flag: 0x00,
            cycles: 0,
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

    // Flag manipulation methods
    fn set_flag(&mut self, flag: u8) {
        self.f |= flag;
    }

    fn clear_flag(&mut self, flag: u8) {
        self.f &= !flag;
    }

    fn is_flag_set(&self, flag: u8) -> bool {
        self.f & flag != 0
    }

    fn set_flag_if(&mut self, flag: u8, condition: bool) {
        if condition {
            self.set_flag(flag);
        } else {
            self.clear_flag(flag);
        }
    }

    // Specific flag checks
    fn is_zero_flag_set(&self) -> bool {
        self.is_flag_set(Self::FLAG_Z)
    }

    fn is_subtract_flag_set(&self) -> bool {
        self.is_flag_set(Self::FLAG_N)
    }

    fn is_half_carry_flag_set(&self) -> bool {
        self.is_flag_set(Self::FLAG_H)
    }

    fn is_carry_flag_set(&self) -> bool {
        self.is_flag_set(Self::FLAG_C)
    }

    // Setting specific flags
    fn set_zero_flag(&mut self, condition: bool) {
        self.set_flag_if(Self::FLAG_Z, condition);
    }

    fn set_subtract_flag(&mut self, condition: bool) {
        self.set_flag_if(Self::FLAG_N, condition);
    }

    fn set_half_carry_flag(&mut self, condition: bool) {
        self.set_flag_if(Self::FLAG_H, condition);
    }

    fn set_carry_flag(&mut self, condition: bool) {
        self.set_flag_if(Self::FLAG_C, condition);
    }

    // Fetch next byte
    fn fetch_byte<M: Memory + ?Sized>(&mut self, memory: &mut M) -> u8 {
        let byte = memory.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        byte
    }

    // Fetch next word (2 bytes)
    fn fetch_word<M: Memory + ?Sized>(&mut self, memory: &mut M) -> u16 {
        let low = self.fetch_byte(memory) as u16;
        let high = self.fetch_byte(memory) as u16;
        (high << 8) | low
    }

    // Stack operations
    fn push_stack<M: Memory>(&mut self, memory: &mut M, value: u16) {
        self.sp = self.sp.wrapping_sub(1);
        memory.write_byte(self.sp, (value & 0xFF) as u8); // Push low byte first
        self.sp = self.sp.wrapping_sub(1);
        memory.write_byte(self.sp, (value >> 8) as u8); // Then push high byte
    }

    fn pop_stack<M: Memory>(&mut self, memory: &mut M) -> u16 {
        let low = memory.read_byte(self.sp) as u16;
        self.sp = self.sp.wrapping_add(1);
        let high = memory.read_byte(self.sp) as u16;
        self.sp = self.sp.wrapping_add(1);
        (high << 8) | low // Combine high and low bytes
    }

    // Increment the cycle counter by the specified number of cycles
    fn add_cycles(&mut self, cycles: u64) {
        self.cycles += cycles;
    }

    //Halt the CPU for a certain number of cycles
    pub fn cpu_halt<M: Memory>(&mut self, memory: &mut M, cycles: u64) {
        self.add_cycles(cycles);
        // During halt, interrupts can still be checked and potentially serviced
        while self.cycles > 0 {
            self.check_interrupts(memory);
            // Simulate passage of time by decrementing the cycles
            self.cycles -= 1;
        }
    }

    // Define a lookup table for opcode handlers
    const OPCODE_TABLE: [OpcodeHandler<dyn Memory>; 256] = {
        let mut table: [OpcodeHandler<dyn Memory>; 256] = [CPU::unimplemented; 256];
        table[0x00] = CPU::nop;
        table[0x01] = CPU::ld_bc_d16;
        table[0x06] = CPU::ld_b_n;
        table
    };

    // Execute a single instruction
    pub fn step<M: Memory + 'static>(&mut self, memory: &mut M) {
        self.check_interrupts(memory);
        let opcode = self.fetch_byte(memory);
        self.execute(opcode, memory);
    }

    // Check and handle interrupts
    fn check_interrupts<M: Memory>(&mut self, memory: &mut M) {
        if self.ime && (self.interrupt_flag & self.interrupt_enable) != 0 {
            for &(flag, vector) in INTERRUPT_LOOKUP {
                if self.interrupt_flag & flag != 0 {
                    self.handle_interrupt(memory, vector, flag);
                    break; // Handle only one interrupt per step
                }
            }
        }
    }

    // Handle an interrupt
    fn handle_interrupt<M: Memory>(&mut self, memory: &mut M, vector: u16, flag: u8) {
        self.ime = false; // Disable further interrupts
        self.interrupt_flag &= !flag; // Clear the interrupt flag
        self.push_stack(memory, self.pc); // Push current PC onto stack
        self.pc = vector; // Jump to the interrupt vector
        self.add_cycles(20); // Interrupt handling takes 20 cycles
    }

    // Method to execute an instruction based on the opcode
    fn execute<M: Memory + 'static>(&mut self, opcode: u8, memory: &mut M) {
        let handler = CPU::OPCODE_TABLE[opcode as usize];
        handler(self, memory);
    }

    // Instructions implementations
    fn nop<M: Memory + ?Sized>(&mut self, _memory: &mut M) {
        // No operation (do nothing)
        println!("NOP");
        self.add_cycles(4); // NOP takes 4 cycles
    }

    fn ld_bc_d16<M: Memory + ?Sized>(&mut self, memory: &mut M) {
        // LD BC, d16 : Load 16-bit immediate value into BC register
        let value = self.fetch_word(memory);
        self.set_bc(value);
        println!("LD BC, 0x{:04X}", value);
        self.add_cycles(12); // LD BC, d16 takes 12 cycles
    }

    fn ld_b_n<M: Memory + ?Sized>(&mut self, memory: &mut M) {
        // LD B, n : Load immediate 8-bit value into B register
        let value = self.fetch_byte(memory);
        self.b = value;
        println!("LD B, 0x{:02X}", value);
        self.add_cycles(8); // LD B, n takes 8 cycles
    }

    // Handle unimplemented opcodes
    fn unimplemented<M: Memory + ?Sized>(&mut self, _memory: &mut M) {
        panic!("Unimplemented opcode");
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
    fn test_get_af() {
        let mut cpu = CPU::new();

        // Set specific values to registers A and F
        cpu.a = 0x12;
        cpu.f = 0x34;

        // The combined 16-bit value should be 0x1234
        assert_eq!(cpu.get_af(), 0x1234);
    }

    #[test]
    fn test_get_bc() {
        let mut cpu = CPU::new();

        // Set specific values to registers B and C
        cpu.b = 0xAB;
        cpu.c = 0xCD;

        // The combined 16-bit value should be 0xABCD
        assert_eq!(cpu.get_bc(), 0xABCD);
    }

    #[test]
    fn test_get_de() {
        let mut cpu = CPU::new();

        // Set specific values to registers D and E
        cpu.d = 0x56;
        cpu.e = 0x78;

        // The combined 16-bit value should be 0x5678
        assert_eq!(cpu.get_de(), 0x5678);
    }

    #[test]
    fn test_get_hl() {
        let mut cpu = CPU::new();

        // Set specific values to registers H and L
        cpu.h = 0x9A;
        cpu.l = 0xBC;

        // The combined 16-bit value should be 0x9ABC
        assert_eq!(cpu.get_hl(), 0x9ABC);
    }

    #[test]
    fn test_set_af() {
        let mut cpu = CPU::new();

        // Set AF register pair to a 16-bit value
        cpu.set_af(0x1234);

        // Verify that A and F registers are set correctly
        assert_eq!(cpu.a, 0x12);
        assert_eq!(cpu.f, 0x30); // Lower 4 bits should be zero, so 0x34 becomes 0x30
    }

    #[test]
    fn test_set_bc() {
        let mut cpu = CPU::new();

        // Set BC register pair to a 16-bit value
        cpu.set_bc(0xABCD);

        // Verify that B and C registers are set correctly
        assert_eq!(cpu.b, 0xAB);
        assert_eq!(cpu.c, 0xCD);
    }

    #[test]
    fn test_set_de() {
        let mut cpu = CPU::new();

        // Set DE register pair to a 16-bit value
        cpu.set_de(0x5678);

        // Verify that D and E registers are set correctly
        assert_eq!(cpu.d, 0x56);
        assert_eq!(cpu.e, 0x78);
    }

    #[test]
    fn test_set_hl() {
        let mut cpu = CPU::new();

        // Set HL register pair to a 16-bit value
        cpu.set_hl(0x9ABC);

        // Verify that H and L registers are set correctly
        assert_eq!(cpu.h, 0x9A);
        assert_eq!(cpu.l, 0xBC);
    }

    #[test]
    fn test_set_flag() {
        let mut cpu = CPU::new();

        cpu.f = 0x00; // Start with no flags set

        // Set the Zero flag (0b1000_0000)
        cpu.set_flag(CPU::FLAG_Z);
        assert_eq!(cpu.f, 0x80); // Zero flag should be set

        // Set the Carry flag (0b0001_0000)
        cpu.set_flag(CPU::FLAG_C);
        assert_eq!(cpu.f, 0x90); // Both Zero and Carry flags should be set
    }

    #[test]
    fn test_clear_flag() {
        let mut cpu = CPU::new();

        cpu.f = 0xF0; // Start with all flags set (0b1111_0000)

        // Clear the Zero flag (0b1000_0000)
        cpu.clear_flag(CPU::FLAG_Z);
        assert_eq!(cpu.f, 0x70); // Zero flag should be cleared

        // Clear the Carry flag (0b0001_0000)
        cpu.clear_flag(CPU::FLAG_C);
        assert_eq!(cpu.f, 0x60); // Both Zero and Carry flags should be cleared
    }

    #[test]
    fn test_is_flag_set() {
        let mut cpu = CPU::new();

        cpu.f = 0x30; // Set some flags: 0b0101_0000 (Half Carry and Carry flags)

        // Check if the Zero flag is set
        assert_eq!(cpu.is_flag_set(CPU::FLAG_Z), false); // Zero flag should not be set

        // Check if the Half Carry flag is set
        assert_eq!(cpu.is_flag_set(CPU::FLAG_H), true); // Half Carry flag should be set

        // Check if the Carry flag is set
        assert_eq!(cpu.is_flag_set(CPU::FLAG_C), true); // Carry flag should be set
    }

    #[test]
    fn test_set_flag_if() {
        let mut cpu = CPU::new();

        cpu.f = 0x00; // Start with no flags set

        // Set the Zero flag if condition is true
        cpu.set_flag_if(CPU::FLAG_Z, true);
        assert_eq!(cpu.f, 0x80); // Zero flag should be set

        // Clear the Zero flag if condition is false
        cpu.set_flag_if(CPU::FLAG_Z, false);
        assert_eq!(cpu.f, 0x00); // Zero flag should be cleared

        // Set the Carry flag if condition is true
        cpu.set_flag_if(CPU::FLAG_C, true);
        assert_eq!(cpu.f, 0x10); // Carry flag should be set

        // Ensure the Carry flag remains set if condition is false
        cpu.set_flag_if(CPU::FLAG_C, false);
        assert_eq!(cpu.f, 0x00); // Carry flag should be cleared
    }

    #[test]
    fn test_is_zero_flag_set() {
        let mut cpu = CPU::new();

        cpu.f = 0x80; // Set the Zero flag (0b1000_0000)
        assert_eq!(cpu.is_zero_flag_set(), true); // Zero flag should be set

        cpu.f = 0x00; // Clear all flags
        assert_eq!(cpu.is_zero_flag_set(), false); // Zero flag should not be set
    }

    #[test]
    fn test_is_subtract_flag_set() {
        let mut cpu = CPU::new();

        cpu.f = 0x40; // Set the Subtract flag (0b0100_0000)
        assert_eq!(cpu.is_subtract_flag_set(), true); // Subtract flag should be set

        cpu.f = 0x00; // Clear all flags
        assert_eq!(cpu.is_subtract_flag_set(), false); // Subtract flag should not be set
    }

    #[test]
    fn test_is_half_carry_flag_set() {
        let mut cpu = CPU::new();

        cpu.f = 0x20; // Set the Half Carry flag (0b0010_0000)
        assert_eq!(cpu.is_half_carry_flag_set(), true); // Half Carry flag should be set

        cpu.f = 0x00; // Clear all flags
        assert_eq!(cpu.is_half_carry_flag_set(), false); // Half Carry flag should not be set
    }

    #[test]
    fn test_is_carry_flag_set() {
        let mut cpu = CPU::new();

        cpu.f = 0x10; // Set the Carry flag (0b0001_0000)
        assert_eq!(cpu.is_carry_flag_set(), true); // Carry flag should be set

        cpu.f = 0x00; // Clear all flags
        assert_eq!(cpu.is_carry_flag_set(), false); // Carry flag should not be set
    }

    #[test]
    fn test_set_zero_flag() {
        let mut cpu = CPU::new();

        // Set Zero flag when condition is true
        cpu.set_zero_flag(true);
        assert_eq!(cpu.is_zero_flag_set(), true); // Zero flag should be set

        // Clear Zero flag when condition is false
        cpu.set_zero_flag(false);
        assert_eq!(cpu.is_zero_flag_set(), false); // Zero flag should not be set
    }

    #[test]
    fn test_set_subtract_flag() {
        let mut cpu = CPU::new();

        // Set Subtract flag when condition is true
        cpu.set_subtract_flag(true);
        assert_eq!(cpu.is_subtract_flag_set(), true); // Subtract flag should be set

        // Clear Subtract flag when condition is false
        cpu.set_subtract_flag(false);
        assert_eq!(cpu.is_subtract_flag_set(), false); // Subtract flag should not be set
    }

    #[test]
    fn test_set_half_carry_flag() {
        let mut cpu = CPU::new();

        // Set Half Carry flag when condition is true
        cpu.set_half_carry_flag(true);
        assert_eq!(cpu.is_half_carry_flag_set(), true); // Half Carry flag should be set

        // Clear Half Carry flag when condition is false
        cpu.set_half_carry_flag(false);
        assert_eq!(cpu.is_half_carry_flag_set(), false); // Half Carry flag should not be set
    }

    #[test]
    fn test_set_carry_flag() {
        let mut cpu = CPU::new();

        // Set Carry flag when condition is true
        cpu.set_carry_flag(true);
        assert_eq!(cpu.is_carry_flag_set(), true); // Carry flag should be set

        // Clear Carry flag when condition is false
        cpu.set_carry_flag(false);
        assert_eq!(cpu.is_carry_flag_set(), false); // Carry flag should not be set
    }

    #[test]
    fn test_fetch_byte() {
        let mut cpu = CPU::new();
        let mut memory = TestMemory::new();

        // Load some bytes into memory at the program counter (PC) location
        memory.load(0x0100, &[0x12, 0x34, 0x56]);

        // Set the PC to 0x0100
        cpu.pc = 0x0100;

        // Fetch the first byte
        let byte1 = cpu.fetch_byte(&mut memory);
        assert_eq!(byte1, 0x12); // The first byte should be 0x12
        assert_eq!(cpu.pc, 0x0101); // PC should increment by 1

        // Fetch the next byte
        let byte2 = cpu.fetch_byte(&mut memory);
        assert_eq!(byte2, 0x34); // The second byte should be 0x34
        assert_eq!(cpu.pc, 0x0102); // PC should increment by 1 again

        // Fetch the next byte
        let byte3 = cpu.fetch_byte(&mut memory);
        assert_eq!(byte3, 0x56); // The third byte should be 0x56
        assert_eq!(cpu.pc, 0x0103); // PC should increment by 1 again
    }

    #[test]
    fn test_fetch_word() {
        let mut cpu = CPU::new();
        let mut memory = TestMemory::new();

        // Load a word (two bytes) into memory at the program counter (PC) location
        memory.load(0x0100, &[0x34, 0x12, 0x78, 0x56]);

        // Set the PC to 0x0100
        cpu.pc = 0x0100;

        // Fetch the first word (2 bytes)
        let word1 = cpu.fetch_word(&mut memory);
        assert_eq!(word1, 0x1234); // The first word should be 0x1234
        assert_eq!(cpu.pc, 0x0102); // PC should increment by 2

        // Fetch the next word (2 bytes)
        let word2 = cpu.fetch_word(&mut memory);
        assert_eq!(word2, 0x5678); // The second word should be 0x5678
        assert_eq!(cpu.pc, 0x0104); // PC should increment by 2 again
    }

    #[test]
    fn test_push_stack() {
        let mut cpu = CPU::new();
        let mut memory = TestMemory::new();

        cpu.sp = 0xFFFE; // Set SP to top of the stack
        let value_to_push = 0x1234;

        cpu.push_stack(&mut memory, value_to_push);

        // SP should now be decremented by 2
        assert_eq!(cpu.sp, 0xFFFC);

        // The value 0x1234 should be stored at memory[0xFFFC] and memory[0xFFFD]
        assert_eq!(memory.read_byte(0xFFFC), 0x12); // High byte
        assert_eq!(memory.read_byte(0xFFFD), 0x34); // Low byte
    }

    #[test]
    fn test_pop_stack() {
        let mut cpu = CPU::new();
        let mut memory = TestMemory::new();

        // Preload stack with a value in correct order
        cpu.sp = 0xFFFC; // Set SP to where we want to pop from
        memory.write_byte(0xFFFC, 0x34); // Low byte
        memory.write_byte(0xFFFD, 0x12); // High byte

        let popped_value = cpu.pop_stack(&mut memory);

        // SP should now be incremented by 2
        assert_eq!(cpu.sp, 0xFFFE);

        // The popped value should be 0x1234
        assert_eq!(popped_value, 0x1234);
    }

    #[test]
    fn test_add_cycles() {
        let mut cpu = CPU::new();

        // Ensure the initial cycle count is zero
        assert_eq!(cpu.cycles, 0);

        // Add 4 cycles
        cpu.add_cycles(4);
        assert_eq!(cpu.cycles, 4); // Cycle counter should now be 4

        // Add another 10 cycles
        cpu.add_cycles(10);
        assert_eq!(cpu.cycles, 14); // Cycle counter should now be 14

        // Add 100 cycles
        cpu.add_cycles(100);
        assert_eq!(cpu.cycles, 114); // Cycle counter should now be 114
    }

    #[test]
    fn test_cpu_initialization_and_execution() {
        let mut cpu = CPU::new();
        let mut memory = TestMemory::new();

        // Load a small program into memory:
        // 0x0100: LD BC, 0x1234 (0x01 0x34 0x12)
        // 0x0103: LD B, 0x42 (0x06 0x42)
        // 0x0105: NOP (0x00)
        let program = [
            0x01, 0x34, 0x12, // LD BC, 0x1234
            0x06, 0x42, // LD B, 0x42
            0x00, // NOP
        ];

        memory.load(0x0100, &program);

        // Execute the first instruction: LD BC, 0x1234
        cpu.step(&mut memory);
        assert_eq!(cpu.get_bc(), 0x1234);
        assert_eq!(cpu.pc, 0x0103);

        // Execute the second instruction: LD B, 0x42
        cpu.step(&mut memory);
        assert_eq!(cpu.b, 0x42);
        assert_eq!(cpu.pc, 0x0105);

        // Execute the third instruction: NOP
        cpu.step(&mut memory);
        assert_eq!(cpu.pc, 0x0106); // PC should increment by 1 after NOP
        assert_eq!(cpu.cycles, 24); // Ensure the correct number of cycles have passed

        // Ensure the CPU registers have the expected values after execution
        assert_eq!(cpu.get_bc(), 0x4234); // B should now be 0x42, C remains 0x34
    }

    #[test]
    fn test_check_interrupts() {
        let mut cpu = CPU::new();
        let mut memory = TestMemory::new();

        // Set up initial CPU state
        cpu.ime = true; // Enable interrupts
        cpu.interrupt_enable = CPU::INTERRUPT_VBLANK; // Enable VBLANK interrupt
        cpu.interrupt_flag = CPU::INTERRUPT_VBLANK; // Trigger the VBLANK interrupt
        cpu.pc = 0x0100; // Set PC to a known location

        // Execute check_interrupts, which should handle the VBLANK interrupt
        cpu.check_interrupts(&mut memory);

        // After handling the VBLANK interrupt, the following should have happened:
        // 1. IME should be disabled
        assert_eq!(cpu.ime, false);

        // 2. The interrupt flag for VBLANK should be cleared
        assert_eq!(cpu.interrupt_flag & CPU::INTERRUPT_VBLANK, 0);

        // 3. The current PC (0x0100) should have been pushed onto the stack
        let sp_after_push = cpu.sp;
        assert_eq!(memory.read_byte(sp_after_push), 0x01); // Higher byte of PC
        assert_eq!(memory.read_byte(sp_after_push.wrapping_add(1)), 0x00); // Lower byte of PC

        // 4. The PC should now point to the VBLANK interrupt vector (0x0040)
        assert_eq!(cpu.pc, CPU::VECTOR_VBLANK);

        // 5. Cycles should have been incremented by 20 (for handling the interrupt)
        assert_eq!(cpu.cycles, 20);
    }

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
