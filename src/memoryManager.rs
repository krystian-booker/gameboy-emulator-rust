use crate::cpu::Memory;

pub struct MemoryManager {
    // Memory regions
    rom: Vec<u8>,       // ROM, initially 32KB but can be banked
    vram: [u8; 0x2000], // 8KB Video RAM
    eram: Vec<u8>,      // External RAM (can be banked)
    wram: [u8; 0x2000], // 8KB Internal RAM
    oam: [u8; 0xA0],    // Object Attribute Memory (OAM)
    io: [u8; 0x80],     // I/O Registers
    hram: [u8; 0x7F],   // High RAM (HRAM)
    ie: u8,             // Interrupt Enable Register

    // MBC state
    rom_bank: usize,   // Current ROM bank (1 to n)
    ram_bank: usize,   // Current RAM bank (0 to n)
    ram_enabled: bool, // RAM enable flag
    mbc_type: MbcType, // The type of MBC used (if any)
}

impl MemoryManager {
    pub fn new(rom_data: Vec<u8>, eram_size: usize, mbc_type: MbcType) -> Self {
        let mut memory_manager = MemoryManager {
            rom: rom_data,
            vram: [0; 0x2000],
            eram: vec![0; eram_size],
            wram: [0; 0x2000],
            oam: [0; 0xA0],
            io: [0; 0x80],
            hram: [0; 0x7F],
            ie: 0,
            rom_bank: 1, // Start with bank 1 (bank 0 is fixed)
            ram_bank: 0, // Start with RAM bank 0
            ram_enabled: false,
            mbc_type,
        };

        // Initialize I/O registers according to Game Boy power-up values
        memory_manager.initialize_io_registers();

        // Return initialized memory_manager
        memory_manager
    }

    fn initialize_io_registers(&mut self) {
        // Initialize specific I/O registers with power-up values
        self.io[0x00] = 0xCF; // JOYP
        self.io[0x05] = 0x00; // TIMA
        self.io[0x06] = 0x00; // TMA
        self.io[0x07] = 0xF8; // TAC
        self.io[0x0F] = 0xE1; // IF
        self.io[0x10] = 0x80; // NR10
        self.io[0x11] = 0xBF; // NR11
        self.io[0x12] = 0xF3; // NR12
        self.io[0x14] = 0xBF; // NR14
        self.io[0x16] = 0x3F; // NR21
        self.io[0x17] = 0x00; // NR22
        self.io[0x19] = 0xBF; // NR24
        self.io[0x1A] = 0x7F; // NR30
        self.io[0x1B] = 0xFF; // NR31
        self.io[0x1C] = 0x9F; // NR32
        self.io[0x1E] = 0xBF; // NR34
        self.io[0x20] = 0xFF; // NR41
        self.io[0x21] = 0x00; // NR42
        self.io[0x22] = 0x00; // NR43
        self.io[0x23] = 0xBF; // NR44
        self.io[0x24] = 0x77; // NR50
        self.io[0x25] = 0xF3; // NR51
        self.io[0x26] = 0xF1; // NR52
        self.io[0x40] = 0x91; // LCDC
        self.io[0x41] = 0x85; // STAT
        self.io[0x42] = 0x00; // SCY
        self.io[0x43] = 0x00; // SCX
        self.io[0x45] = 0x00; // LYC
        self.io[0x47] = 0xFC; // BGP
        self.io[0x48] = 0xFF; // OBP0
        self.io[0x49] = 0xFF; // OBP1
        self.io[0x4A] = 0x00; // WY
        self.io[0x4B] = 0x00; // WX
        self.io[0xFF] = 0x00; // IE
    }

    // Helper function to map address to the correct memory region
    fn map_address(&self, addr: u16) -> (&[u8], usize) {
        match addr {
            0x0000..=0x7FFF => (&self.rom, addr as usize),
            0x8000..=0x9FFF => (&self.vram, (addr - 0x8000) as usize),
            0xA000..=0xBFFF => (&self.eram, (addr - 0xA000) as usize),
            0xC000..=0xDFFF => (&self.wram, (addr - 0xC000) as usize),
            0xE000..=0xFDFF => (&self.wram, (addr - 0xE000) as usize), // Echo RAM handling
            0xFE00..=0xFE9F => (&self.oam, (addr - 0xFE00) as usize),
            0xFF00..=0xFF7F => (&self.io, (addr - 0xFF00) as usize),
            0xFF80..=0xFFFE => (&self.hram, (addr - 0xFF80) as usize),
            0xFFFF => (std::slice::from_ref(&self.ie), 0),
            _ => panic!("Attempted to access unknown memory address: {:#06X}", addr),
        }
    }

    // Helper function to map address to the correct mutable memory region
    fn map_address_mut(&mut self, addr: u16) -> (&mut [u8], usize) {
        match addr {
            0x0000..=0x7FFF => (&mut self.rom, addr as usize),
            0x8000..=0x9FFF => (&mut self.vram, (addr - 0x8000) as usize),
            0xA000..=0xBFFF => (&mut self.eram, (addr - 0xA000) as usize),
            0xC000..=0xDFFF => (&mut self.wram, (addr - 0xC000) as usize),
            0xE000..=0xFDFF => (&mut self.wram, (addr - 0xE000) as usize), // Echo RAM handling
            0xFE00..=0xFE9F => (&mut self.oam, (addr - 0xFE00) as usize),
            0xFF00..=0xFF7F => (&mut self.io, (addr - 0xFF00) as usize),
            0xFF80..=0xFFFE => (&mut self.hram, (addr - 0xFF80) as usize),
            0xFFFF => (std::slice::from_mut(&mut self.ie), 0),
            _ => panic!("Attempted to access unknown memory address: {:#06X}", addr),
        }
    }

    // Handle bank switching for ROM and RAM based on MBC type
    fn handle_bank_switching(&mut self, addr: u16, value: u8) {
        match self.mbc_type {
            MbcType::MBC1 => self.handle_mbc1(addr, value),
            MbcType::MBC3 => self.handle_mbc3(addr, value),
            MbcType::MBC5 => self.handle_mbc5(addr, value),
            _ => {}
        }
    }

    // Handle MBC1 bank switching logic
    fn handle_mbc1(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x1FFF => {
                // Enable/disable RAM
                self.ram_enabled = value & 0x0F == 0x0A;
            }
            0x2000..=0x3FFF => {
                // Set lower 5 bits of ROM bank number
                let bank = value as usize & 0x1F;
                self.rom_bank = match bank {
                    0 => 1,
                    _ => bank,
                };
            }
            0x4000..=0x5FFF => {
                // Set RAM bank number or upper bits of ROM bank number
                self.ram_bank = (value as usize) & 0x03;
            }
            0x6000..=0x7FFF => {
                // ROM/RAM mode select (not implemented here)
            }
            _ => {}
        }
    }

    // Handle MBC3 bank switching logic
    fn handle_mbc3(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x1FFF => {
                // Enable/disable RAM and RTC
                self.ram_enabled = value & 0x0F == 0x0A;
            }
            0x2000..=0x3FFF => {
                // Set ROM bank number (7 bits)
                self.rom_bank = match value as usize & 0x7F {
                    0 => 1,
                    n => n,
                };
            }
            0x4000..=0x5FFF => {
                // Set RAM bank number or RTC register select
                self.ram_bank = value as usize & 0x03;
            }
            0x6000..=0x7FFF => {
                // Latch clock data (if RTC is implemented)
            }
            _ => {}
        }
    }

    // Handle MBC5 bank switching logic
    fn handle_mbc5(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x1FFF => {
                // Enable/disable RAM
                self.ram_enabled = value & 0x0F == 0x0A;
            }
            0x2000..=0x2FFF => {
                // Set lower 8 bits of ROM bank number
                self.rom_bank = (self.rom_bank & 0x100) | value as usize;
            }
            0x3000..=0x3FFF => {
                // Set upper 1 bit of ROM bank number
                self.rom_bank = (self.rom_bank & 0xFF) | ((value as usize & 0x01) << 8);
            }
            0x4000..=0x5FFF => {
                // Set RAM bank number
                self.ram_bank = value as usize & 0x0F;
            }
            _ => {}
        }
    }

    // Handle special I/O writes for specific registers
    fn handle_special_io_writes(&mut self, addr: u16, value: u8) {
        match addr {
            0xFF04 => {
                // Writing to DIV register resets it to 0
                self.io[0x04] = 0;
            }
            0xFF46 => {
                // Trigger DMA transfer
                self.dma_transfer(value);
            }
            _ => {
                // Standard I/O write
                let (region, offset) = self.map_address_mut(addr);
                region[offset] = value;
            }
        }
    }

    // Handle DMA transfer (OAM DMA)
    fn dma_transfer(&mut self, start_address: u8) {
        let src_base = (start_address as u16) << 8;
        for i in 0..0xA0 {
            let data = self.read_byte(src_base + i);
            self.oam[i as usize] = data;
        }
    }

    // Handle reading from memory-mapped I/O registers
    fn handle_io_read(&self, addr: u16) -> u8 {
        match addr {
            0xFF00 => {
                // Handle P1/JOYP register (joypad input)
                self.io[0x00] // Just an example, typically you would return the current state of the buttons
            }
            _ => {
                // Standard I/O read
                let (region, offset) = self.map_address(addr);
                region[offset]
            }
        }
    }

    pub fn read_byte(&self, addr: u16) -> u8 {
        match addr {
            // Unusable memory areas
            0xFEA0..=0xFEFF | 0xFF4C..=0xFF7F => 0xFF,

            // Regular memory mapping
            0x0000..=0x3FFF => self.rom[addr as usize], // ROM bank 0
            0x4000..=0x7FFF => self.rom[self.rom_bank * 0x4000 + (addr as usize - 0x4000)], // Switched ROM bank
            0xA000..=0xBFFF if self.ram_enabled => {
                self.eram[self.ram_bank * 0x2000 + (addr as usize - 0xA000)]
            } // Switched RAM bank
            0xFF00..=0xFF7F => self.handle_io_read(addr), // I/O registers
            _ => {
                let (region, offset) = self.map_address(addr);
                region[offset]
            }
        }
    }

    pub fn write_byte(&mut self, addr: u16, value: u8) {
        match addr {
            // Unusable memory areas
            0xFEA0..=0xFEFF | 0xFF4C..=0xFF7F => {
                panic!(
                    "Attempted to write to an unusable memory address: {:#06X}",
                    addr
                );
            }

            // Regular memory mapping
            0x0000..=0x7FFF => self.handle_bank_switching(addr, value), // Bank switching and RAM enable
            0xA000..=0xBFFF if self.ram_enabled => {
                let offset = self.ram_bank * 0x2000 + (addr as usize - 0xA000);
                self.eram[offset] = value;
            }
            0xFF00..=0xFF7F => self.handle_special_io_writes(addr, value), // Special I/O writes
            _ => {
                let (region, offset) = self.map_address_mut(addr);
                region[offset] = value;
            }
        }
    }
}

// Example integration with CPU
impl Memory for MemoryManager {
    fn read_byte(&mut self, addr: u16) -> u8 {
        self.read_byte(addr)
    }

    fn write_byte(&mut self, addr: u16, value: u8) {
        self.write_byte(addr, value)
    }
}

// Define a simple enum for MBC types
#[derive(Clone, Copy)]
pub enum MbcType {
    None,
    MBC1,
    MBC2,
    MBC3,
    MBC5,
}
