use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};

use crate::{Partition, State, BOOT_MAGIC, SWAP_MAGIC};

/// Errors returned by bootloader
#[derive(PartialEq, Eq, Debug)]
pub enum BootError {
    /// Error from flash.
    Flash(NorFlashErrorKind),
    /// Invalid bootloader magic
    BadMagic,
}

#[cfg(feature = "defmt")]
impl defmt::Format for BootError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            BootError::Flash(_) => defmt::write!(fmt, "BootError::Flash(_)"),
            BootError::BadMagic => defmt::write!(fmt, "BootError::BadMagic"),
        }
    }
}

impl<E> From<E> for BootError
where
    E: NorFlashError,
{
    fn from(error: E) -> Self {
        BootError::Flash(error.kind())
    }
}

/// Extension of the embedded-storage flash type information with block size and erase value.
pub trait Flash: NorFlash + ReadNorFlash {
    /// The block size that should be used when writing to flash. For most builtin flashes, this is the same as the erase
    /// size of the flash, but for external QSPI flash modules, this can be lower.
    const BLOCK_SIZE: usize;
    /// The erase value of the flash. Typically the default of 0xFF is used, but some flashes use a different value.
    const ERASE_VALUE: u8 = 0xFF;
}

/// Trait defining the flash handles used for active and DFU partition
pub trait FlashConfig {
    /// Flash type used for the state partition.
    type STATE: Flash;
    /// Flash type used for the active partition.
    type ACTIVE: Flash;
    /// Flash type used for the dfu partition.
    type DFU: Flash;

    /// Return flash instance used to write/read to/from active partition.
    fn active(&mut self) -> &mut Self::ACTIVE;
    /// Return flash instance used to write/read to/from dfu partition.
    fn dfu(&mut self) -> &mut Self::DFU;
    /// Return flash instance used to write/read to/from bootloader state.
    fn state(&mut self) -> &mut Self::STATE;
}

/// BootLoader works with any flash implementing embedded_storage and can also work with
/// different page sizes and flash write sizes.
pub struct BootLoader {
    // Page with current state of bootloader. The state partition has the following format:
    // | Range          | Description                                                                      |
    // | 0 - WRITE_SIZE | Magic indicating bootloader state. BOOT_MAGIC means boot, SWAP_MAGIC means swap. |
    // | WRITE_SIZE - N | Progress index used while swapping or reverting                                  |
    state: Partition,
    // Location of the partition which will be booted from
    active: Partition,
    // Location of the partition which will be swapped in when requested
    dfu: Partition,
}

impl BootLoader {
    /// Create a new instance of a bootloader with the given partitions.
    ///
    /// - All partitions must be aligned with the PAGE_SIZE const generic parameter.
    /// - The dfu partition must be at least PAGE_SIZE bigger than the active partition.
    pub fn new(active: Partition, dfu: Partition, state: Partition) -> Self {
        Self { active, dfu, state }
    }

    /// Return the boot address for the active partition.
    pub fn boot_address(&self) -> usize {
        self.active.from
    }

    /// Perform necessary boot preparations like swapping images.
    ///
    /// The DFU partition is assumed to be 1 page bigger than the active partition for the swap
    /// algorithm to work correctly.
    ///
    /// SWAPPING
    ///
    /// Assume a flash size of 3 pages for the active partition, and 4 pages for the DFU partition.
    /// The swap index contains the copy progress, as to allow continuation of the copy process on
    /// power failure. The index counter is represented within 1 or more pages (depending on total
    /// flash size), where a page X is considered swapped if index at location (X + WRITE_SIZE)
    /// contains a zero value. This ensures that index updates can be performed atomically and
    /// avoid a situation where the wrong index value is set (page write size is "atomic").
    ///
    /// +-----------+------------+--------+--------+--------+--------+
    /// | Partition | Swap Index | Page 0 | Page 1 | Page 3 | Page 4 |
    /// +-----------+------------+--------+--------+--------+--------+
    /// |    Active |          0 |      1 |      2 |      3 |      - |
    /// |       DFU |          0 |      3 |      2 |      1 |      X |
    /// +-----------+------------+--------+--------+--------+--------+
    ///
    /// The algorithm starts by copying 'backwards', and after the first step, the layout is
    /// as follows:
    ///
    /// +-----------+------------+--------+--------+--------+--------+
    /// | Partition | Swap Index | Page 0 | Page 1 | Page 3 | Page 4 |
    /// +-----------+------------+--------+--------+--------+--------+
    /// |    Active |          1 |      1 |      2 |      1 |      - |
    /// |       DFU |          1 |      3 |      2 |      1 |      3 |
    /// +-----------+------------+--------+--------+--------+--------+
    ///
    /// The next iteration performs the same steps
    ///
    /// +-----------+------------+--------+--------+--------+--------+
    /// | Partition | Swap Index | Page 0 | Page 1 | Page 3 | Page 4 |
    /// +-----------+------------+--------+--------+--------+--------+
    /// |    Active |          2 |      1 |      2 |      1 |      - |
    /// |       DFU |          2 |      3 |      2 |      2 |      3 |
    /// +-----------+------------+--------+--------+--------+--------+
    ///
    /// And again until we're done
    ///
    /// +-----------+------------+--------+--------+--------+--------+
    /// | Partition | Swap Index | Page 0 | Page 1 | Page 3 | Page 4 |
    /// +-----------+------------+--------+--------+--------+--------+
    /// |    Active |          3 |      3 |      2 |      1 |      - |
    /// |       DFU |          3 |      3 |      1 |      2 |      3 |
    /// +-----------+------------+--------+--------+--------+--------+
    ///
    /// REVERTING
    ///
    /// The reverting algorithm uses the swap index to discover that images were swapped, but that
    /// the application failed to mark the boot successful. In this case, the revert algorithm will
    /// run.
    ///
    /// The revert index is located separately from the swap index, to ensure that revert can continue
    /// on power failure.
    ///
    /// The revert algorithm works forwards, by starting copying into the 'unused' DFU page at the start.
    ///
    /// +-----------+--------------+--------+--------+--------+--------+
    /// | Partition | Revert Index | Page 0 | Page 1 | Page 3 | Page 4 |
    //*/
    /// +-----------+--------------+--------+--------+--------+--------+
    /// |    Active |            3 |      1 |      2 |      1 |      - |
    /// |       DFU |            3 |      3 |      1 |      2 |      3 |
    /// +-----------+--------------+--------+--------+--------+--------+
    ///
    ///
    /// +-----------+--------------+--------+--------+--------+--------+
    /// | Partition | Revert Index | Page 0 | Page 1 | Page 3 | Page 4 |
    /// +-----------+--------------+--------+--------+--------+--------+
    /// |    Active |            3 |      1 |      2 |      1 |      - |
    /// |       DFU |            3 |      3 |      2 |      2 |      3 |
    /// +-----------+--------------+--------+--------+--------+--------+
    ///
    /// +-----------+--------------+--------+--------+--------+--------+
    /// | Partition | Revert Index | Page 0 | Page 1 | Page 3 | Page 4 |
    /// +-----------+--------------+--------+--------+--------+--------+
    /// |    Active |            3 |      1 |      2 |      3 |      - |
    /// |       DFU |            3 |      3 |      2 |      1 |      3 |
    /// +-----------+--------------+--------+--------+--------+--------+
    ///
    pub fn prepare_boot<P: FlashConfig>(
        &mut self,
        p: &mut P,
        magic: &mut [u8],
        page: &mut [u8],
    ) -> Result<State, BootError> {
        // Ensure we have enough progress pages to store copy progress
        assert_partitions(self.active, self.dfu, self.state, page.len(), P::STATE::WRITE_SIZE);
        assert_eq!(magic.len(), P::STATE::WRITE_SIZE);

        // Copy contents from partition N to active
        let state = self.read_state(p, magic)?;
        if state == State::Swap {
            //
            // Check if we already swapped. If we're in the swap state, this means we should revert
            // since the app has failed to mark boot as successful
            //
            if !self.is_swapped(p, magic, page)? {
                trace!("Swapping");
                self.swap(p, magic, page)?;
                trace!("Swapping done");
            } else {
                trace!("Reverting");
                self.revert(p, magic, page)?;

                // Overwrite magic and reset progress
                let fstate = p.state();
                magic.fill(!P::STATE::ERASE_VALUE);
                fstate.write(self.state.from as u32, magic)?;
                fstate.erase(self.state.from as u32, self.state.to as u32)?;

                magic.fill(BOOT_MAGIC);
                fstate.write(self.state.from as u32, magic)?;
            }
        }
        Ok(state)
    }

    fn is_swapped<P: FlashConfig>(&mut self, p: &mut P, magic: &mut [u8], page: &mut [u8]) -> Result<bool, BootError> {
        let page_size = page.len();
        let page_count = self.active.len() / page_size;
        let progress = self.current_progress(p, magic)?;

        Ok(progress >= page_count * 2)
    }

    fn current_progress<P: FlashConfig>(&mut self, config: &mut P, aligned: &mut [u8]) -> Result<usize, BootError> {
        let write_size = aligned.len();
        let max_index = ((self.state.len() - write_size) / write_size) - 1;
        aligned.fill(!P::STATE::ERASE_VALUE);

        let flash = config.state();
        for i in 0..max_index {
            flash.read((self.state.from + write_size + i * write_size) as u32, aligned)?;

            if aligned.iter().any(|&b| b == P::STATE::ERASE_VALUE) {
                return Ok(i);
            }
        }
        Ok(max_index)
    }

    fn update_progress<P: FlashConfig>(&mut self, idx: usize, p: &mut P, magic: &mut [u8]) -> Result<(), BootError> {
        let flash = p.state();
        let write_size = magic.len();
        let w = self.state.from + write_size + idx * write_size;

        let aligned = magic;
        aligned.fill(!P::STATE::ERASE_VALUE);
        flash.write(w as u32, aligned)?;
        Ok(())
    }

    fn active_addr(&self, n: usize, page_size: usize) -> usize {
        self.active.from + n * page_size
    }

    fn dfu_addr(&self, n: usize, page_size: usize) -> usize {
        self.dfu.from + n * page_size
    }

    fn copy_page_once_to_active<P: FlashConfig>(
        &mut self,
        idx: usize,
        from_page: usize,
        to_page: usize,
        p: &mut P,
        magic: &mut [u8],
        page: &mut [u8],
    ) -> Result<(), BootError> {
        let buf = page;
        if self.current_progress(p, magic)? <= idx {
            let mut offset = from_page;
            for chunk in buf.chunks_mut(P::DFU::BLOCK_SIZE) {
                p.dfu().read(offset as u32, chunk)?;
                offset += chunk.len();
            }

            p.active().erase(to_page as u32, (to_page + buf.len()) as u32)?;

            let mut offset = to_page;
            for chunk in buf.chunks(P::ACTIVE::BLOCK_SIZE) {
                p.active().write(offset as u32, chunk)?;
                offset += chunk.len();
            }
            self.update_progress(idx, p, magic)?;
        }
        Ok(())
    }

    fn copy_page_once_to_dfu<P: FlashConfig>(
        &mut self,
        idx: usize,
        from_page: usize,
        to_page: usize,
        p: &mut P,
        magic: &mut [u8],
        page: &mut [u8],
    ) -> Result<(), BootError> {
        let buf = page;
        if self.current_progress(p, magic)? <= idx {
            let mut offset = from_page;
            for chunk in buf.chunks_mut(P::ACTIVE::BLOCK_SIZE) {
                p.active().read(offset as u32, chunk)?;
                offset += chunk.len();
            }

            p.dfu().erase(to_page as u32, (to_page + buf.len()) as u32)?;

            let mut offset = to_page;
            for chunk in buf.chunks(P::DFU::BLOCK_SIZE) {
                p.dfu().write(offset as u32, chunk)?;
                offset += chunk.len();
            }
            self.update_progress(idx, p, magic)?;
        }
        Ok(())
    }

    fn swap<P: FlashConfig>(&mut self, p: &mut P, magic: &mut [u8], page: &mut [u8]) -> Result<(), BootError> {
        let page_size = page.len();
        let page_count = self.active.len() / page_size;
        trace!("Page count: {}", page_count);
        for page_num in 0..page_count {
            trace!("COPY PAGE {}", page_num);
            // Copy active page to the 'next' DFU page.
            let active_page = self.active_addr(page_count - 1 - page_num, page_size);
            let dfu_page = self.dfu_addr(page_count - page_num, page_size);
            //trace!("Copy active {} to dfu {}", active_page, dfu_page);
            self.copy_page_once_to_dfu(page_num * 2, active_page, dfu_page, p, magic, page)?;

            // Copy DFU page to the active page
            let active_page = self.active_addr(page_count - 1 - page_num, page_size);
            let dfu_page = self.dfu_addr(page_count - 1 - page_num, page_size);
            //trace!("Copy dfy {} to active {}", dfu_page, active_page);
            self.copy_page_once_to_active(page_num * 2 + 1, dfu_page, active_page, p, magic, page)?;
        }

        Ok(())
    }

    fn revert<P: FlashConfig>(&mut self, p: &mut P, magic: &mut [u8], page: &mut [u8]) -> Result<(), BootError> {
        let page_size = page.len();
        let page_count = self.active.len() / page_size;
        for page_num in 0..page_count {
            // Copy the bad active page to the DFU page
            let active_page = self.active_addr(page_num, page_size);
            let dfu_page = self.dfu_addr(page_num, page_size);
            self.copy_page_once_to_dfu(page_count * 2 + page_num * 2, active_page, dfu_page, p, magic, page)?;

            // Copy the DFU page back to the active page
            let active_page = self.active_addr(page_num, page_size);
            let dfu_page = self.dfu_addr(page_num + 1, page_size);
            self.copy_page_once_to_active(page_count * 2 + page_num * 2 + 1, dfu_page, active_page, p, magic, page)?;
        }

        Ok(())
    }

    fn read_state<P: FlashConfig>(&mut self, config: &mut P, magic: &mut [u8]) -> Result<State, BootError> {
        let flash = config.state();
        flash.read(self.state.from as u32, magic)?;

        if !magic.iter().any(|&b| b != SWAP_MAGIC) {
            Ok(State::Swap)
        } else {
            Ok(State::Boot)
        }
    }
}

fn assert_partitions(active: Partition, dfu: Partition, state: Partition, page_size: usize, write_size: usize) {
    assert_eq!(active.len() % page_size, 0);
    assert_eq!(dfu.len() % page_size, 0);
    assert!(dfu.len() - active.len() >= page_size);
    assert!(2 * (active.len() / page_size) <= (state.len() - write_size) / write_size);
}

/// A flash wrapper implementing the Flash and embedded_storage traits.
pub struct BootFlash<F, const BLOCK_SIZE: usize, const ERASE_VALUE: u8 = 0xFF>
where
    F: NorFlash + ReadNorFlash,
{
    flash: F,
}

impl<F, const BLOCK_SIZE: usize, const ERASE_VALUE: u8> BootFlash<F, BLOCK_SIZE, ERASE_VALUE>
where
    F: NorFlash + ReadNorFlash,
{
    /// Create a new instance of a bootable flash
    pub fn new(flash: F) -> Self {
        Self { flash }
    }
}

impl<F, const BLOCK_SIZE: usize, const ERASE_VALUE: u8> Flash for BootFlash<F, BLOCK_SIZE, ERASE_VALUE>
where
    F: NorFlash + ReadNorFlash,
{
    const BLOCK_SIZE: usize = BLOCK_SIZE;
    const ERASE_VALUE: u8 = ERASE_VALUE;
}

impl<F, const BLOCK_SIZE: usize, const ERASE_VALUE: u8> ErrorType for BootFlash<F, BLOCK_SIZE, ERASE_VALUE>
where
    F: ReadNorFlash + NorFlash,
{
    type Error = F::Error;
}

impl<F, const BLOCK_SIZE: usize, const ERASE_VALUE: u8> NorFlash for BootFlash<F, BLOCK_SIZE, ERASE_VALUE>
where
    F: ReadNorFlash + NorFlash,
{
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        F::erase(&mut self.flash, from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        F::write(&mut self.flash, offset, bytes)
    }
}

impl<F, const BLOCK_SIZE: usize, const ERASE_VALUE: u8> ReadNorFlash for BootFlash<F, BLOCK_SIZE, ERASE_VALUE>
where
    F: ReadNorFlash + NorFlash,
{
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        F::read(&mut self.flash, offset, bytes)
    }

    fn capacity(&self) -> usize {
        F::capacity(&self.flash)
    }
}

/// Convenience provider that uses a single flash for all partitions.
pub struct SingleFlashConfig<'a, F>
where
    F: Flash,
{
    flash: &'a mut F,
}

impl<'a, F> SingleFlashConfig<'a, F>
where
    F: Flash,
{
    /// Create a provider for a single flash.
    pub fn new(flash: &'a mut F) -> Self {
        Self { flash }
    }
}

impl<'a, F> FlashConfig for SingleFlashConfig<'a, F>
where
    F: Flash,
{
    type STATE = F;
    type ACTIVE = F;
    type DFU = F;

    fn active(&mut self) -> &mut Self::STATE {
        self.flash
    }
    fn dfu(&mut self) -> &mut Self::ACTIVE {
        self.flash
    }
    fn state(&mut self) -> &mut Self::DFU {
        self.flash
    }
}

/// Convenience flash provider that uses separate flash instances for each partition.
pub struct MultiFlashConfig<'a, ACTIVE, STATE, DFU>
where
    ACTIVE: Flash,
    STATE: Flash,
    DFU: Flash,
{
    active: &'a mut ACTIVE,
    state: &'a mut STATE,
    dfu: &'a mut DFU,
}

impl<'a, ACTIVE, STATE, DFU> MultiFlashConfig<'a, ACTIVE, STATE, DFU>
where
    ACTIVE: Flash,
    STATE: Flash,
    DFU: Flash,
{
    /// Create a new flash provider with separate configuration for all three partitions.
    pub fn new(active: &'a mut ACTIVE, state: &'a mut STATE, dfu: &'a mut DFU) -> Self {
        Self { active, state, dfu }
    }
}

impl<'a, ACTIVE, STATE, DFU> FlashConfig for MultiFlashConfig<'a, ACTIVE, STATE, DFU>
where
    ACTIVE: Flash,
    STATE: Flash,
    DFU: Flash,
{
    type STATE = STATE;
    type ACTIVE = ACTIVE;
    type DFU = DFU;

    fn active(&mut self) -> &mut Self::ACTIVE {
        self.active
    }
    fn dfu(&mut self) -> &mut Self::DFU {
        self.dfu
    }
    fn state(&mut self) -> &mut Self::STATE {
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn test_range_asserts() {
        const ACTIVE: Partition = Partition::new(4096, 4194304);
        const DFU: Partition = Partition::new(4194304, 2 * 4194304);
        const STATE: Partition = Partition::new(0, 4096);
        assert_partitions(ACTIVE, DFU, STATE, 4096, 4);
    }
}
