extern crate alloc;

use alloc::string::String;
use embassy_embedded_hal::adapter::BlockingAsync;
use esp_bootloader_esp_idf::partitions;
use esp_bootloader_esp_idf::partitions::{
    DataPartitionSubType, Error, PartitionEntry, PartitionType,
};
use esp_storage::FlashStorage;
use sequential_storage::cache::KeyPointerCache;
use sequential_storage::map::{MapConfig, MapStorage};

pub struct MyFlashRegion<'a, F> {
    pub offset: u32,
    pub size: usize,
    pub read_only: bool,
    pub flash: &'a mut F,
}

impl<'a, F> MyFlashRegion<'a, F> {
    pub fn new(partition_entry: &PartitionEntry, flash: &'a mut F) -> Self {
        Self {
            offset: partition_entry.offset(),
            size: partition_entry.len() as usize,
            read_only: partition_entry.is_read_only(),
            flash,
        }
    }
}

impl<F> MyFlashRegion<'_, F> {
    /// Returns the size of the partition in bytes.
    pub fn partition_size(&self) -> usize {
        self.size
    }

    fn range(&self) -> core::ops::Range<u32> {
        self.offset..self.offset + self.size as u32
    }

    fn in_range(&self, start: u32, len: usize) -> bool {
        self.range().contains(&start) && (start + len as u32 <= self.range().end)
    }
}

impl<F> embedded_storage::Region for MyFlashRegion<'_, F> {
    fn contains(&self, address: u32) -> bool {
        self.range().contains(&address)
    }
}

impl<F> embedded_storage::ReadStorage for MyFlashRegion<'_, F>
where
    F: embedded_storage::ReadStorage,
{
    type Error = Error;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let address = offset + self.offset;

        if !self.in_range(address, bytes.len()) {
            return Err(Error::OutOfBounds);
        }

        self.flash
            .read(address, bytes)
            .map_err(|_e| Error::StorageError)
    }

    fn capacity(&self) -> usize {
        self.partition_size()
    }
}

impl<F> embedded_storage::Storage for MyFlashRegion<'_, F>
where
    F: embedded_storage::Storage,
{
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let address = offset + self.offset;

        if self.read_only {
            return Err(Error::WriteProtected);
        }

        if !self.in_range(address, bytes.len()) {
            return Err(Error::OutOfBounds);
        }

        self.flash
            .write(address, bytes)
            .map_err(|_e| Error::StorageError)
    }
}

impl<F> embedded_storage::nor_flash::ErrorType for MyFlashRegion<'_, F> {
    type Error = Error;
}

impl<F> embedded_storage::nor_flash::ReadNorFlash for MyFlashRegion<'_, F>
where
    F: embedded_storage::nor_flash::ReadNorFlash,
{
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let address = offset + self.offset;

        if !self.in_range(address, bytes.len()) {
            return Err(Error::OutOfBounds);
        }

        self.flash
            .read(address, bytes)
            .map_err(|_e| Error::StorageError)
    }

    fn capacity(&self) -> usize {
        self.partition_size()
    }
}

impl<F> embedded_storage::nor_flash::NorFlash for MyFlashRegion<'_, F>
where
    F: embedded_storage::nor_flash::NorFlash,
{
    const WRITE_SIZE: usize = F::WRITE_SIZE;

    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let address_from = from + self.offset;
        let address_to = to + self.offset;

        if self.read_only {
            return Err(Error::WriteProtected);
        }

        if !self.range().contains(&address_from) {
            return Err(Error::OutOfBounds);
        }

        if !self.range().contains(&address_to) {
            return Err(Error::OutOfBounds);
        }

        self.flash
            .erase(address_from, address_to)
            .map_err(|_e| Error::StorageError)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let address = offset + self.offset;

        if self.read_only {
            return Err(Error::WriteProtected);
        }

        if !self.in_range(address, bytes.len()) {
            return Err(Error::OutOfBounds);
        }

        self.flash
            .write(address, bytes)
            .map_err(|_e| Error::StorageError)
    }
}

pub struct Storage<'a> {
    map_storage: MapStorage<
        u8,
        BlockingAsync<MyFlashRegion<'a, FlashStorage<'a>>>,
        KeyPointerCache<4, u8, 8>,
    >,
    scratch: [u8; 128],
}

impl<'a> Storage<'a> {
    pub fn new(flash: &'a mut FlashStorage<'a>) -> Option<Self> {
        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let partition_table = partitions::read_partition_table(flash, &mut pt_mem).unwrap();

        let mut partition_index = usize::MAX;

        for n in 0..partition_table.len() {
            let entry = partition_table.get_partition(n).unwrap();

            if entry.partition_type() == PartitionType::Data(DataPartitionSubType::Spiffs) {
                if entry.label_as_str() == "cfg_0" {
                    partition_index = n;
                }
            }
        }

        if partition_index < usize::MAX {
            match partition_table.get_partition(partition_index) {
                Ok(partition) => {
                    let region = MyFlashRegion::new(&partition, flash);
                    let storage = BlockingAsync::new(region);

                    const MAP_FLASH_RANGE: core::ops::Range<u32> = 0x000000..0x002000;

                    let map_storage = MapStorage::new(
                        storage,
                        const { MapConfig::new(MAP_FLASH_RANGE) },
                        KeyPointerCache::<4, u8, 8>::new(),
                    );
                    Some(Self {
                        map_storage,
                        scratch: [0; 128],
                    })
                }
                Err(_e) => None,
            }
        } else {
            None
        }
    }

    pub async fn get_string(&mut self, key: u8) -> Option<String> {
        match self
            .map_storage
            .fetch_item::<String>(&mut self.scratch, &key)
            .await
        {
            Ok(value) => value,
            Err(_) => None,
        }
    }

    pub async fn set_string(&mut self, key: u8, value: String) -> usize {
        match self
            .map_storage
            .store_item(&mut self.scratch, &key, &value)
            .await
        {
            Ok(_) => value.len(),
            Err(_) => 0,
        }
    }
}
