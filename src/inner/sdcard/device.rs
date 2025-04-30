//! SD card SPI device trait and provided implementations.

use core::{
    cell::RefCell,
    mem::{self, MaybeUninit},
};

use super::super::{
    super::{Operation, OutputPin, SpiBus},
    bisync, only_async, only_sync,
};

/// Trait for SD cards connected via SPI.
#[bisync]
pub trait SdCardSpiDevice {
    /// Perform a transaction against the device.
    ///
    /// This is similar to [`embedded_hal::spi::SpiDevice::transaction`], except that this sends
    /// a dummy `0xFF` byte to the device after deasserting the CS pin but before unlocking the
    /// bus.
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), SdCardDeviceError>;

    /// Send 80 clock pulses to the device with CS deasserted.
    async fn send_clock_pulses(&mut self) -> Result<(), SdCardDeviceError>;

    /// Do a read within a transaction.
    ///
    /// This is a convenience method equivalent to `device.transaction(&mut [Operation::Read(buf)])`.
    ///
    /// See also: [`SdCardDevice::transaction`], [`embedded_hal::spi::SpiBus::read`]
    #[inline]
    async fn read(&mut self, buf: &mut [u8]) -> Result<(), SdCardDeviceError> {
        self.transaction(&mut [Operation::Read(buf)]).await
    }

    /// Do a write within a transaction.
    ///
    /// This is a convenience method equivalent to `device.transaction(&mut [Operation::Write(buf)])`.
    ///
    /// See also: [`SdCardDevice::transaction`], [`embedded_hal::spi::SpiBus::write`]
    #[inline]
    async fn write(&mut self, buf: &[u8]) -> Result<(), SdCardDeviceError> {
        self.transaction(&mut [Operation::Write(buf)]).await
    }

    /// Do a transfer within a transaction.
    ///
    /// This is a convenience method equivalent to `device.transaction(&mut [Operation::Transfer(read, write)]`.
    ///
    /// See also: [`SdCardDevice::transaction`], [`embedded_hal::spi::SpiBus::transfer`]
    #[inline]
    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), SdCardDeviceError> {
        self.transaction(&mut [Operation::Transfer(read, write)])
            .await
    }

    /// Do an in-place transfer within a transaction.
    ///
    /// This is a convenience method equivalent to `device.transaction(&mut [Operation::TransferInPlace(buf)]`.
    ///
    /// See also: [`SdCardDevice::transaction`], [`embedded_hal::spi::SpiBus::transfer_in_place`]
    #[inline]
    async fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), SdCardDeviceError> {
        self.transaction(&mut [Operation::TransferInPlace(buf)])
            .await
    }
}

/// Errors that can occur when using the [`SdCardDevice`].
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-log", derive(defmt::Format))]
#[non_exhaustive]
pub enum SdCardDeviceError {
    /// An operation on the inner SPI bus failed.
    Spi,
    /// Setting the value of the Chip Select (CS) pin failed.
    Cs,
}

/// A wrapper around a SPI bus and a CS pin, using a `RefCell`.
///
/// This allows sharing the bus within the same thread.
pub struct RefCellSdCardDevice<'a, BUS, CS> {
    bus: &'a RefCell<BUS>,
    cs: CS,
}

impl<'a, BUS, CS> RefCellSdCardDevice<'a, BUS, CS> {
    /// Create a new `RefCellSdCardDevice`.
    pub fn new(bus: &'a RefCell<BUS>, cs: CS) -> Self {
        Self { bus, cs }
    }
}

#[bisync]
impl<BUS, CS> SdCardSpiDevice for RefCellSdCardDevice<'_, BUS, CS>
where
    BUS: SpiBus,
    CS: OutputPin,
{
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), SdCardDeviceError> {
        let mut bus = self.bus.borrow_mut();
        bus_transaction(&mut *bus, &mut self.cs, operations).await
    }

    async fn send_clock_pulses(&mut self) -> Result<(), SdCardDeviceError> {
        let mut bus = self.bus.borrow_mut();
        send_clock_pulses(&mut *bus, &mut self.cs).await
    }
}

#[cfg(feature = "embassy-sync-06")]
#[only_sync]
mod embassy_sync_06_blocking {
    use core::cell::RefCell;

    use ::embassy_sync_06::blocking_mutex;

    use super::*;

    /// A wrapper around a SPI bus and a CS pin, using an `embassy-sync` blocking mutex.
    ///
    /// This allows sharing the bus with according to the `embassy-sync` mutex model.
    /// See [`blocking_mutex::Mutex`] for more details.
    pub struct EmbassyMutexSdCardDevice<'a, BUS, CS, M> {
        bus: &'a blocking_mutex::Mutex<M, RefCell<BUS>>,
        cs: CS,
    }

    impl<'a, BUS, CS, M> EmbassyMutexSdCardDevice<'a, BUS, CS, M> {
        /// Create a new `EmbassyMutexSdCardDevice`.
        pub fn new(bus: &'a blocking_mutex::Mutex<M, RefCell<BUS>>, cs: CS) -> Self {
            Self { bus, cs }
        }
    }

    impl<CS, BUS, M> SdCardSpiDevice for EmbassyMutexSdCardDevice<'_, BUS, CS, M>
    where
        CS: OutputPin,
        BUS: SpiBus,
        M: blocking_mutex::raw::RawMutex,
    {
        fn transaction(
            &mut self,
            operations: &mut [Operation<'_, u8>],
        ) -> Result<(), SdCardDeviceError> {
            self.bus.lock(|bus| {
                let mut bus = bus.borrow_mut();
                bus_transaction(&mut *bus, &mut self.cs, operations)
            })
        }

        fn send_clock_pulses(&mut self) -> Result<(), SdCardDeviceError> {
            self.bus.lock(|bus| {
                let mut bus = bus.borrow_mut();
                send_clock_pulses(&mut *bus, &mut self.cs)
            })
        }
    }
}

#[cfg(feature = "embassy-sync-06")]
#[only_sync]
pub use self::embassy_sync_06_blocking::*;

#[cfg(feature = "embassy-sync-06")]
#[only_async]
mod embassy_sync_06_async {

    use ::embassy_sync_06::{blocking_mutex::raw::RawMutex, mutex};

    use super::*;

    /// A wrapper around a SPI bus and a CS pin, using an `embassy-sync` async mutex.
    ///
    /// This allows sharing the bus with according to the `embassy-sync` mutex model.
    /// See [`mutex::Mutex`] for more details.
    pub struct EmbassyMutexSdCardDevice<'a, BUS, CS, M: RawMutex> {
        bus: &'a mutex::Mutex<M, BUS>,
        cs: CS,
    }

    impl<'a, BUS, CS, M: RawMutex> EmbassyMutexSdCardDevice<'a, BUS, CS, M> {
        /// Create a new `EmbassyMutexSdCardDevice`.
        pub fn new(bus: &'a mutex::Mutex<M, BUS>, cs: CS) -> Self {
            Self { bus, cs }
        }
    }

    impl<CS, BUS, M> SdCardSpiDevice for EmbassyMutexSdCardDevice<'_, BUS, CS, M>
    where
        CS: OutputPin,
        BUS: SpiBus,
        M: RawMutex,
    {
        async fn transaction(
            &mut self,
            operations: &mut [Operation<'_, u8>],
        ) -> Result<(), SdCardDeviceError> {
            let mut bus = self.bus.lock().await;
            bus_transaction(&mut *bus, &mut self.cs, operations).await
        }

        async fn send_clock_pulses(&mut self) -> Result<(), SdCardDeviceError> {
            let mut bus = self.bus.lock().await;
            send_clock_pulses(&mut *bus, &mut self.cs).await
        }
    }
}

#[cfg(feature = "embassy-sync-06")]
#[only_async]
pub use self::embassy_sync_06_async::*;

#[cfg(feature = "embedded-hal-bus-03")]
#[only_sync]
mod embedded_hal_bus_03 {
    use ::embedded_hal_bus_03::spi::ExclusiveDevice;
    use embedded_hal::spi::SpiDevice;

    use super::*;

    // `ExclusiveDevice` represents exclusive access to the bus so there's no need to send the dummy
    // byte after deasserting the CS pin. We can delegate the implementation to the `embedded_hal` trait.

    impl<CS, BUS, D> SdCardSpiDevice for ExclusiveDevice<BUS, CS, D>
    where
        BUS: SpiBus,
        CS: OutputPin,
        D: embedded_hal::delay::DelayNs,
    {
        fn transaction(
            &mut self,
            operations: &mut [Operation<'_, u8>],
        ) -> Result<(), SdCardDeviceError> {
            <Self as SpiDevice>::transaction(self, operations).map_err(|_| SdCardDeviceError::Spi)
        }

        fn send_clock_pulses(&mut self) -> Result<(), SdCardDeviceError> {
            let bus = self.bus_mut();

            // There's no way to access the CS pin here so we can't set it high. Most likely it is already high so this is probably fine(?)

            let send_res = bus.write(&[0xFF; 10]);

            // On failure, it's important to still flush.
            let flush_res = bus.flush().map_err(|_| SdCardDeviceError::Spi);

            send_res.map_err(|_| SdCardDeviceError::Spi)?;
            flush_res.map_err(|_| SdCardDeviceError::Spi)?;
            Ok(())
        }
    }
}

/// Perform a transaction against the device. This sends a dummy `0xFF` byte to the device after
/// deasserting the CS pin but before unlocking the bus.
///
/// This function can be used to implement [`SdCardSpiDevice::transaction`] for your own types.
#[bisync]
pub async fn bus_transaction<BUS, CS>(
    bus: &mut BUS,
    cs: &mut CS,
    operations: &mut [Operation<'_, u8>],
) -> Result<(), SdCardDeviceError>
where
    BUS: SpiBus,
    CS: OutputPin,
{
    cs.set_low().map_err(|_| SdCardDeviceError::Cs)?;

    let cs_drop = OnDrop::new(|| {
        // This drop guard deasserts CS pin if the async operation is cancelled.
        // Errors are ignored in this drop handler, as there's nothing we can do about them.
        // If the async operation is completed without cancellation, this handler will not
        // be run, and the CS pin will be deasserted with proper error handling.
        let _ = cs.set_high();
    });

    let op_res = 'ops: {
        for op in operations {
            let res = match op {
                Operation::Read(buf) => bus.read(buf).await,
                Operation::Write(buf) => bus.write(buf).await,
                Operation::Transfer(read, write) => bus.transfer(read, write).await,
                Operation::TransferInPlace(buf) => bus.transfer_in_place(buf).await,
                Operation::DelayNs(_) => {
                    // We don't use delays in SPI transations in this crate so it fine to panic here.
                    panic!("Tried to use a delay in a SPI transaction. This is a bug in embedded-sdmmc.")
                }
            };
            if let Err(e) = res {
                break 'ops Err(e);
            }
        }
        Ok(())
    };

    // On failure, it's important to still flush and deassert CS.
    let flush_res = bus.flush().await;

    // Now that all the async operations that need CS low are done, we defuse
    // the CS guard, and manually set the CS pin high (to better handle the
    // possible errors).
    cs_drop.defuse();
    let cs_res = cs.set_high();

    op_res.map_err(|_| SdCardDeviceError::Spi)?;
    flush_res.map_err(|_| SdCardDeviceError::Spi)?;
    cs_res.map_err(|_| SdCardDeviceError::Cs)?;

    // Write the dummy byte
    let dummy_res = bus.write(&[0xFF]).await;
    let flush_res = bus.flush().await;

    dummy_res.map_err(|_| SdCardDeviceError::Spi)?;
    flush_res.map_err(|_| SdCardDeviceError::Spi)?;

    Ok(())
}

/// Send 80 clock pulses to the device with CS deasserted. This is needed to initialize the SD card.
///
/// This function can be used to implement [`SdCardSpiDevice::send_clock_pulses`] for your own types.
#[bisync]
pub async fn send_clock_pulses<BUS, CS>(bus: &mut BUS, cs: &mut CS) -> Result<(), SdCardDeviceError>
where
    BUS: SpiBus,
    CS: OutputPin,
{
    cs.set_high().map_err(|_| SdCardDeviceError::Cs)?;
    let send_res = bus.write(&[0xFF; 10]).await;

    // On failure, it's important to still flush.
    let flush_res = bus.flush().await;

    send_res.map_err(|_| SdCardDeviceError::Spi)?;
    flush_res.map_err(|_| SdCardDeviceError::Spi)?;

    Ok(())
}

/// A type to delay the drop handler invocation.
#[must_use = "to delay the drop handler invocation to the end of the scope"]
struct OnDrop<F: FnOnce()> {
    f: MaybeUninit<F>,
}

impl<F: FnOnce()> OnDrop<F> {
    /// Create a new instance.
    fn new(f: F) -> Self {
        Self {
            f: MaybeUninit::new(f),
        }
    }

    /// Prevent drop handler from running.
    fn defuse(self) {
        mem::forget(self)
    }
}

impl<F: FnOnce()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        unsafe { self.f.as_ptr().read()() }
    }
}
