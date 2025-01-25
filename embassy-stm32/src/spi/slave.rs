use embassy_embedded_hal::SetConfig;
#[cfg(not(gpdma))]
use embassy_futures::join::join;
use embassy_hal_internal::{into_ref, PeripheralRef};
use embedded_hal_02::spi::{Mode, Phase, Polarity, MODE_0};

#[cfg(not(gpdma))]
use super::{set_rxdmaen, set_txdmaen, RxDma, TxDma, ChannelAndRequest};
use super::{
    word_impl, finish_dma, BitOrder, CsPin, Error, Info, Instance, MisoPin, MosiPin, RegsExt, SckPin,
    SealedWord, Word,
};
use crate::gpio::{AfType, AnyPin, OutputType, Pull, SealedPin as _, Speed};
use crate::pac::spi::vals;
use crate::{rcc, Peripheral};

/// SPI slave configuration.
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// SPI mode.
    pub mode: Mode,
    /// Bit order.
    pub bit_order: BitOrder,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: BitOrder::MsbFirst,
        }
    }
}

impl Config {
    fn raw_phase(&self) -> vals::Cpha {
        match self.mode.phase {
            Phase::CaptureOnSecondTransition => vals::Cpha::SECOND_EDGE,
            Phase::CaptureOnFirstTransition => vals::Cpha::FIRST_EDGE,
        }
    }

    fn raw_polarity(&self) -> vals::Cpol {
        match self.mode.polarity {
            Polarity::IdleHigh => vals::Cpol::IDLE_HIGH,
            Polarity::IdleLow => vals::Cpol::IDLE_LOW,
        }
    }

    fn raw_byte_order(&self) -> vals::Lsbfirst {
        match self.bit_order {
            BitOrder::LsbFirst => vals::Lsbfirst::LSBFIRST,
            BitOrder::MsbFirst => vals::Lsbfirst::MSBFIRST,
        }
    }
}

/// SPI slave driver.
///
/// This driver provides asynchronous DMA-driven read and write methods from user provided buffers.
pub struct SpiSlave<'d, T: Instance> {
    pub(crate) info: &'static Info,
    _peri: PeripheralRef<'d, T>,
    sck: Option<PeripheralRef<'d, AnyPin>>,
    mosi: Option<PeripheralRef<'d, AnyPin>>,
    miso: Option<PeripheralRef<'d, AnyPin>>,
    cs: Option<PeripheralRef<'d, AnyPin>>,
    tx_dma: Option<ChannelAndRequest<'d>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    current_word_size: word_impl::Config,
}

impl<'d, T: Instance> SpiSlave<'d, T> {
    /// Create a new SPI slave driver.
    pub fn new<Cs>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'd,
        cs: impl Peripheral<P = Cs> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Self
    where
        Cs: CsPin<T>,
    {
        into_ref!(peri, sck, mosi, miso, cs);

        sck.set_as_af(sck.af_num(), AfType::input(Pull::None));
        mosi.set_as_af(mosi.af_num(), AfType::input(Pull::None));
        miso.set_as_af(miso.af_num(), AfType::output(OutputType::PushPull, Speed::VeryHigh));
        cs.set_as_af(cs.af_num(), AfType::input(Pull::None));

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            Some(cs.map_into()),
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    fn new_inner(
        peri: impl Peripheral<P = T> + 'd,
        sck: Option<PeripheralRef<'d, AnyPin>>,
        mosi: Option<PeripheralRef<'d, AnyPin>>,
        miso: Option<PeripheralRef<'d, AnyPin>>,
        cs: Option<PeripheralRef<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        let cpha = config.raw_phase();
        let cpol = config.raw_polarity();

        let lsbfirst = config.raw_byte_order();

        rcc::enable_and_reset::<T>();

        let info = T::info();
        let regs = info.regs;

        #[cfg(any(spi_v1, spi_f1))]
        {
            regs.cr1().modify(|w| {
                w.set_cpha(cpha);
                w.set_cpol(cpol);

                w.set_mstr(vals::Mstr::SLAVE);
                w.set_ssm(false);

                w.set_lsbfirst(lsbfirst);
                w.set_crcen(false);
                w.set_bidimode(vals::Bidimode::UNIDIRECTIONAL);
                if miso.is_none() {
                    w.set_rxonly(vals::Rxonly::OUTPUTDISABLED);
                }
                w.set_dff(<u8 as SealedWord>::CONFIG)
            });
        }
        #[cfg(spi_v2)]
        {
            regs.cr2().modify(|w| {
                let (ds, frxth) = <u8 as SealedWord>::CONFIG;
                w.set_frxth(frxth);
                w.set_ds(ds);
            });
            regs.cr1().modify(|w| {
                w.set_cpha(cpha);
                w.set_cpol(cpol);

                w.set_mstr(vals::Mstr::SLAVE);
                w.set_ssm(false);

                w.set_lsbfirst(lsbfirst);
                w.set_crcen(false);
                w.set_bidimode(vals::Bidimode::UNIDIRECTIONAL);
            });
        }
        #[cfg(any(spi_v3, spi_v4, spi_v5))]
        {
            regs.ifcr().write(|w| w.0 = 0xffff_ffff);
            regs.cfg2().modify(|w| {
                w.set_cpha(cpha);
                w.set_cpol(cpol);
                w.set_lsbfirst(lsbfirst);

                w.set_master(vals::Master::SLAVE);
                w.set_ssm(false);

                w.set_comm(vals::Comm::FULL_DUPLEX);
                w.set_ssom(vals::Ssom::ASSERTED);
                w.set_midi(0);
                w.set_mssi(0);
                w.set_afcntr(true);
                w.set_ssiop(vals::Ssiop::ACTIVE_HIGH);
            });
            regs.cfg1().modify(|w| {
                w.set_crcen(false);
                w.set_dsize(<u8 as SealedWord>::CONFIG);
                w.set_fthlv(vals::Fthlv::ONE_FRAME);
                w.set_udrcfg(vals::Udrcfg::CONSTANT); // This is specific to my use case and should be configurable.
                w.set_udrdet(vals::Udrdet::END_OF_FRAME); // This is specific to my use case and should be configurable.
            });
            regs.cr2().modify(|w| {
                w.set_tsize(0);
            });
            regs.cr1().modify(|w| {
                w.set_ssi(false);
            });
            regs.udrdr().write(|w| w.set_udrdr(0x0000_0000)); // This is specific to my use case and should be configurable.
        }

        Self {
            info,
            _peri: peri,
            sck,
            mosi,
            miso,
            cs,
            tx_dma,
            rx_dma,
            current_word_size: <u8 as SealedWord>::CONFIG,
        }
    }

    /// Create a new SPI driver, in RX-only mode (only MISO pin, no MOSI).
    pub fn new_rxonly<Cs>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'd,
        cs: impl Peripheral<P = Cs> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Self 
    where
        Cs: CsPin<T>,
    {
        into_ref!(peri, sck, mosi, cs);

        sck.set_as_af(sck.af_num(), AfType::input(Pull::None));
        mosi.set_as_af(mosi.af_num(), AfType::input(Pull::None));
        cs.set_as_af(cs.af_num(), AfType::input(Pull::None));

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            None,
            Some(cs.map_into()),
            None,
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new SPI driver, in RX-only mode (only MISO pin, no MOSI).
    pub fn new_rxonly_no_cs(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, sck, mosi);

        sck.set_as_af(sck.af_num(), AfType::input(Pull::None));
        mosi.set_as_af(mosi.af_num(), AfType::input(Pull::None));

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            None,
            None,
            None,
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode (only MOSI pin, no MISO).
    pub fn new_txonly<Cs>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'd,
        cs: impl Peripheral<P = Cs> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        config: Config,
    ) -> Self 
    where
        Cs: CsPin<T>,
    {
        into_ref!(peri, sck, miso, cs);

        sck.set_as_af(sck.af_num(), AfType::input(Pull::None));
        miso.set_as_af(miso.af_num(), AfType::output(OutputType::PushPull, Speed::VeryHigh));
        cs.set_as_af(cs.af_num(), AfType::input(Pull::None));

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            None,
            Some(miso.map_into()),
            Some(cs.map_into()),
            new_dma!(tx_dma),
            None,
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode (only MOSI pin, no MISO).
    pub fn new_txonly_no_cs(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, sck, miso);

        sck.set_as_af(sck.af_num(), AfType::input(Pull::None));
        miso.set_as_af(miso.af_num(), AfType::output(OutputType::PushPull, Speed::VeryHigh));

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            None,
            Some(miso.map_into()),
            None,
            new_dma!(tx_dma),
            None,
            config,
        )
    }

    fn set_word_size(&mut self, word_size: word_impl::Config) {
        if self.current_word_size == word_size {
            return;
        }

        #[cfg(any(spi_v1, spi_f1))]
        {
            self.info.regs.cr1().modify(|reg| {
                reg.set_spe(false);
                reg.set_dff(word_size)
            });
            self.info.regs.cr1().modify(|reg| {
                reg.set_spe(true);
            });
        }
        #[cfg(spi_v2)]
        {
            self.info.regs.cr1().modify(|w| {
                w.set_spe(false);
            });
            self.info.regs.cr2().modify(|w| {
                w.set_frxth(word_size.1);
                w.set_ds(word_size.0);
            });
            self.info.regs.cr1().modify(|w| {
                w.set_spe(true);
            });
        }
        #[cfg(any(spi_v3, spi_v4, spi_v5))]
        {
            self.info.regs.cr1().modify(|w| {
                w.set_csusp(true);
            });
            while self.info.regs.sr().read().eot() {}
            self.info.regs.cr1().modify(|w| {
                w.set_spe(false);
            });
            self.info.regs.cfg1().modify(|w| {
                w.set_dsize(word_size);
            });
            self.info.regs.cr1().modify(|w| {
                w.set_csusp(false);
                w.set_spe(true);
            });
        }

        self.current_word_size = word_size;
    }

    /// SPI write, using DMA.
    pub async fn write<W: Word>(&mut self, data: &[W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        self.info.regs.cr1().modify(|w| {
            w.set_spe(false);
        });
        self.set_word_size(W::CONFIG);

        let tx_dst = self.info.regs.tx_ptr();
        let tx_f = unsafe { self.tx_dma.as_mut().unwrap().write(data, tx_dst, Default::default()) };

        set_txdmaen(self.info.regs, true);
        self.info.regs.cr1().modify(|w| {
            w.set_spe(true);
        });

        tx_f.await;

        finish_dma(self.info.regs);

        Ok(())
    }

    /// SPI read, using DMA.
    #[cfg(any(spi_v3, spi_v4, spi_v5))]
    pub async fn read<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        let regs = self.info.regs;

        regs.cr1().modify(|w| {
            w.set_spe(false);
        });

        self.set_word_size(W::CONFIG);

        let comm = regs.cfg2().modify(|w| {
            let prev = w.comm();
            w.set_comm(vals::Comm::RECEIVER);
            prev
        });

        #[cfg(spi_v3)]
        let i2scfg = regs.i2scfgr().modify(|w| {
            w.i2smod().then(|| {
                let prev = w.i2scfg();
                w.set_i2scfg(match prev {
                    vals::I2scfg::SLAVE_RX | vals::I2scfg::SLAVE_FULL_DUPLEX => vals::I2scfg::SLAVE_RX,
                    vals::I2scfg::MASTER_RX | vals::I2scfg::MASTER_FULL_DUPLEX => vals::I2scfg::MASTER_RX,
                    _ => panic!("unsupported configuration"),
                });
                prev
            })
        });

        let rx_src = regs.rx_ptr();

        for mut chunk in data.chunks_mut(u16::max_value().into()) {
            set_rxdmaen(regs, true);

            let tsize = chunk.len();

            let transfer = unsafe {
                self.rx_dma
                    .as_mut()
                    .unwrap()
                    .read(rx_src, &mut chunk, Default::default())
            };

            regs.cr2().modify(|w| {
                w.set_tsize(tsize as u16);
            });

            regs.cr1().modify(|w| {
                w.set_spe(true);
            });

            regs.cr1().modify(|w| {
                w.set_cstart(true);
            });

            transfer.await;

            finish_dma(regs);
        }

        regs.cr1().modify(|w| {
            w.set_spe(false);
        });

        regs.cfg2().modify(|w| {
            w.set_comm(comm);
        });

        regs.cr2().modify(|w| {
            w.set_tsize(0);
        });

        #[cfg(spi_v3)]
        if let Some(i2scfg) = i2scfg {
            regs.i2scfgr().modify(|w| {
                w.set_i2scfg(i2scfg);
            });
        }

        Ok(())
    }

    /// SPI read, using DMA.
    #[cfg(any(spi_v1, spi_f1, spi_v2))]
    pub async fn read<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        self.info.regs.cr1().modify(|w| {
            w.set_spe(false);
        });

        self.set_word_size(W::CONFIG);

        // SPIv3 clears rxfifo on SPE=0
        #[cfg(not(any(spi_v3, spi_v4, spi_v5)))]
        flush_rx_fifo(self.info.regs);

        set_rxdmaen(self.info.regs, true);

        let clock_byte_count = data.len();

        let rx_src = self.info.regs.rx_ptr();
        let rx_f = unsafe { self.rx_dma.as_mut().unwrap().read(rx_src, data, Default::default()) };

        let tx_dst = self.info.regs.tx_ptr();
        let clock_byte = W::default();
        let tx_f = unsafe {
            self.tx_dma
                .as_mut()
                .unwrap()
                .write_repeated(&clock_byte, clock_byte_count, tx_dst, Default::default())
        };

        set_txdmaen(self.info.regs, true);
        self.info.regs.cr1().modify(|w| {
            w.set_spe(true);
        });

        join(tx_f, rx_f).await;

        finish_dma(self.info.regs);

        Ok(())
    }

    async fn transfer_inner<W: Word>(&mut self, read: *mut [W], write: *const [W]) -> Result<(), Error> {
        assert_eq!(read.len(), write.len());
        if read.len() == 0 {
            return Ok(());
        }

        self.info.regs.cr1().modify(|w| {
            w.set_spe(false);
        });

        self.set_word_size(W::CONFIG);

        // SPIv3 clears rxfifo on SPE=0
        #[cfg(not(any(spi_v3, spi_v4, spi_v5)))]
        flush_rx_fifo(self.info.regs);

        set_rxdmaen(self.info.regs, true);

        let rx_src = self.info.regs.rx_ptr();
        let rx_f = unsafe { self.rx_dma.as_mut().unwrap().read_raw(rx_src, read, Default::default()) };

        let tx_dst = self.info.regs.tx_ptr();
        let tx_f = unsafe {
            self.tx_dma
                .as_mut()
                .unwrap()
                .write_raw(write, tx_dst, Default::default())
        };

        set_txdmaen(self.info.regs, true);
        self.info.regs.cr1().modify(|w| {
            w.set_spe(true);
        });
        #[cfg(any(spi_v3, spi_v4, spi_v5))]
        self.info.regs.cr1().modify(|w| {
            w.set_cstart(true);
        });

        join(tx_f, rx_f).await;

        finish_dma(self.info.regs);

        Ok(())
    }

    /// Bidirectional transfer, using DMA.
    ///
    /// This transfers both buffers at the same time, so it is NOT equivalent to `write` followed by `read`.
    ///
    /// The transfer runs for `max(read.len(), write.len())` bytes. If `read` is shorter extra bytes are ignored.
    /// If `write` is shorter it is padded with zero bytes.
    pub async fn transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        self.transfer_inner(read, write).await
    }

    /// In-place bidirectional transfer, using DMA.
    ///
    /// This writes the contents of `data` on MOSI, and puts the received data on MISO in `data`, at the same time.
    pub async fn transfer_in_place<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        self.transfer_inner(data, data).await
    }
}

impl<'d, T: Instance> Drop for SpiSlave<'d, T> {
    fn drop(&mut self) {
        self.sck.as_ref().map(|x| x.set_as_disconnected());
        self.mosi.as_ref().map(|x| x.set_as_disconnected());
        self.miso.as_ref().map(|x| x.set_as_disconnected());
        self.cs.as_ref().map(|x| x.set_as_disconnected());

        self.info.rcc.disable();
    }
}

impl<'d, T: Instance> SetConfig for SpiSlave<'d, T> {
    type Config = Config;
    type ConfigError = ();
    fn set_config(&mut self, _config: &Self::Config) -> Result<(), ()> {
        unimplemented!()
    }
}
