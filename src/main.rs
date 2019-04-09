#![feature(alloc)]
#![feature(alloc_error_handler)]
#![no_main]
#![no_std]

#[macro_use]
extern crate alloc;
extern crate alloc_cortex_m;
extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
#[macro_use]
extern crate stm32f7;
#[macro_use]
extern crate stm32f7_discovery;
extern crate smoltcp;

mod parser;
use parser::Parser;

use alloc::vec::Vec;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout as AllocLayout;
use core::fmt::Write;
use core::panic::PanicInfo;
use cortex_m::{asm, interrupt, peripheral::NVIC};
use rt::{entry, exception, ExceptionFrame};
use sh::hio::{self, HStdout};
use smoltcp::{
    dhcp::Dhcpv4Client,
    socket::{
        Socket, SocketSet, TcpSocket, TcpSocketBuffer, UdpPacketMetadata, UdpSocket,
        UdpSocketBuffer,
    },
    time::Instant,
    wire::{EthernetAddress, IpAddress, IpCidr, IpEndpoint, Ipv4Address},
};
use stm32f7::stm32f7x6::{CorePeripherals, Interrupt, Peripherals};
use stm32f7_discovery::{
    ethernet,
    gpio::{GpioPort, InputPin, OutputPin},
    init,
    lcd::AudioWriter,
    lcd::{self, Color},
    random::Rng,
    sd,
    system_clock::{self, Hz},
    touch,
};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

const HEAP_SIZE: usize = 50 * 1024; // in bytes
const ETH_ADDR: EthernetAddress = EthernetAddress([0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef]);

#[entry]
fn main() -> ! {
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut systick = core_peripherals.SYST;
    let mut nvic = core_peripherals.NVIC;

    let peripherals = Peripherals::take().unwrap();
    let mut rcc = peripherals.RCC;
    let mut pwr = peripherals.PWR;
    let mut flash = peripherals.FLASH;
    let mut fmc = peripherals.FMC;
    let mut ltdc = peripherals.LTDC;
    let mut sai_2 = peripherals.SAI2;
    let mut rng = peripherals.RNG;
    let mut sdmmc = peripherals.SDMMC1;
    let mut syscfg = peripherals.SYSCFG;
    let mut ethernet_mac = peripherals.ETHERNET_MAC;
    let mut ethernet_dma = peripherals.ETHERNET_DMA;

    init::init_system_clock_216mhz(&mut rcc, &mut pwr, &mut flash);
    init::enable_gpio_ports(&mut rcc);

    let gpio_a = GpioPort::new(peripherals.GPIOA);
    let gpio_b = GpioPort::new(peripherals.GPIOB);
    let gpio_c = GpioPort::new(peripherals.GPIOC);
    let gpio_d = GpioPort::new(peripherals.GPIOD);
    let gpio_e = GpioPort::new(peripherals.GPIOE);
    let gpio_f = GpioPort::new(peripherals.GPIOF);
    let gpio_g = GpioPort::new(peripherals.GPIOG);
    let gpio_h = GpioPort::new(peripherals.GPIOH);
    let gpio_i = GpioPort::new(peripherals.GPIOI);
    let gpio_j = GpioPort::new(peripherals.GPIOJ);
    let gpio_k = GpioPort::new(peripherals.GPIOK);
    let mut pins = init::pins(
        gpio_a, gpio_b, gpio_c, gpio_d, gpio_e, gpio_f, gpio_g, gpio_h, gpio_i, gpio_j, gpio_k,
    );

    // configures the system timer to trigger a SysTick exception every second
    init::init_systick(Hz(100), &mut systick, &rcc);
    systick.enable_interrupt();

    init::init_sdram(&mut rcc, &mut fmc);
    let mut lcd = init::init_lcd(&mut ltdc, &mut rcc);
    pins.display_enable.set(true);
    pins.backlight.set(true);

    let mut layer_1 = lcd.layer_1().unwrap();
    layer_1.clear();
    lcd::init_stdout(lcd.layer_1().unwrap());
    // 480 x 272

    unsafe { ALLOCATOR.init(rt::heap_start() as usize, HEAP_SIZE) }

    let mut i2c_3 = init::init_i2c_3(peripherals.I2C3, &mut rcc);
    i2c_3.test_1();
    i2c_3.test_2();

    init::init_sai_2(&mut sai_2, &mut rcc);
    init::init_wm8994(&mut i2c_3).expect("WM8994 init failed");
    // touch initialization should be done after audio initialization, because the touch
    // controller might not be ready yet
    touch::check_family_id(&mut i2c_3).unwrap();

    // ethernet
    let mut ethernet_interface = ethernet::EthernetDevice::new(
        Default::default(),
        Default::default(),
        &mut rcc,
        &mut syscfg,
        &mut ethernet_mac,
        &mut ethernet_dma,
        ETH_ADDR,
    )
    .map(|device| {
        let iface = device.into_interface();
        let prev_ip_addr = iface.ipv4_addr().unwrap();
        (iface, prev_ip_addr)
    });
    if let Err(e) = ethernet_interface {
        println!("ethernet init failed: {:?}", e);
    };

    let mut sockets = SocketSet::new(Vec::new());

    if let Ok((ref mut iface, ref mut prev_ip_addr)) = ethernet_interface {
        for &cidr in iface.ip_addrs().iter() {
            println!("assigned {}", cidr);
        }

        let mut parser = [Parser::new(); 8];

        for i in { 0..8 } {
            let tcp_rx_buffer = TcpSocketBuffer::new(vec![0; ethernet::MTU]);
            let tcp_tx_buffer = TcpSocketBuffer::new(vec![0; ethernet::MTU]);
            let mut example_tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
            sockets.add(example_tcp_socket);
        }

        loop {
            if let Ok((ref mut iface, ref mut prev_ip_addr)) = ethernet_interface {
                let timestamp = Instant::from_millis(system_clock::ms() as i64);
                for mut sock in sockets.iter_mut() {
                    if let Socket::Tcp(ref mut sockt) = *sock {
                        if sockt.state() == smoltcp::socket::TcpState::CloseWait {
                            sockt.close();
                        }
                        if sockt.state() == smoltcp::socket::TcpState::Closed {
                            sockt.listen(IpEndpoint::new(IpAddress::Unspecified, 1234));
                            parser[sockt.handle().get()] = Parser::new();
                        }
                    }
                }
                match iface.poll(&mut sockets, timestamp) {
                    Err(::smoltcp::Error::Exhausted) => {
                        continue;
                    }
                    Err(::smoltcp::Error::Unrecognized) => print!("U"),
                    Err(e) => println!("Network error: {:?}", e),
                    Ok(socket_changed) => {
                        if socket_changed {
                            for mut socket in sockets.iter_mut() {
                                poll_socket(&mut socket, &mut parser, &mut layer_1)
                                    .expect("socket poll failed");
                            }
                        }
                    }
                }
                iface.poll_delay(&sockets, timestamp);
            };
        }
    }

    loop {}
}

struct ParserCallback<'a> {
    reply: &'static [u8],
    layer: &'a mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
}

impl<'a> parser::ParserCallback for ParserCallback<'a> {
    fn size(&mut self) {
        self.reply = b"SIZE 480 272\n"
    }

    fn help(&mut self) {
        self.reply = b"Type PX x y rrggbb to display color rrggbb on Pixel x y.\n
						Type PX x y rrggbbaa to blend a color over the current color.\n
						Type SIZE to get the resolution of the Display\n
						Type HELP to get this help\n"
    }

    fn set(&mut self, x: u16, y: u16, rgb: u32) {
        self.layer
            .print_point_color_at(x as usize, y as usize, Color::from_rgb888(rgb & 0xffffff));
    }

    fn blend(&mut self, x: u16, y: u16, rgba: u32) {
        self.layer
            .blend(x as usize, y as usize, Color::from_rgba8888(rgba));
    }
}

fn poll_socket(
    socket: &mut Socket,
    parser: &mut [Parser],
    layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
) -> Result<(), smoltcp::Error> {
    match socket {
        &mut Socket::Tcp(ref mut socket) => match socket.local_endpoint().port {
            1234 => {
                if !socket.may_recv() {
                    return Ok(());
                }
                let p = &mut parser[socket.handle().get()];
                let reply = socket.recv(|data| {
                    if data.len() > 0 {
                        let mut cb = ParserCallback { reply: b"", layer };
                        for a in data.iter() {
                            p.parse_byte(*a, &mut cb)
                        }
                        (data.len(), cb.reply)
                    } else {
                        (data.len(), b"")
                    }
                })?;
                socket.send_slice(reply);
            }
            _ => {}
        },
        _ => {}
    }
    Ok(())
}

#[exception]
fn SysTick() {
    system_clock::tick();
    // print a `.` every 500ms
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn rust_oom(_: AllocLayout) -> ! {
    panic!("out of memory");
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    interrupt::disable();

    if lcd::stdout::is_initialized() {
        println!("{}", info);
    }

    if let Ok(mut hstdout) = hio::hstdout() {
        let _ = writeln!(hstdout, "{}", info);
    }

    // OK to fire a breakpoint here because we know the microcontroller is connected to a debugger
    asm::bkpt();

    loop {}
}
