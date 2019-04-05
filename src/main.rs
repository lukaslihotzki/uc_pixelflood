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
    wire::{EthernetAddress, IpCidr, IpEndpoint, Ipv4Address},
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

    let bg_color = Color {
        red: 255,
        green: 0,
        blue: 0,
        alpha: 255,
    };
    let blue = Color {
        red: 0,
        green: 0,
        blue: 255,
        alpha: 255,
    };
    let green = Color {
        red: 0,
        green: 255,
        blue: 0,
        alpha: 255,
    };
    let red = Color {
        red: 255,
        green: 0,
        blue: 0,
        alpha: 255,
    };
    let black = Color {
        red: 0,
        green: 0,
        blue: 0,
        alpha: 255,
    };
    let grey = Color {
        red: 127,
        green: 127,
        blue: 127,
        alpha: 127,
    };

    // configures the system timer to trigger a SysTick exception every second
    init::init_systick(Hz(100), &mut systick, &rcc);
    systick.enable_interrupt();

    init::init_sdram(&mut rcc, &mut fmc);
    let mut lcd = init::init_lcd(&mut ltdc, &mut rcc);
    pins.display_enable.set(true);
    pins.backlight.set(true);

    let mut layer_1 = lcd.layer_1().unwrap();
    let mut layer_2 = lcd.layer_2().unwrap();

    layer_1.clear();
    layer_2.clear();
    lcd::init_stdout(layer_2);
    // 480 x 272

    unsafe { ALLOCATOR.init(rt::heap_start() as usize, HEAP_SIZE) }

    let mut i2c_3 = init::init_i2c_3(peripherals.I2C3, &mut rcc);
    i2c_3.test_1();
    i2c_3.test_2();

    nvic.enable(Interrupt::EXTI0);

    let mut sd = sd::Sd::new(&mut sdmmc, &mut rcc, &pins.sdcard_present);

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
        iface.update_ip_addrs(|ipa| {
            *(ipa.first_mut().unwrap()) =
                IpCidr::new(smoltcp::wire::IpAddress::v4(192, 168, 42, 2), 24)
        });

        println!("assigned {}", iface.ipv4_addr().unwrap());

        let mut parser = Parser {
            state: State::Start,
            color: 1,
            x: 0,
            y: 0,
        };

        // add new sockets
        let endpoint = IpEndpoint::new(iface.ipv4_addr().unwrap().into(), 1234);

        let udp_rx_buffer = UdpSocketBuffer::new(vec![UdpPacketMetadata::EMPTY; 3], vec![0u8; 256]);
        let udp_tx_buffer = UdpSocketBuffer::new(vec![UdpPacketMetadata::EMPTY; 1], vec![0u8; 128]);
        let mut example_udp_socket = UdpSocket::new(udp_rx_buffer, udp_tx_buffer);
        example_udp_socket.bind(endpoint).unwrap();
        sockets.add(example_udp_socket);

        let tcp_rx_buffer = TcpSocketBuffer::new(vec![0; ethernet::MTU]);
        let tcp_tx_buffer = TcpSocketBuffer::new(vec![0; ethernet::MTU]);
        let mut example_tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
        sockets.add(example_tcp_socket);

        loop {
            if let Ok((ref mut iface, ref mut prev_ip_addr)) = ethernet_interface {
                let timestamp = Instant::from_millis(system_clock::ms() as i64);
                for mut sock in sockets.iter_mut() {
                    if let Socket::Tcp(ref mut sockt) = *sock {
                        if sockt.state() == smoltcp::socket::TcpState::CloseWait {
                            sockt.close();
                        }
                        if sockt.state() == smoltcp::socket::TcpState::Closed {
                            sockt.listen(endpoint);
                            parser = Parser {
                                state: State::Start,
                                color: 1,
                                x: 0,
                                y: 0,
                            };
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
                                poll_socket(&mut socket, &mut parser).expect("socket poll failed");
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

enum State {
    Start = 0,
    Px1 = 1,
    Px2 = 2,
    Px3 = 3,
    Px4 = 4,
    Px5 = 5,
    Px6 = 6,
    Px7 = 7,
    Px8 = 8,
    Size1 = 9,
    Size2 = 10,
    Size3 = 11,
    Size4 = 12,
    Size5 = 13,
    Help1 = 14,
    Help2 = 15,
    Help3 = 16,
    Help4 = 17,
    Help5 = 18,
    Invalid = 31,
}

struct Parser {
    state: State,
    x: u16,
    y: u16,
    color: u32,
}

struct ParserCallback {
    reply: &'static [u8],
    layer: stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
}

impl ParserCallback {
    fn size(&mut self) {
        self.reply = b"SIZE 480 272\n"
    }

    fn help(&mut self) {
        self.reply = b"uc_pixelflood\n"
    }

    fn set(&mut self, x: u16, y: u16, rgb: u32) {
        self.layer
            .print_point_color_at(x as usize, y as usize, Color::from_rgb888(rgb));
    }

    fn blend(&mut self, x: u16, y: u16, rgba: u32) {
        self.layer
            .blend(x as usize, y as usize, Color::from_rgba8888(rgba));
    }
}

impl Parser {
    fn parse_byte(&mut self, a: u8, cb: &mut ParserCallback) {
        use State::*;

        self.state = match self.state {
            Start => match a {
                b'P' => Px1,
                b'S' => Size1,
                b'H' => Help1,
                _ => Invalid,
            },
            Px1 => match a {
                b'X' => Px2,
                _ => Invalid,
            },
            Px2 => match a {
                b' ' => Px3,
                _ => Invalid,
            },
            Px3 => match a {
                b'0'..=b'9' => {
                    self.x = self.x * 10 + (a - b'0') as u16;
                    if self.x < 480 {
                        Px3
                    } else {
                        Invalid
                    }
                }
                b' ' => Px4,
                _ => Invalid,
            },
            Px4 => match a {
                b'0'..=b'9' => {
                    self.y = self.y * 10 + (a - b'0') as u16;
                    if self.y < 272 {
                        Px4
                    } else {
                        Invalid
                    }
                }
                b' ' => Px5,
                _ => Invalid,
            },
            Px5 => {
                let overflow = self.color >> 28 == 1;
                match a {
                    b'0'..=b'9' => {
                        self.color = (self.color << 4) | (a - b'0' + 0x0) as u32;
                        if overflow {
                            Px7
                        } else {
                            Px5
                        }
                    }
                    b'a'..=b'f' => {
                        self.color = (self.color << 4) | (a - b'a' + 0xa) as u32;
                        if overflow {
                            Px7
                        } else {
                            Px5
                        }
                    }
                    b'A'..=b'F' => {
                        self.color = (self.color << 4) | (a - b'A' + 0xA) as u32;
                        if overflow {
                            Px7
                        } else {
                            Px5
                        }
                    }
                    b'\r' => Px6,
                    b'\n' => {
                        if self.color >> 24 == 1 {
                            cb.set(self.x, self.y, self.color);
                            Start
                        } else {
                            Invalid
                        }
                    }
                    _ => Invalid,
                }
            }
            Px6 => match a {
                b'\n' => {
                    cb.set(self.x, self.y, self.color);
                    Start
                }
                _ => Invalid,
            },
            Px7 => match a {
                b'\r' => Px8,
                b'\n' => {
                    cb.blend(self.x, self.y, self.color);
                    Start
                }
                _ => Invalid,
            },
            Px8 => match a {
                b'\n' => {
                    cb.blend(self.x, self.y, self.color);
                    Start
                }
                _ => Invalid,
            },
            Help1 => match a {
                b'E' => Help2,
                _ => Invalid,
            },
            Help2 => match a {
                b'L' => Help3,
                _ => Invalid,
            },
            Help3 => match a {
                b'P' => Help4,
                _ => Invalid,
            },
            Help4 => match a {
                b'\n' => {
                    cb.help();
                    Start
                }
                b'\r' => Help5,
                _ => Invalid,
            },
            Help5 => match a {
                b'\n' => {
                    cb.help();
                    Start
                }
                _ => Invalid,
            },
            Size1 => match a {
                b'I' => Size2,
                _ => Invalid,
            },
            Size2 => match a {
                b'Z' => Size3,
                _ => Invalid,
            },
            Size3 => match a {
                b'E' => Size4,
                _ => Invalid,
            },
            Size4 => match a {
                b'\n' => {
                    cb.size();
                    Start
                }
                b'\r' => Help5,
                _ => Invalid,
            },
            Size5 => match a {
                b'\n' => {
                    cb.size();
                    Start
                }
                _ => Invalid,
            },
            Invalid => return,
            _ => return,
        }
    }
}

fn poll_socket(
    socket: &mut Socket,
    parser: &mut Parser,
    layer: stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
) -> Result<(), smoltcp::Error> {
    match socket {
        &mut Socket::Tcp(ref mut socket) => match socket.local_endpoint().port {
            1234 => {
                if !socket.may_recv() {
                    return Ok(());
                }
                let reply = socket.recv(|data| {
                    if data.len() > 0 {
                        let mut cb = ParserCallback { reply: b"", layer };
                        for a in data.iter() {
                            parser.parse_byte(*a, &mut cb)
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

interrupt!(EXTI0, exti0, state: Option<HStdout> = None);

fn exti0(_state: &mut Option<HStdout>) {
    println!("Interrupt fired! This means that the button was pressed.");
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
