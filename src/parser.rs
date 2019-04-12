pub trait ParserCallback {
    fn size(&mut self);
    fn help(&mut self);
    fn set(&mut self, x: u16, y: u16, rgb: u32);
    fn blend(&mut self, x: u16, y: u16, rgba: u32);
}

#[derive(Copy, Clone, PartialEq)]
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

#[derive(Copy, Clone)]
pub struct Parser {
    state: State,
    x: u16,
    y: u16,
    color: u32,
}

impl Parser {
    pub fn new() -> Parser {
        Parser {
            state: State::Start,
            color: 8,
            x: 0,
            y: 0,
        }
    }

    fn reset(&mut self) -> State {
        *self = Parser::new();
        State::Start
    }

    pub fn parse_byte(&mut self, a: u8, cb: &mut ParserCallback) -> bool {
        use State::*;

        self.state = match (self.state, a) {
            (Start, b'P') => Px1,
            (Start, b'S') => Size1,
            (Start, b'H') => Help1,
            (Px1, b'X') => Px2,
            (Px2, b' ') => Px3,
            (Px3, _) => match a {
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
            (Px4, _) => match a {
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
            (Px5, _) => {
                let state_after_digit = if self.color >> 31 != 0 { Px7 } else { Px5 };
                match a {
                    b'0'..=b'9' => {
                        self.color = (self.color << 4) | (a - b'0' + 0x0) as u32;
                        state_after_digit
                    }
                    b'a'..=b'f' => {
                        self.color = (self.color << 4) | (a - b'a' + 0xa) as u32;
                        state_after_digit
                    }
                    b'A'..=b'F' => {
                        self.color = (self.color << 4) | (a - b'A' + 0xA) as u32;
                        state_after_digit
                    }
                    b'\r' => Px6,
                    b'\n' => {
                        if self.color >> 24 == 8 {
                            cb.set(self.x, self.y, self.color);
                            self.reset()
                        } else {
                            Invalid
                        }
                    }
                    _ => Invalid,
                }
            }
            (Px6, b'\n') => {
                cb.set(self.x, self.y, self.color);
                self.reset()
            }
            (Px7, b'\r') => Px8,
            (Px7, b'\n') => {
                cb.blend(self.x, self.y, self.color);
                self.reset()
            }
            (Px8, b'\n') => {
                cb.blend(self.x, self.y, self.color);
                self.reset()
            }
            (Help1, b'E') => Help2,
            (Help2, b'L') => Help3,
            (Help3, b'P') => Help4,
            (Help4, b'\n') => {
                cb.help();
                Start
            }
            (Help4, b'\r') => Help5,
            (Help5, b'\n') => {
                cb.help();
                Start
            }
            (Size1, b'I') => Size2,
            (Size2, b'Z') => Size3,
            (Size3, b'E') => Size4,
            (Size4, b'\n') => {
                cb.size();
                Start
            }
            (Size4, b'\r') => Size5,
            (Size5, b'\n') => {
                cb.size();
                Start
            }
            (Invalid, _) => {
                return false;
            }
            _ => Invalid,
        };

        self.state != Invalid
    }
}
