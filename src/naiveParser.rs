struct naiveParser {
    message: [u8; 64],
}

impl naiveParser {
    fn parse_all(&mut self, incoming_message: &[u8], callback: &mut ParserCallback) -> usize {
        let mut array_index = 0;
        while self.message[array_index] != 0 {
            array_index += 1;
        }
        let mut slice_index = 0;
        while incoming_message[slice_index] != b'\n' {
            self.message[array_index] = incoming_message[slice_index];
            slice_index += 1;
            array_index += 1;
        }
        slice_index += 1;
        naiveParser::parse_command(&self.message[0..array_index], callback);
        let mut index: usize = slice_index;
        for i in slice_index..incoming_message.len() {
            if incoming_message[i] == b'\n' {
                naiveParser::parse_command(&incoming_message[index..i], callback);
                index = i + 1;
            }
        }
        for i in 0..64 {
            self.message[i] = 0;
        }
        for i in 0..incoming_message.len() - index {
            self.message[i] = incoming_message[i + index];
        }
        return index;
    }

    fn parse_command(command: &[u8], callback: &mut ParserCallback) {
        if &command[..4] == b"SIZE" {
            callback.size();
            println!("Size");
        } else if &command[..4] == b"HELP" {
            callback.help();
        }
        if &command[..2] == b"PX" {
            let mut x_end_point = 3;
            while command[x_end_point] != b' ' {
                x_end_point += 1;
            }
            x_end_point -= 1;
            let mut multiplicator = 1;
            let mut x_value: u16 = 0;
            for j in 0..=(x_end_point - 3) {
                x_value += (command[x_end_point - j] - b'0') as u16 * multiplicator as u16;
                multiplicator *= 10;
            }
            let mut y_end_point = x_end_point + 2;
            while command[y_end_point] != b' ' {
                y_end_point += 1;
            }
            y_end_point -= 1;
            multiplicator = 1;
            let mut y_value: u16 = 0;
            for j in 0..=(y_end_point - (x_end_point + 2)) {
                y_value += (command[y_end_point - j] - b'0') as u16 * multiplicator as u16;
                multiplicator *= 10;
            }
            let color_start_point = y_end_point + 2;
            if command.len() - 6 == color_start_point {
                let red = naiveParser::get_color_value(
                    &command[color_start_point..color_start_point + 2],
                );
                let green = naiveParser::get_color_value(
                    &command[color_start_point + 2..color_start_point + 4],
                );
                let blue = naiveParser::get_color_value(
                    &command[color_start_point + 4..color_start_point + 6],
                );
                let color: Color = Color {
                    red: red,
                    blue: blue,
                    green: green,
                    alpha: 255,
                };
                callback.set(x_value, y_value, color.to_rgb888());
            } else if command.len() - 8 == color_start_point {
                let red = naiveParser::get_color_value(
                    &command[color_start_point..color_start_point + 2],
                );
                let green = naiveParser::get_color_value(
                    &command[color_start_point + 2..color_start_point + 4],
                );
                let blue = naiveParser::get_color_value(
                    &command[color_start_point + 4..color_start_point + 6],
                );
                let alpha = naiveParser::get_color_value(
                    &command[color_start_point + 6..color_start_point + 8],
                );
                let color: Color = Color {
                    red: red,
                    blue: blue,
                    green: green,
                    alpha: alpha,
                };
                callback.blend(x_value, y_value, color.to_rgba8888());
            }
        }
    }

    fn get_color_value(values: &[u8]) -> u8 {
        let mut mult = 1;
        let mut color = 0;
        for i in 0..=1 {
            match values[1 - i] {
                b'A'..=b'F' => color += (values[1 - i] - b'A' + 10) * mult,
                b'a'..=b'f' => color += (values[1 - i] - b'a' + 10) * mult,
                b'0'..=b'9' => color += (values[1 - i] - b'0') * mult,
                _ => {}
            }
            mult *= 16;
        }
        return color;
    }
}
