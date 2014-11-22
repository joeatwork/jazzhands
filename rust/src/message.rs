use std::fmt::{Show, Formatter, FormatError};

pub struct GloveState {
    pub serialno: i16,
    pub fingers: [i16, ..5],
    pub accel: [i16, ..3],
    pub gyro: [i16, ..3],
    pub mag: [i16, ..3],
}

impl Show for GloveState {
    fn fmt(&self, f: &mut Formatter) -> Result<(), FormatError> {
        write!(
            f,
            "GloveState {} [fingers {} {} {} {} {}] [accel {} {} {}] [gyro {} {} {}] [mag {} {} {}]",
            self.serialno,
            self.fingers[0], self.fingers[1], self.fingers[2], self.fingers[3], self.fingers[4],
            self.accel[0], self.accel[1], self.accel[2],
            self.gyro[0], self.gyro[1], self.gyro[2],
            self.mag[0], self.mag[1], self.mag[2]
        )
    }
}
