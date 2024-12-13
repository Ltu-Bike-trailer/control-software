pub struct TimeStruct {
    hours: u8,
    min: u8,
    sec: u8
}

impl TimeStruct {
    pub fn new(hours: u8, min: u8, sec:u8) -> Self {
        TimeStruct{
            hours: hours % 24,
            min: min % 60,
            sec: sec % 60,
        }
    }

    pub fn add_sec(&mut self){
        self.sec += 1;
        if self.sec >= 60 {
            self.sec = 0;
            self.min += 1;
            if self.min >= 60 {
                self.min = 0;
                self.hours = (self.hours + 1) % 24;
            }
        }
    }
    pub fn get_time(&self) -> (u8, u8, u8) {
        (self.hours, self.min, self.sec)
    }

}
