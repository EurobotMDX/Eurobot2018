from time import sleep
import time


class Encoders:
    def __init__(self):
        self.encoder1 = 0
        self.encoder2 = 0
        self.interval1 = 0.1
        self.interval2 = 0.12
        self.main_time = time.time()

        self.when1 = self.main_time
        self.when2 = self.main_time

        self.ran = self.main_time + 2
        self.encoder_threshold = 50
        # self.main_when = time.time()

    def add_encoders(self):
        if time.time() >= self.when1:
            self.encoder1 += 1
            self.when1 = time.time() + self.interval1

        if time.time() >= self.when2:
            self.encoder2 += 1
            self.when2 = time.time() + self.interval2

    def call_interrupt(self):
        if time.time() >= self.ran:
            self.ran = time.time() + 2
            return 102331
        else:
            return None

    def read_encoder1(self):

        c = self.call_interrupt()

        if c is not None:
            return c

        return self.encoder1

    def read_encoder2(self):
        return self.encoder2

    def main(self):

        e1_buffer = 0
        e2_buffer = 0

        main_interval = 0.2
        main_when = time.time()

        while True:

            if time.time() >= main_when:
                self.add_encoders()

                main_when = time.time() + main_interval

                enc1 = self.read_encoder1()
                enc2 = self.read_encoder2()

                if enc1 - e1_buffer > self.encoder_threshold:
                    # print("Error enc1: {}".format(enc1 - e1_buffer))
                    enc1 = e1_buffer
                    # print(" After fix Enc1: {} | Enc2: {}".format(enc1, enc2))

                e1_buffer = enc1

                print("Enc1: {} | Enc2: {}".format(enc1, enc2))

if __name__ == '__main__':
    Enc = Encoders()

    Enc.main()
