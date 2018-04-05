

def setAngle(channel, angle, delta=170):
    """
        Sets angle of servo (approximate).
        :param channel: Channel servo is attached to (0-15)
        :param angle:   off of center, ( -80 through 80)
        :param delta:   Angle changed from past position. Used to calculate
                        delay, since rotating thru a larger arc takes longer
                        than a shorter arc.
    """
    delay = max(delta * 0.003, 0.03)        # calculate delay
    zero_pulse = (servoMin + servoMax) / 2  # half-way == 0 degrees
    pulse_width = zero_pulse - servoMin     # maximum pulse to either side 
    pulse = zero_pulse + (pulse_width * angle / 80)
    print "angle=%s pulse=%s" % (angle, pulse)
    pwm.setPWM(channel, 0, int(pulse))
    time.sleep(delay)  # sleep to give the servo time to do its thing
