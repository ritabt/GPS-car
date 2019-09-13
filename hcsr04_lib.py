import board
from digitalio import DigitalInOut, DriveMode
from pulseio import PulseIn
import time


class HCSR04:
    """Control a HC-SR04 ultrasonic range sensor.

    Example use:

    ::

        with HCSR04(trig, echo) as sonar:
            try:
                while True:
                    print(sonar.dist_cm())
                    sleep(2)
            except KeyboardInterrupt:
                pass
    """
    def __init__(self, trig_pin, echo_pin, timeout_sec=.1):
        """
        :param trig_pin: The pin on the microcontroller that's connected to the
            ``Trig`` pin on the HC-SR04.
        :type trig_pin: str or microcontroller.Pin
        :param echo_pin: The pin on the microcontroller that's connected to the
            ``Echo`` pin on the HC-SR04.
        :type echo_pin: str or microcontroller.Pin
        :param float timeout_sec: Max seconds to wait for a response from the
            sensor before assuming it isn't going to answer. Should *not* be
            set to less than 0.05 seconds!
        """
        if isinstance(trig_pin, str):
            trig_pin = getattr(board, trig_pin)
        if isinstance(echo_pin, str):
            echo_pin = getattr(board, echo_pin)
        self.dist_cm = self._dist_two_wire
        self.timeout_sec = timeout_sec

        self.trig = DigitalInOut(trig_pin)
        self.trig.switch_to_output(value=False, drive_mode=DriveMode.PUSH_PULL)

        self.echo = PulseIn(echo_pin)
        self.echo.pause()
        self.echo.clear()

    def __enter__(self):
        """Allows for use in context managers."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Automatically de-initialize after a context manager."""
        self.deinit()

    def deinit(self):
        """De-initialize the trigger and echo pins."""
        self.trig.deinit()
        self.echo.deinit()


    def dist_cm(self):
        """Return the distance measured by the sensor in cm.

        This is the function that will be called most often in user code. The
        distance is calculated by timing a pulse from the sensor, indicating
        how long between when the sensor sent out an ultrasonic signal and when
        it bounced back and was received again.

        If no signal is received, the return value will be ``-1``. This means
        either the sensor was moving too fast to be pointing in the right
        direction to pick up the ultrasonic signal when it bounced back (less
        likely), or the object off of which the signal bounced is too far away
        for the sensor to handle. In my experience, the sensor can detect
        objects over 460 cm away.

        :return: Distance in centimeters.
        :rtype: float
        """
        # This method only exists to make it easier to document. See either
        # _dist_one_wire or _dist_two_wire for the actual implementation. One
        # of those two methods will be assigned to be used in place of this
        # method on instantiation.
        pass


    def _dist_two_wire(self):
        self.echo.clear()  # Discard any previous pulse values
        self.trig.value = 1  # Set trig high
        time.sleep(0.00001)  # 10 micro seconds 10/1000/1000
        self.trig.value = 0  # Set trig low
        timeout = time.monotonic()
        self.echo.resume()
        while len(self.echo) == 0:
            # Wait for a pulse
            if (time.monotonic() - timeout) > self.timeout_sec:
                self.echo.pause()
                return -1
        self.echo.pause()
        if self.echo[0] == 65535:
            return -1

        return (self.echo[0] / 2) / (291 / 10)



def test(trig, echo, delay=2):
    """Create and get distances from an :class:`HCSR04` object.

    This is meant to be helpful when first setting up the HC-SR04. It will get
    a distance every ``delay`` seconds and print it to standard out.

    :param trig: The pin on the microcontroller that's connected to the
        ``Trig`` pin on the HC-SR04.
    :type trig: str or microcontroller.Pin
    :param echo: The pin on the microcontroller that's connected to the
        ``Echo`` pin on the HC-SR04.
    :type echo: str or microcontroller.Pin
    :param delay: Seconds to wait between triggers.
    :type delay: int or float
    :rtype: None
    """
    with HCSR04(trig, echo) as sonar:
        try:
            while True:
                print(sonar.dist_cm())
                time.sleep(delay)
        except KeyboardInterrupt:
            pass