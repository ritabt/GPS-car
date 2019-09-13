#import audiobusio
#import array

#mic = audiobusio.PDMIn(board.MICROPHONE_CLOCK, board.MICROPHONE_DATA, frequency=16000, bit_depth=16)
#samples = array.array('H', [0] * NUM_SAMPLES)

import time, board, digitalio
from analogio import AnalogIn

sd_cs        =   digitalio.DigitalInOut(board.xSDCS)
rts          =   digitalio.DigitalInOut(board.RTS)
dtr          =   digitalio.DigitalInOut(board.DTR)
sd_cs.direction        =  digitalio.Direction.OUTPUT
rts.direction          =  digitalio.Direction.OUTPUT
dtr.direction          =  digitalio.Direction.OUTPUT
analog_in              =  AnalogIn(board.A9)

sd_cs.value     = 1
rts.value       = 0
dtr.value       = 0

print(analog_in)