import wiringpi
wiringpi.wiringPiSetup()
wiringpi.pinMode(22,1)
wiringpi.pinMode(23,1)
wiringpi.pinMode(24,1)
wiringpi.pinMode(25,1)
while True:
    wiringpi.digitalWrite(22,1)
    wiringpi.digitalWrite(23,0)
    wiringpi.digitalWrite(24,1)
    wiringpi.digitalWrite(25,1)
