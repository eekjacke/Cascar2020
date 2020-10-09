# README

## Bygga och ladda upp kod
För att bygga och ladda upp kod på bilen så måste det ställas in:
* Board: "Arduino Pro or Pro Mini"
* Processor: ATmega328P (5V, 16 MHz)


## Meddelanden styrning

### Trottel
Skicka meddelanden
```
T;xxx
```
där ```xxx``` är ett tal mellan -100 och 100. Strängen avslutas med carriage return (```\r```).

### Styrning
Skicka meddelanden
```
S;xxx
```
där ```xxx``` är ett tal mellan -100 och 100. Strängen avslutas med carriage return (```\r```).

## Meddelanden odometri
Båda bakhjulen är bestyckade med 10 magneter och en hallsensor. Varje gång en magnet passerar vänster sensor skickas ett meddelande på seriebussen
```
L;xxx
```
där ```xxx``` är microsekunder från förra läsningen. Strängen avslutas med carriage return (```\r```). Meddelandena för höger bakhjul ser lika ut fast med ```R``` istället för ```L```.


## Lågnivå, PWM-> throttle/steering
Arduinon sätter register ```OCR1A``` för att ge ut gaspådrag. Nollnivå är 1.5 msek vilket motsvarar ```OCR1A = 3000```, maxpådrag framåt är 4000 (svarar mot 2 msek), och maxpådrag bakåt är 2000 (svarar mot 1 msek).
```
OCR1A = Throttle_zero + Throttle;
```
Det finns även en icke-symmetrisk dödzon som måste kompenseras. I nuvarande implementation så motsvarar +-100 inte maxppådrag.

Det funkar liknande för styrningen men då är det register ```OCR1B```.
