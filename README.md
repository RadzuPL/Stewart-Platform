# Stewart Platform

## Cel
Opracowanie oprogramowania dla sterownika Platformy Stewarta (manipulatora równoległego), będącego częścią większego systemu Awatara Kinematycznego. 
Głównym zadaniem jest precyzyjne pozycjonowanie platformy w 6 stopniach swobody (3 liniowe, 3 obrotowe) na podstawie zadanych parametrów.

## Sprzęt

### Płyta sterująca

https://www.waveshare.com/wiki/Servo_Driver_with_ESP32
https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

- mikrokontroler ESP32
- port szeregowy UART do komunikacji z serwami (1000000bps): TX -> GPIO 19, RX -> GPIO 18
- port szeregowy UART debug (115200bps): TX -> GPIO 1 / U0TXD, RX -> GPIO 3 / U0RXD 
- port szeregowy UART do komunikacji z masterem (115200bps): TX -> GPIO 16, RX -> GPIO 17
- wyświetlacz OLED SSD1306 128x32, komunikacja I2C: SDA -> GPIO 21, SCL -> GPIO 22
- diody LED RGB 2x WS2812: GPIO 23

#### Pinout i połączenia

| Funkcja | GPIO | Notatka |
|---------|------|---------|
| Serial2 TX (serwa) | 19 | UART2 @ 1 Mbps |
| Serial2 RX (serwa) | 18 | UART2 @ 1 Mbps |
| Serial TX (debug) | 1 (U0TXD) | UART0 @ 115200 bps |
| Serial RX (debug) | 3 (U0RXD) | UART0 @ 115200 bps |
| I2C SDA (OLED) | 21 | SSD1306 128x32 |
| I2C SCL (OLED) | 22 | SSD1306 128x32 |
| WS2812 (LEDs) | 23 | 2x RGB diody |
| Master TX (przyszłość) | 16 | Do implementacji |
| Master RX (przyszłość) | 17 | Do implementacji |

### Serwomechanizmy

https://www.makerstore.com.au/product/mb-elc-servo-scs225-270/

- 6 szt. Feetech SCS225 (cyfrowe, magistrala szeregowa)
- ID serw: 11, 12, 13, 14, 15, 16
- Interfejs: TTL half-duplex, baudrate 1 Mbps

### Zasilanie
- ograniczenie wynikające z serw: 4-7.4V
-- prąd spoczynkowy: 150mA@7.4V
-- prąd maksymalny: 2.5A@7.4V
- ograniczenie wynikające z płyty sterującej: DC 6-12V
- zasilanie wynikowe: 6-7.4V
- pobór finalny dla 7,4V: 
-- spoczynkowy: ok 1A
-- prąd maksymalny: 15A (nierealny dla ścieżek i przedodów - przy zakładanym projekcie nigdy nie zostanie osiągnięty ze względu na niskie obciążenia serw - nie trzeba się tym przejmować)

## Kinematyka

Skorzystać z gotowego projektu po adaptacji do serw cyfrowych:
https://github.com/NicHub/stewart-platform-esp32


## Funkcje
- sprawdzić i uzupełnić opis

## Struktura kodu
- uzupełnić

## Znane problemy
- sprawdzić

## TODO
- do określenia

## Roadmap

- Inicjalizacja serw
- implematacja monitoringu działania poprzez port szeregowy debugowania
- Testowanie pingowania serw o ID 11-16 
- Wyświetlanie statusu na OLED
- Implementacja portu szeregowego do otrzymywania komend 
- Opracowanie funkcji dla ruchu pojedynczego serwa
- Opracowanie funkcji dla synchronicznego ruchu 6 serw
- Obsługa parametrów ddodatkowych
- Integracja matematyki platformy Stewarta (przeliczanie 6DoF na kąty serw)
- określenie fizycznych limitów i ich implementacja w kodzie
- Optymalizacja działania kinematyki

