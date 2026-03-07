# Stewart Platform

## Cel projektu
Oprogramowanie dla sterownika Platformy Stewarta będącej częścią większego systemu Awatara Kinematycznego.

Głównym zadaniem sterownika jest:
- komunikacja z 6 serwomechanizmami magistralowymi,
- realizacja ruchu na podstawie zadanych pozycji,
- przygotowanie warstwy pod docelowe sterowanie 6DoF,
- diagnostyka i monitoring stanu układu.

## Aktualny stan projektu
Obecna wersja oprogramowania zapewnia:
- komunikację z serwami Feetech SCS225 przez UART2,
- skanowanie serw o ID `1` oraz `11..16`,
- sterowanie momentem serw,
- wysyłanie pozycji do pojedynczych i wielu serw,
- zmianę ID serwa,
- podstawową i rozszerzoną diagnostykę przez port szeregowy,
- prezentację statusu na OLED,
- sygnalizację stanu przez LED RGB.

Status praktyczny:
- komendy wysyłane z terminala działają poprawnie,
- serwomechanizmy reagują i wykonują ruch,
- komenda diagnostyczna działa,
- komenda resetu fabrycznego nie jest potwierdzona jako wspierana przez model SCS225.

## Sprzęt

### Płyta sterująca
Sterownik oparty o ESP32:
- mikrokontroler ESP32,
- komunikacja z serwami przez UART2,
- port debug przez UART0,
- przygotowany port UART do komunikacji z masterem,
- wyświetlacz OLED SSD1306 128x32 po I2C,
- 2 diody WS2812 jako sygnalizacja stanu.

Przydatne linki:
- https://www.waveshare.com/wiki/Servo_Driver_with_ESP32
- https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

### Pinout i połączenia

| Funkcja | GPIO | Notatka |
|---------|------|---------|
| Serial2 TX (serwa) | 19 | UART2, 1 Mbps |
| Serial2 RX (serwa) | 18 | UART2, 1 Mbps |
| Serial TX (debug) | 1 | UART0, 115200 bps |
| Serial RX (debug) | 3 | UART0, 115200 bps |
| I2C SDA (OLED) | 21 | SSD1306 128x32 |
| I2C SCL (OLED) | 22 | SSD1306 128x32 |
| WS2812 (LEDs) | 23 | 2x RGB |
| Master TX | 16 | Do implementacji |
| Master RX | 17 | Do implementacji |

## Serwomechanizmy

Model:
- Feetech SCS225

Link:
- https://www.makerstore.com.au/product/mb-elc-servo-scs225-270/

Parametry używane w projekcie:
- liczba serw: 6
- ID robocze: `11, 12, 13, 14, 15, 16`
- ID obserwowane dodatkowo: `1`
- interfejs: TTL half-duplex
- baudrate: `1000000 bps`

Uwagi:
- ID `1` jest uwzględniane w skanowaniu głównie diagnostycznie, np. po zmianie ID lub po ewentualnym resecie.
- Komenda resetu fabrycznego nie jest obecnie potwierdzona jako obsługiwana przez firmware SCS225.

## Zasilanie

Ograniczenia:
- serwa: `4.0V - 7.4V`
- płyta sterująca: `DC 6V - 12V`

Przyjęty zakres pracy:
- zasilanie układu: `6.0V - 7.4V`

Szacowany pobór:
- prąd spoczynkowy pojedynczego serwa: około `150 mA @ 7.4V`
- prąd maksymalny pojedynczego serwa: około `2.5 A @ 7.4V`
- prąd spoczynkowy całego układu: około `1 A`
- teoretyczny prąd maksymalny dla 6 serw: do około `15 A`

Uwagi:
- wartość maksymalna jest skrajna i projektowo mało prawdopodobna,
- należy zachować zapas prądowy zasilania,
- przewody i ścieżki powinny być dobrane pod pracę z kilkoma serwami pod obciążeniem.

## Środowisko programistyczne

- PlatformIO
- framework: Arduino
- board: `esp32dev`
- platform: `espressif32`

Biblioteki:
- `Adafruit GFX Library`
- `Adafruit SSD1306`
- `Adafruit NeoPixel`
- `FTServo`

## Konfiguracja PlatformIO

Plik `platformio.ini` definiuje:
- płytkę `esp32dev`,
- framework `arduino`,
- monitor portu szeregowego `115200`,
- zależności bibliotek wymaganych do OLED, LED i serw.

## Struktura projektu

```text
src/
  main.cpp       - logika główna, komendy UART, komunikacja z serwami
  display.cpp    - obsługa OLED
  display.h      - interfejs OLED
  leds.cpp       - obsługa WS2812
  leds.h         - interfejs LED
platformio.ini   - konfiguracja PlatformIO
README.md        - dokumentacja projektu
```

## Obsługiwane komendy UART

Komendy są przyjmowane przez port debug `Serial` z prędkością `115200 bps`.

### `T0` / `T1`
Włącza lub wyłącza moment serw.

Przykłady:
- `T1`
- `T0`

### `S<speed>`
Ustawia globalną prędkość używaną przy komendach ruchu.

Przykłady:
- `S500`
- `S2000`

Zakres stosowany w kodzie:
- minimalnie `50`
- maksymalnie `4000`

### `P<id:pos[,id:pos...]>`
Ustawia pozycję jednego lub wielu serw.

Przykłady:
- `P11:512`
- `P11:512,12:430,13:700`

Zakres pozycji:
- `0..1023`

### `R`
Skanuje serwa i wypisuje skrócony raport.

Skanowane ID:
- `1`
- `11..16`

### `D`
Wykonuje diagnostykę wszystkich znalezionych serw.

Zakres diagnostyki:
- odpowiedź na `Ping`,
- pozycja,
- napięcie,
- zrzut rejestrów EEPROM,
- zrzut rejestrów SRAM.

### `D<id>`
Wykonuje diagnostykę pojedynczego serwa.

Przykład:
- `D11`

### `I<old:new>`
Zmienia ID serwa przez zapis do rejestru EEPROM ID.

Przykłady:
- `I1:11`
- `I11:12`

Uwagi:
- operacja jest wrażliwa,
- nowe ID nie może być zajęte,
- po operacji może być potrzebne ponowne zasilenie zależnie od modelu serwa.

### `Z<id>`
Wysyła próbę resetu fabrycznego do wskazanego serwa.

Przykład:
- `Z11`

Uwagi:
- komenda jest zaimplementowana diagnostycznie,
- dla modelu Feetech SCS225 nie ma obecnie potwierdzenia, że reset fabryczny jest obsługiwany,
- wynik dodatni z biblioteki nie gwarantuje rzeczywistego resetu,
- wynik odrzucenia nie oznacza problemu z całą komunikacją.

### `H`
Wyświetla pomoc.

## OLED i LED

### OLED
Wyświetlacz pokazuje:
- liczbę wykrytych serw,
- stan torque,
- globalną prędkość,
- pozycje aktualnie znalezionych serw.

### LED RGB
Sygnalizacja:
- zielony: pełna gotowość i aktywny torque,
- żółty: wykryto część serw lub torque jest wyłączony,
- czerwony migający: brak wykrytych serw.

## Diagnostyka

Aktualnie diagnostyka obejmuje:
- test obecności serwa,
- odczyt pozycji,
- odczyt napięcia,
- zrzut rejestrów w blokach EEPROM i SRAM.

Zastosowanie:
- weryfikacja komunikacji,
- porównanie konfiguracji serw,
- szukanie różnic między egzemplarzami,
- analiza odpowiedzi firmware.

## Znane problemy i ograniczenia

- Nie ma jeszcze warstwy kinematyki 6DoF.
- Nie ma jeszcze interfejsu komend od nadrzędnego kontrolera na UART 16/17.
- Komenda resetu fabrycznego nie jest potwierdzona dla SCS225.
- Część danych diagnostycznych wymaga interpretacji na podstawie dokumentacji rejestrów serwa.
- Aktualne sterowanie działa na poziomie pozycji serw, nie pozycji platformy.

## Roadmap

### Etap 1 — baza komunikacyjna
- [x] inicjalizacja UART dla serw
- [x] skanowanie serw
- [x] sterowanie pojedynczym serwem
- [x] sterowanie wieloma serwami
- [x] monitoring przez UART debug
- [x] prezentacja statusu na OLED
- [x] sygnalizacja LED
- [x] podstawowa diagnostyka
- [x] rozszerzona diagnostyka rejestrów

### Etap 2 — warstwa sterowania urządzeniem
- [ ] implementacja komunikacji z masterem przez UART 16/17
- [ ] standaryzacja formatu komend wejściowych
- [ ] spójny format odpowiedzi i logów

### Etap 3 — warstwa ruchu
- [ ] opracowanie funkcji ruchu synchronicznego 6 serw
- [ ] obsługa dodatkowych parametrów ruchu
- [ ] określenie fizycznych limitów
- [ ] implementacja limitów w kodzie

### Etap 4 — kinematyka platformy
- [ ] integracja matematyki platformy Stewarta
- [ ] przeliczanie 6DoF na kąty serw
- [ ] walidacja mechanicznych zakresów pracy
- [ ] optymalizacja obliczeń

## Kinematyka
Planowana baza odniesienia:
- https://github.com/NicHub/stewart-platform-esp32

Docelowo kod kinematyki ma zostać zaadaptowany do:
- sterowania cyfrowymi serwami magistralowymi,
- fizycznych wymiarów tej konkretnej platformy,
- ograniczeń mechanicznych i elektrycznych układu.

## Najbliższe kroki
Najbardziej logiczne następne kroki:
1. uzupełnić i ustabilizować README jako główną dokumentację techniczną,
2. dopracować komentarze w `main.cpp` i zachować komentarze sekcyjne,
3. zdefiniować docelowy format komend od kontrolera nadrzędnego,
4. przygotować warstwę abstrakcji pod sterowanie 6 osiami,
5. rozpocząć integrację kinematyki.

