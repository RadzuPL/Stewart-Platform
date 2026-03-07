# Instrukcje dla Copilota – Stewart Platform

## Ogólne zasady
- Projekt embedded: PlatformIO + C++ na ESP32.
- Nie zmieniaj pinów (są połączone na stałe w gotowym układzie), nazw ID serw (11-16) bez wyraźnej prośby.
- Zanim zaproponujesz refaktor, opisz problem który rozwiązujesz.
- Preferuj małe zmiany zamiast pełnego przepisywania.
- Kod musi działać na rzeczywistym sprzęcie.

## Konfiguracja sprzętu (nie zmieniać bez zgody)
- **ESP32**, komunikacja z serwami: Serial2 (UART2) 1 Mbps, TX=GPIO19, RX=GPIO18
- **Debug/komendy**: Serial (UART0) 115200bps (GPIO1=TX, GPIO3=RX)
- **Master UART**: GPIO16/17 — do implementacji
- **OLED**: I2C SDA=GPIO21, SCL=GPIO22
- **LED RGB**: WS2812 GPIO23
- **Serwa**: ID 11–16, biblioteka SCSCL

## Styl propozycji zmian — WYMAGANE
- **Czisty, pełny kod** — nigdy nie używaj `// ... istniejący kod ...` ani innych zaznaczników
- Jeśli zmiana wymaga wstawienia, opisz WHERE w komentarzu filepath, a cały kod niech będzie spójny
- **Brak zaznaczników historii**: nie pisz o tym co zostało zmienione, jakie były błędy, co było przed
- Kod powinien wyglądać jak kompletny, nowy moduł

## Komentarze — WYMAGANE
- **Każda funkcja**: nagłówek wyjaśniający CO robi, wejście, wyjście
- **Każdy krok logiki**: okomentuj w pętlach, parsowaniu, warunkach, transformacjach danych
- **Nie pisz**: 
  - Oczywistych rzeczy (`i++; // increment i`)
  - Komentarzy na temat zmian/przeszłości (np. "Stary kod był błędny", "Zmieniono żeby...")
- **Pisz**: CO kod robi TERAZ — wartości zwracane, warunki, ograniczenia, rejestry, zakresy
- **Nie kasuj**: komentarzy bez wyraźnej prośby nawet jeśli wydają się oczywiste.

Przykład DOBRY:

```cpp
if(sc.Ping(id) != -1) {
  // Ping zwrócił coś innego niż -1 — serwo odpowiada i jest dostępne
  foundIDs[servoCount] = id;
  servoCount++;
}
```

Przykład ZŁY:

```cpp
if(sc.Ping(id) != -1) {
  // Zmieniono warunek aby poprawnie mapować serwa
  foundIDs[servoCount] = id;
  servoCount++;
}
```