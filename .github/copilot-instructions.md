# Instrukcje dla Copilota – Stewart Platform

## Rola pliku
Ten plik opisuje sposób współpracy z asystentem, styl zmian i oczekiwany format kodu.
Dane techniczne projektu, pinout, komendy, stan funkcji i roadmapa powinny być utrzymywane w `README.md`.

## Ogólne zasady pracy
- Projekt embedded: PlatformIO + C++ na ESP32.
- Preferuj małe, lokalne zmiany zamiast pełnego przepisywania plików.
- Zanim zaproponujesz refaktor, najpierw opisz problem, który ma zostać rozwiązany.
- Kod ma działać na rzeczywistym sprzęcie, nie tylko kompilować się teoretycznie.
- Nie zmieniaj pinów, zakresów ID serw ani parametrów interfejsów bez wyraźnej prośby.
- Nie usuwaj istniejących komentarzy technicznych bez wyraźnej prośby.
- Jeżeli jakaś komenda lub funkcja zależy od konkretnego firmware serwa, zaznacz to w opisie i w komentarzu kodu.

## Styl propozycji zmian
- Generuj czysty, pełny kod.
- Nie używaj znaczników typu `...existing code...`.
- Jeśli zmiana dotyczy konkretnego pliku, podawaj pełny, spójny fragment gotowy do wklejenia.
- Nie dopisuj komentarzy o historii zmian, poprzednim stanie ani o tym, co było błędne.
- Opisuj tylko to, co kod robi teraz.

## Komentarze w kodzie
- Każda funkcja ma mieć nagłówek:
  - co robi,
  - wejście,
  - wyjście.
- Komentuj kroki logiki tam, gdzie występują:
  - parsowanie danych,
  - walidacja,
  - komunikacja z urządzeniem,
  - warunki ochronne,
  - operacje na rejestrach,
  - ograniczenia zakresów.
- Zachowuj komentarze sekcyjne organizujące plik, np.:
  - `// === DEKLARACJE FORWARD ===`
  - `// === PINY ===`
  - `// === ZAKRESY ID ===`
  - `// === PROTOKÓŁ ===`
- Zachowuj komentarze przy grupach stałych, jeśli niosą znaczenie techniczne.
- Nie komentuj rzeczy oczywistych składniowo.

## Bezpieczeństwo zmian sprzętowych
- Nie proponuj zmian zwiększających ryzyko niekontrolowanego ruchu serw bez wyraźnej potrzeby.
- Operacje na EEPROM, ID serw i komendach resetu traktuj jako wrażliwe.
- Przy zmianach związanych z ruchem, momentem i komunikacją szeregową preferuj walidację wejścia i czytelne logi diagnostyczne.
- Jeśli zachowanie komendy zależy od modelu serwa lub wersji firmware, zaznacz niepewność zamiast zakładać zgodność.

## Odpowiedzi i komunikacja
- Odpowiedzi mają być krótkie, konkretne i praktyczne.
- Gdy proponowana jest zmiana strukturalna, najpierw podaj krótko powód.
- Gdy dane techniczne projektu wymagają uporządkowania, aktualizuj `README.md`, a nie ten plik.