# CLAUDE.md – Domino robot (Robosoutěž 2026, SŠ)

Robot z LEGO + kostka **Brian** (FEL ČVUT, MicroPython) na úlohu
[Domino](https://robosoutez.fel.cvut.cz/zadani-soutezni-ulohy-domino-0): sebrat kostky domina
z řad na hřišti a složit je ve žlutém poli do barevně navazujících řetězů (n kostek = n² bodů,
L-spoje povoleny, spoj: mezera ≤ 5 mm, úhel ≤ 10°). Jízda trvá 90 s.
Uživatel (Metoděj) ladí na reálném robotu a posílá výpisy z konzole; komunikace česky.

## Architektura
- **Co** robot dělá = `mission.json` (mise, makra, kalibrace, rozmístění kostek). **Jak** = Python.
  Ladění = měnit čísla v JSONu, ne kód. Klíče začínající `_` jsou komentáře.
- `main.py` – načte a **zvaliduje** JSON (chyba → `CHYBA: …`, nespustí se), spustí misi, hlídá `time_limit_s`.
- `actions.py` – interpret příkazů (`ACTIONS` = cmd → (funkce, povinné parametry)): `drive`, `turn`,
  `curve` (oblouk), `drive_until_colour`, `follow_line`, `pick_row`, `pick_prepare`, `unload`, `arm`, `macro`, `print_colour`, `print_gyro`…
- `planner.py` – hrubou silou (2^4) volí orientaci kostek, aby barvy navazovaly; umí navázání na existující řetěz (`append_to`, `join_end`).
- `mechanism.py` – stavový automat motoru C: rotace (NONE/CW/CCW) × slot zásobníku (LEFT/RIGHT).
- `hw.py` – **jediný** soubor importující `brian.*`. `hw_mock.py` – simulace se stejným rozhraním.
- `t_*.py` – spouštěče testovacích misí (`t_arm.py` → mise `test_arm`). `kalibrace_gyra.py` – ruční kalibrace.
- `sample.py` – původní prototyp, jen reference (nenahrávat).

## Příkazy
```bash
python3 planner.py                 # ověří, že z každé řady jde složit řetěz
python3 main.py --mock             # simulace active_mission (virtuální čas, Celkový čas na konci)
python3 main.py --mock test_shift  # konkrétní mise; --flip = kostky otočené o 180°; --side right
```
Po každé změně kódu/JSONu spustit aspoň `python3 main.py --mock` (validace + čas mise).
Simulace NEověřuje fyziku – mock „vidí“ polovinu kostky s indexem 0, reálný senzor (vlevo) může vidět jinou.

Na robot: nahrát všechny `.py` (kromě `sample.py`, `hw_mock.py`) + `mission.json` do **stejné složky**
(BriCode ve VS Code nebo BRemote přes WiFi, IP 192.168.4.1), spustit ze SD karty. Výstup v BRemote Program console.
Po úpravě připomenout uživateli, **které soubory nahrát znovu**.

## Zjištění z ladění na robotu (důležité)
- **Starý firmware Brian**: motor nemá `rotate_by_angle` → `hw._arm_start` zkouší `rotate_by_angle` / `rotate_to_angle` / `rotate_to`.
- **`is_stalled()` je nespolehlivé** (hlásí zablokování při normálním těžkém posunu zásobníku) → zablokování se
  pozná podle enkodéru (`arm.stall_s`). `arm_move` loguje `arm ok: … deg za … s` nebo `ZABLOKOVÁNO/TIMEOUT`.
- Motor C reálně ~1000–1100 deg/s i při nastavených 1400; posun zásobníku ~3,8 s.
- **Mechanika zásobníku má vůli**: posun doprava potřebuje víc stupňů → `arm.shift_extra` {LEFT, RIGHT} (může být i záporné).
- **V levém slotu motor otáčí kostkou zrcadlově** → `pick.seen_to_front` = `{"RIGHT": "CCW", "LEFT": "CW"}`.
  Pravidlo uživatele: barva, kterou senzor vidí, jde do zásobníku vždy první.
- Před posunem zásobníku se kostka zasune déle (`advance_mm + shift_advance_mm`) a počká `shift_wait_s`.
- **Gyro**: problémy byly ze špatné kalibrace (robot se hýbal). Kalibrace je **ruční** (`kalibrace_gyra.py`),
  `gyro_reboot_on_start: false`; při startu mise se 1 s měří drift (`gyro_check_s`). `gyro_sign: -1` ověřeno.
- Domácí plocha má **hnědou podlahu** → `BROWN` (a `BLACK`) jsou v `colour.background`. Na soutěži (bílá) přeměřit `line_target`.
- Otáčení na místě kmitalo kolem cíle (gyro vrací celé stupně) → `turn_deg` zastaví i při přejetí cíle o ≤ 3°.

## Mise `main` (levá půlka hřiště)
1. `pick_row left` shora dolů → řetěz A; `curve +90` (oblouk, ne na místě – zadkem by narazil do zdi) do středu;
   jízda doprava + nahoru k levému hornímu rohu žlutého pole; `unload` (A shora dolů).
2. Přesun do levého dolního rohu; `pick_prepare bottom` (rameno předem natočené pro 1. kostku, kterou nabere zatáčka;
   senzor vlevo vidí horní polovinu = `seen: 0`); `curve +90`; `pick_row bottom first_taken`; zpět do středu
   a napojení do **L** k hornímu konci A (`append_to: A.rear`, `join_end: front`).
Kroky s `_doladit` mají odhadnuté vzdálenosti. Mise je těsně pod limitem (~82 s v simulaci) – hlídat čas.
`main_v1` = původní mise (jen reference).

## Konvence
- Jednotky: mm, stupně, deg/s, s. `turn`/`curve` kladně = doleva (CCW). Záporné `mm` = couvání.
- Kód musí běžet v MicroPythonu i CPythonu (žádné f-stringy navíc, `.format()`; čas přes `common.now()`).
- Nové chování přidávat jako parametr v `mission.json` s rozumným defaultem v kódu (`.get(..., default)`),
  nový příkaz zaregistrovat v `ACTIONS` a případně ve validaci v `main.py`; aktualizovat README.
- Editovat soubory na místě, neměnit nesouvisející věci; necommitovat bez požádání.
