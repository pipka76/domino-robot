# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Projekt
Robot z LEGO + kostka **Brian** (FEL ČVUT, MicroPython) na úlohu
[Domino](https://robosoutez.fel.cvut.cz/zadani-soutezni-ulohy-domino-0) (Robosoutěž 2026, SŠ): sebrat kostky
domina z řad na hřišti a složit je ve žlutém poli do barevně navazujících řetězů (n kostek = n² bodů,
+1 za kolmý spoj; spoj: mezera ≤ 5 mm, úhel ≤ 10°). Jízda trvá 90 s.
Uživatel (Metoděj) ladí na reálném robotu a posílá výpisy z konzole; komunikace česky.

## Příkazy
```bash
python3 planner.py                 # ověří, že z každé řady jde složit řetěz
python3 main.py --mock             # simulace active_mission (virtuální čas, „Celkový čas“ na konci)
python3 main.py --mock test_bottom # konkrétní mise; --flip = kostky otočené o 180°; --side right; --realtime
```
3D simulace (Godot, složka `simulation/` od Jiřího): `"/Applications/Godot_mono.app/Contents/MacOS/Godot" --path simulation` (výchozí = naše mise main, `-- --ours MISE` jiná, `-- --theirs` Jiřího MissionController)
(okno; `--headless --fixed-fps 120 … --mission` rychle). Náš Python běží přes `hw_sim.py` (TCP 5577, lock-step),
model zásobníku/rukou v `simulation/src/Ours/OurRunner.cs`; `mechanism.py` hlásí stav přes `hw.arm_target`. Po změně C# `dotnet build` v `simulation/`.
Po každé změně kódu/JSONu spustit aspoň `python3 main.py --mock` (validace + čas mise) a u dotčených
testů i jejich misi. Simulace NEověřuje fyziku (mock vidí polovinu kostky s indexem 0, odražené světlo
podlahy 17 / kostky 40 z `mission.json` → `mock`).

Na robot: nahrát `.py` (kromě `sample.py`, `hw_mock.py`) + `mission.json` do **stejné složky**
(BriCode ve VS Code nebo BRemote přes WiFi, IP 192.168.4.1), spustit ze SD karty; výstup v BRemote Program console.
Po úpravě vždy napsat, **které soubory nahrát znovu**. Commit/push jen na požádání (push rovnou na `main`).

## Architektura
- **Co** robot dělá = `mission.json` (mise, makra, kalibrace, rozmístění kostek). **Jak** = Python.
  Ladění = měnit čísla v JSONu. Klíče začínající `_` jsou komentáře (`_doladit` = odhadnutá hodnota).
- `main.py` – načte a zvaliduje JSON (`CHYBA: …` → nespustí se), spustí misi; `time_limit_s: 0` = bez limitu
  (teď kvůli ladění; **na soutěž 88**).
- `actions.py` – interpret příkazů (`ACTIONS` = cmd → (funkce, povinné parametry)). Klíčové:
  - `drive_mm`/`turn_deg`/`curve_deg` drží směr přes `heading_ref`: při `drive.absolute_heading` se cílový směr
    počítá od startu mise (gyro se nuluje jen při startu programu), chyby otáček se nesčítají.
  - `pick_row` – hledá kostku (`until_block`: odražené světlo mimo `colour.floor_reflected` = kostka),
    přečte barvu, natočí ruce (`choose_rot`), zasune (`advance_len`), posune zásobník po `pick.shift_after`
    (`shift_at_next`: až u další kostky, kde robot stejně stojí). Parametry kroku přepisují `pick`
    (`approach_speed`, `advance_speed`, `ignore_colours`, `shift_extra`, `first_taken`).
  - `pick_prepare` + `curve` + `pick_row first_taken` = 1. kostku nabere zatáčka (u spodní řady jinak nejde).
  - `drive_time` s `square: true` umí po dotlačení ke stěně převzít směr gyra (teď nepoužito).
- `mechanism.py` – stavový automat motoru C: rotace rukou (NONE/CW/CCW) × slot zásobníku (LEFT/RIGHT).
  Ruce i zásobník jsou na **jednom motoru**: rotace za krajní polohou posouvá zásobník. `transitions` jsou
  nelineární (hodnoty ze `sample.py`), `shift_extra` = vůle. `nudge` dotlačí zásobník (krajní rotace + `nudge_deg`)
  a motor pak vrátí o skutečně ujeté stupně navíc – jinak se všechny další pohyby posunou.
- `planner.py` – hrubou silou volí orientaci kostek, aby barvy navazovaly; navázání na řetěz (`append_to`, `join_end`).
- `hw.py` – **jediný** soubor importující `brian.*`; `hw_mock.py` – simulace se stejným rozhraním.
- `t_*.py` – spouštěče testovacích misí (`t_bottom.py` → `test_bottom`); `kalibrace_gyra.py`; `sample.py` jen reference.

## Mise `main` (levá půlka) – aktuální stav
1. Start ručně: zadek zásobníkem u horní stěny nad levou řadou, ruce `CW`, zásobník `RIGHT` (nastavuje uživatel, = `arm.initial`).
   `pick_row left` → A; makro `curve_a` (couvnout 40 + větší oblouk, jinak zásobník drhne o stěnu); doprava,
   nahoru (`drive 550`), `unload` A.
2. Makro `to_bottom`: couvnout zásobníkem k levé stěně, otočka dolů kolem pravého kola (zůstane u zdi),
   dolů (`drive 175` – svázáno s výškou vykládky A!), `pick_prepare` s `rot: NONE`, makro `curve_b` (oblouk
   těsně u zdi), `pick_row bottom` (`ignore_colours`, `shift_extra 300`).
3. Makro `after_b`: couvnout 40, otočka na místě 180°, doleva, nahoru, `unload` B **rovnoběžně vlevo od A**
   (napojení do L zatím vypnuto). V simulaci ~94 s – přes limit, bude potřeba zrychlit.

## Zjištění z ladění (důležité)
- Rozměry robota: šířka 220 mm, od osy kol dopředu 175 mm (senzor až 210), dozadu 190 mm. Na hřišti
  (vnitřní plocha 2440×1600, půlka 1210 mm) je před 1. kostkou spodní/horní řady jen ~26 cm a mezi řadami
  ~22,6 cm → rovně se před ně robot nevejde; levou/pravou řadu jde nabrat jen z ručního startu.
- **Barevný senzor** je ve výšce kostky: bílou podlahu čte jako GREEN → přítomnost kostky podle odraženého
  světla (podlaha ~17, zelená/modrá 8, červená 40, žlutá 58 → `floor_reflected: [12, 24]`). Barvu číst až nad kostkou.
- Starý firmware: motor nemá `rotate_by_angle` → `hw._arm_start` zkouší i `rotate_to_angle` / `rotate_to`.
  `is_stalled()` je nespolehlivé; u dorovnání zásobníku nepoužívat ani hlídání dorazu podle enkodéru.
- Motor C reálně ~1000–1100 deg/s; posun zásobníku ~3,8 s. V levém slotu ruce otáčí zrcadlově
  (`seen_to_front: {RIGHT: CCW, LEFT: CW}`); barva, kterou senzor vidí, jde do zásobníku první.
- **Couvání s kostkami v zásobníku max. ~40 mm** (jinak kostky vyjíždějí). U stěny se robot nemůže otočit
  bez couvnutí/většího oblouku (roh jde ke zdi).
- Uživatel nechce další zastavení/popojetí navíc (každé brzdění = chyba); prodlužovat raději existující jízdy.
- Gyro: kalibrace ruční (`kalibrace_gyra.py`), při startu se měří drift; `gyro_sign: -1`. `turn_deg` končí i při
  přejetí cíle o ≤ 3° (jinak kmitá).
- Změna výšky vykládky A posouvá i místo srovnání o levou stěnu → upravit i jízdu dolů v `to_bottom`.

## Konvence
- Jednotky: mm, stupně, deg/s, s. `turn`/`curve` kladně = doleva (CCW). Záporné `mm` = couvání.
  Směry gyra od startu: 0 = dolů (start), +90 doprava, +180 nahoru.
- Kód musí běžet v MicroPythonu i CPythonu (žádné f-stringy, `.format()`; čas přes `common.now()`).
- Nové chování = parametr v `mission.json` s defaultem v kódu (`.get(..., default)`); nový příkaz do `ACTIONS`
  (+ validace v `main.py`); aktualizovat README. Opakované úseky trasy dávat do maker sdílených misí a testem.
