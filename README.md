# Domino robot – návod k používání

Program pro robota z LEGO + **Brian** na úlohu [Robosoutěž – Domino](https://robosoutez.fel.cvut.cz/zadani-soutezni-ulohy-domino-0).

Hlavní myšlenka: **co** robot dělá, je popsané v souboru `mission.json`. **Jak** to dělá, řeší Python kód. Chování se ladí úpravou čísel a kroků v `mission.json`, kód není potřeba měnit.

---

## 1. Soubory

| Soubor | K čemu je |
|---|---|
| `mission.json` | Popisný soubor: hřiště, barvy kostek, porty, kalibrace, mise, makra |
| `main.py` | Spouští misi: načte a zkontroluje JSON, spustí misi, po časovém limitu robota zastaví |
| `actions.py` | Příkazy, které lze použít v misích (`drive`, `turn`, `pick_row`, …) |
| `mechanism.py` | Stav motoru C (otočení kostky × slot zásobníku) |
| `planner.py` | Spočítá, jak natočit kostky, aby barvy v řetězu navazovaly |
| `hw.py` | Ovládání Brian hardware (jediný soubor, který importuje `brian.*`) |
| `hw_mock.py` | Simulace hardware pro zkoušení na PC |
| `common.py` | Sdílené konstanty a pomocné funkce |
| `sample.py` | Původní prototyp (jen pro referenci) |

---

## 2. Rychlý start

### Na PC (bez robota)

```bash
python3 planner.py                       # ověří, že z každé řady jde složit řetěz
python3 main.py --mock                   # simulace mise z active_mission
python3 main.py --mock test_arm          # simulace konkrétní mise
python3 main.py --mock --flip            # kostky náhodně otočené o 180° (2. kolo)
python3 main.py --mock --side right      # simulace pro pravou půlku hřiště
```

Simulace vypíše jednotlivé kroky s časem, plán natočení kostek, pohyby motoru C a celkový čas. Hodí se ke kontrole, že je JSON v pořádku a že se mise vejde do časového limitu. Rychlosti v simulaci jsou jen přibližné.

### Na robotu

1. Na Brian nahrajte **všechny** `.py` soubory a `mission.json` (bez `sample.py` a `hw_mock.py`).
2. V `mission.json` nastavte `"active_mission"` na misi, kterou chcete spustit, a `"side"` na půlku hřiště (`"left"` / `"right"`).
3. Spusťte `main.py`.

> Při startu se rekalibruje gyro. **Robot musí stát v klidu**, dokud se nerozjede.

Před spuštěním se vždy zkontroluje `mission.json`. Pokud obsahuje chybu (neznámý příkaz, chybějící parametr, neplatnou barvu…), program vypíše `CHYBA: …` a nespustí se.

---

## 3. Struktura `mission.json`

JSON nepodporuje komentáře, proto se jako komentáře používají klíče začínající podtržítkem (`"_"`, `"_poznámka"`). Program je ignoruje.

**Jednotky:** vzdálenosti v mm, úhly ve stupních, rychlosti ve °/s (stupně otočení kola za sekundu), čas v sekundách.
**Konvence:** `turn` s kladným úhlem = otočení **doleva** (proti směru hodin), záporná vzdálenost = couvání.

### 3.1 Hlavička

```json
"side": "left",              // půlka hřiště: left / right
"active_mission": "main",    // která mise se spustí na robotu
"time_limit_s": 88,          // po tomto čase robot zastaví (soutěž má 90 s)
"start_delay_s": 0           // prodleva před startem (jen na robotu)
```

### 3.2 `field` – hřiště

- `layouts.<strana>.<řada>`: barvy 4 kostek v řadě.
  - `top` / `bottom`: kostky zleva doprava, každá jako `[horní, dolní]` polovina.
  - `left` / `right`: kostky shora dolů, každá jako `[levá, pravá]` polovina.
  - Platné barvy: `BLACK BLUE GREEN YELLOW RED WHITE BROWN NONE`.
- `geometry_mm.<řada>_spacing`: rozteč kostek v řadě v mm (`pick_row` podle ní pozná, jak daleko má další kostku hledat).

> Pokud pořadatel změní rozmístění kostek (např. ve finále), stačí přepsat `layouts`.

### 3.3 `robot` – hardware a kalibrace

**`ports`**: porty motorů a senzorů (`A`–`D`, `S1`–`S4`). Bez gyra nastavte `"gyro": null`.

**`drive`**: pojezd

| Klíč | Význam | Jak ladit |
|---|---|---|
| `deg_per_mm` | stupně kola na 1 mm jízdy | `test_drive`: robot má ujet přesně 500 mm |
| `wheel_deg_per_robot_deg` | stupně kola na 1° otočení robota (použije se jen bez gyra) | `test_turn` s `use_gyro: false` |
| `left_dir`, `right_dir` | `1` / `-1`, pokud je motor namontovaný obráceně | robot má po `drive` jet dopředu |
| `gyro_sign` | `-1` = gyro šipkami nahoru (Brian měří kladně po směru hodin) | `test_turn`: `90` musí otočit **doleva** |
| `gyro_reboot_on_start` | rekalibrovat gyro při startu | |
| `use_gyro` | jezdit rovně a zatáčet podle gyra | |
| `speed`, `min_speed` | běžná a minimální rychlost (°/s) | |
| `ramp_s` | doba rozjezdu | |
| `decel_k` | jak brzy začne brzdit před cílem | větší = brzdí později |
| `heading_kp` | jak silně drží směr při jízdě rovně | robot kličkuje → snížit |
| `turn_speed`, `turn_min_speed`, `turn_kp`, `turn_tol_deg` | regulace otáčení | přetáčí → snížit `turn_kp` |

**`colour`**: barevný senzor

| Klíč | Význam |
|---|---|
| `background` | barvy, které znamenají „není tu kostka“ (typicky `WHITE`, `NONE`) |
| `line` | barva vodicí čáry (`BLACK`) |
| `debounce_reads` | kolikrát po sobě se musí barva přečíst, aby se jí věřilo |
| `line_target`, `line_kp` | sledování čáry (odražené světlo 0–100) |

**`arm`**: motor C (otáčení kostky i posun zásobníku)

- `initial`: stav motoru C při startu programu (`rot`: `NONE`/`CW`/`CCW`, `slot`: `LEFT`/`RIGHT`). **Mechanismus musí být před startem fyzicky v tomto stavu.**
- `rot_positions`: polohy otočení v rámci jednoho slotu. Přechod např. z `CCW` do `CW` = rozdíl poloh (−2000°).
- `transitions`: posuny zásobníku ve tvaru `"ROT,SLOT>ROT,SLOT": stupně` (relativní pohyb). Pro každý stav musí existovat posun do druhého slotu, jinak kontrola souboru nahlásí chybu.
- `speed`, `shift_speed`: rychlost otáčení kostky / posunu zásobníku.
- `extra_wait_s`: rezerva časového limitu pohybu. Když motor C nedojede (zasekne se), zabrzdí se a do logu se vypíše `arm: timeout`.

**`pick`**: nabírání kostek

| Klíč | Význam |
|---|---|
| `seen_to_front` | Kterým otočením (`CW`/`CCW`) se **první viděná** polovina kostky natočí k přídi robota. Ověřit na robotu! |
| `fallback_rot` | Otočení, když senzor přečte barvu, která ke kostce nepatří |
| `physical_order` | Pořadí nabraných kostek (1 = první nabraná) v řadě po vyložení, **od přídě dozadu** |
| `shift_after` | Po kolikáté nabrané kostce se posune zásobník (např. `[2]`) |
| `approach_speed` | Rychlost při hledání kostky |
| `first_max_mm` | Jak daleko se hledá první kostka |
| `gap_margin_mm` | Rezerva k rozteči při hledání dalších kostek |
| `wait_clear`, `clear_max_mm` | Před hledáním další kostky nejdřív popojet, dokud senzor neuvidí pozadí |
| `settle_mm` | Popojetí po nalezení kostky před čtením barvy |
| `advance_mm`, `advance_speed` | Popojetí po otočení kostky (zasunutí do robota) |
| `reads` | Počet čtení barvy (vybere se nejčastější) |

### 3.4 `macros` – pojmenované sekvence kroků

Makro je seznam kroků se stejnou syntaxí jako mise. Volá se příkazem `{"cmd": "macro", "name": "…"}`. Makro `unload` používá příkaz `unload`.

### 3.5 `missions` – mise

Mise je seznam kroků. Každý krok má `"cmd"` a parametry. Krok se dá dočasně vypnout přes `"enabled": false`. Řádek jen s `"_"` slouží jako komentář.

---

## 4. Příkazy

| `cmd` | Parametry | Co dělá |
|---|---|---|
| `drive` | `mm`, [`speed`], [`hold`] | Jízda rovně (záporné `mm` = couvání). `hold: false` vypne držení směru gyrem. |
| `drive_time` | `s`, [`speed`] | Jízda daný čas (nouzová varianta jako v `sample.py`) |
| `turn` | `deg`, [`speed`] | Otočení na místě, kladně = doleva |
| `drive_until_colour` | `colour` **nebo** `not`, [`max_mm`], [`speed`], [`reverse`], [`debounce`] | Jede, dokud neuvidí barvu `colour` / dokud neuvidí něco jiného než `not`. Hodnoty: název barvy, seznam barev, `"line"` nebo `"background"`. |
| `follow_line` | `mm`, [`edge`: left/right], [`speed`], [`kp`], [`target`] | Sledování hrany čáry podle odraženého světla |
| `pick_row` | `row`, [`count`], [`reverse`], [`chain_id`], [`append_to`], [`join_end`], [`spacing_mm`] | Nabere kostky z řady (viz níže) |
| `unload` | [`chain_id`], [`macro`] | Spustí makro `unload` a zapamatuje si položený řetěz |
| `arm` | [`rot`], [`slot`] | Nastaví motor C. `slot` může být `LEFT`, `RIGHT` nebo `toggle`. |
| `arm_home` | – | Vrátí motor C do stavu `initial` |
| `macro` | `name` | Spustí makro |
| `wait` | `s` | Čekání |
| `beep` | [`hz`=880], [`ms`=150], [`wait`] | Pípnutí. `wait: true` počká, než tón dohraje. |
| `log` | `msg` | Vypíše text |
| `print_colour` | [`seconds`], [`period`], [`reflected`] | Vypisuje barvu (a odražené světlo) – kalibrace senzoru |

### `pick_row` podrobně

1. Z `field.layouts` vezme barvy kostek v dané řadě (`reverse: true` = robot jede řadou opačným směrem, `count` = kolik kostek nabrat).
2. Spočítá, jak natočit každou kostku, aby po vyložení (podle `physical_order`) barvy navazovaly. Plán vypíše do logu.
3. Pro každou kostku: dojede k ní (senzor uvidí něco jiného než pozadí) → přečte barvu → otočí kostku `CW`/`CCW` tak, aby správná barva mířila k přídi → popojede o `advance_mm` → po kostkách ze `shift_after` posune zásobník.

Natočení se rozhoduje podle **skutečně přečtené barvy**, takže si robot poradí i s kostkami otočenými o 180° (2. kolo).

**Navázání na existující řetěz:**

```json
{ "cmd": "pick_row", "row": "right", "chain_id": "B", "append_to": "A.rear", "join_end": "front" }
```

Plánovač zajistí, aby přední konec (`join_end`) nové řady měl stejnou barvu jako zadní konec (`rear`) řetězu A. Robot pak musí nové kostky **fyzicky** položit k tomuto konci, to je věc trasy v misi.

---

## 5. Postup ladění na robotu

Každý krok ladí jednu věc. Po každém testu upravte jen čísla v `mission.json`.

| # | Mise | Co sledovat | Co upravit |
|---|---|---|---|
| 1 | `test_colour` | Bílá → `WHITE`, čára → `BLACK`, kostky → správné barvy | `colour.background`, výška senzoru |
| 2 | `test_line` | Hodnoty odraženého světla na čáře a vedle ní | `colour.line_target` = průměr obou hodnot |
| 3 | `test_drive` | Ujede přesně 500 mm tam a zpět, rovně | `deg_per_mm`, `left_dir`/`right_dir`, `heading_kp` |
| 4 | `test_turn` | `90` = přesně doleva, `-90` zpět, `360` = celá otočka | `gyro_sign`, `turn_kp`, `turn_tol_deg` |
| 5 | `test_arm` | Otočení kostky CW/CCW a posun zásobníku L/P sedí | `arm.rot_positions`, `arm.transitions`, `arm.initial` |
| 6 | `test_pick` | Dvě kostky z horní řady: najde je, správně otočí | `pick.seen_to_front`, `advance_mm`, `approach_speed` |
| 7 | `test_pick4` | Všechny 4 kostky včetně posunu zásobníku | `pick.shift_after`, `clear_max_mm` |
| 8 | `test_unload` | Kostky leží v řadě těsně za sebou (mezera ≤ 5 mm) | makro `unload`, `pick.physical_order` |
| 9 | `main` | Celá jízda | trasa v misi `main` |

**Tip:** Vložte do mise `{"cmd": "beep"}` mezi kroky. Podle pípnutí poznáte, kde robot právě je.

**Jak ověřit `seen_to_front` a `physical_order`:** Nechte robota nabrat a vyložit 2 kostky (`test_pick` + `test_unload`). Porovnejte skutečné pořadí a natočení s plánem, který se vypsal do logu (`řada (od přídě): [...]`, `front=… rear=…`).
- Je každá kostka obráceně → prohoďte `seen_to_front` (`CW` ↔ `CCW`).
- Jsou kostky v jiném pořadí → upravte `physical_order`.

---

## 6. Rozšíření mise

Druhá jízda je v misi `main` připravená, ale vypnutá (`"enabled": false`). Po odladění první jízdy:

1. Doplňte trasu od místa vyložení k další řadě (`turn`, `drive`, `drive_until_colour` s `"colour": "line"`).
2. U kroků 2. jízdy nastavte `"enabled": true`.
3. Ověřte v simulaci, že se mise vejde do limitu: `python3 main.py --mock` → řádek `Celkový čas`.

**Bodování pro představu:** řetěz 4 kostek = 16 b., dva samostatné řetězy po 4 = 32 b., jeden navázaný řetěz 8 kostek = 64 b.

---

## 7. Řešení problémů

| Příznak | Příčina / řešení |
|---|---|
| `CHYBA: …` při startu | Chyba v `mission.json`, hláška říká kde (např. `missions.main[3]`) |
| Robot se točí na opačnou stranu | `gyro_sign` (nebo `left_dir`/`right_dir`) |
| Robot nejede rovně | `heading_kp`, kalibrace gyra (stát v klidu při startu) |
| `NENALEZENO, max_mm` | Senzor nevidí kostku/čáru: zkontrolujte `test_colour`, zvyšte `max_mm` |
| `! neočekávaná barva` | Senzor čte jinou barvu, než je v `layouts`: kalibrace senzoru nebo jiné rozmístění kostek |
| `arm: timeout/zablokováno` | Motor C se zasekl nebo je pohyb delší, než se čekalo: zkontrolujte `transitions`, `extra_wait_s` |
| `ČAS VYPRŠEL – stop` | Mise je delší než `time_limit_s`: zrychlete nebo zkraťte |
| `GYRO nepřipraveno` | Gyro není připojené nebo neodpovídá: robot pak točí podle enkodérů (`wheel_deg_per_robot_deg`) |

---

## 8. Dokumentace Brian API

- Motory: <https://brian.fel.cvut.cz/cs/motors.html>
- EV3 senzory: <https://brian.fel.cvut.cz/cs/EV3_sensors.html>
- Zvuk: <https://brian.fel.cvut.cz/cs/audio.html>
