# Robot autonome (Eurobot)

Code d'un robot Eurobot **entièrement autonome** : tout tourne sur la carte du robot (Raspberry Pi,
Orange Pi Zero 2W...), sans ordinateur déporté, sans interface web et sans écran. On règle le robot
avec les boutons et `config.json`, on le suit par la console et les LED.

Tiré du code du robot d'Eirbot (stk_robot), sans la base déportée : plus de caméra zénithale, de
serveur web, de lien ZMQ ni d'affichage.

## Contenu

- `Rasp/` : code Python qui tourne sur la carte du robot.
  - `main_robot.py` : point d'entrée. Lance le service LED, le driver LiDAR, le matériel, l'évitement,
    la stratégie et les boutons.
  - `shared.py` : état global (config, état du match, pose du robot).
  - `taches.py` : tâches de fond (chrono du match, alerte batterie, check-list écrite dans la console).
  - `hardware.py` : ESP moteurs et actionneurs, caméra embarquée (créés une fois au démarrage).
  - `core/` : modules purs, testés sur PC sans robot. `match_state.py` (machine d'état du match),
    `config_store.py`, `frames.py` (repères), `localisation.py` (odométrie -> repère table),
    `obstacles.py` (LiDAR -> état d'évitement),
    `preflight.py` (check-list), `vision_geom.py` (tags ArUco de la caméra embarquée).
  - `strat/` : `actions.py` (primitives : `set_pos`, `goto` en ligne droite, symétrie d'équipe, actions...),
    `main_strat.py` (lance la stratégie au départ), `strategies/*.py` (une stratégie par fichier,
    fonction `run(robot)`).
  - `LiDAR/` : `lidar_udp.cpp` (driver RPLiDAR C1 sur `/dev/lidar`, compilé au démarrage) et
    `lidar_thread.py` (évitement : envoie l'état d'obstacle à l'ESP), `voir_lidar.py` (vue terminal).
  - `interface_deplacement/esp_motors.py`, `interface_actionneur/esp_actionneur.py` : liaisons série
    avec l'ESP32 (`/dev/esp`, un seul ESP pour les deux ; fusion des firmwares à faire).
  - `Vision/aruco.py` : caméra embarquée (facultative, `camera.enabled`).
  - `config.json` : réglages du robot.
- `ESP/` : firmware PlatformIO des ESP32 (env `Motor` : moteurs et odométrie, env `Actionneurs`).
  Protocole série des moteurs dans `ESP/src/motors/protocol.h`.
- `tools/` : `robot_sim.py` (robot simulé sur PC), `check_strategy.py` (vérifie une stratégie),
  `run_tests.sh`, `deploy.sh`.
- `test/` : tests Python (sans robot).

## Repère

X = largeur 0..2000 mm, Y = longueur -1500..1500 mm, theta en degrés (0 = X+, 90 = Y+). Les stratégies
sont écrites côté BLEUE ; pour l'équipe JAUNE, le code applique la symétrie y -> -y, theta -> -theta.

## Installation sur la carte

```bash
tools/deploy.sh root@<ip>            # copie Rasp/ dans ~/Rasp (config.json de la carte gardée)
ssh root@<ip>
cd ~/Rasp && ./init.sh               # venv, dépendances, service robot_launcher
```

Périphériques attendus : `/dev/lidar`, `/dev/esp`, à créer par des règles
udev selon les adaptateurs USB-série. Exemple pour le LiDAR (puce CP210x, à vérifier avec `lsusb`) :

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
```

Un seul programme doit lire `/dev/lidar` : `main_robot.py` lance lui-même le driver, il ne faut pas en
garder un autre lancé à côté.

## Tester le LiDAR à la main

```bash
cd ~/Rasp && python3 LiDAR/voir_lidar.py            # en ssh : lance le driver et dessine les points
python3 LiDAR/voir_lidar.py --portee 1000           # zoom sur 1 m
python3 LiDAR/voir_lidar.py --texte                 # une ligne par tour (tours/s, plus proche, évitement)
```

Vue de dessus dans le terminal, robot au centre (`R`), avant vers le haut (`lidar.montage` appliqué) :
un `#` par secteur de 5°, `@` le point le plus proche, cercles `:` (`proche_mm`) et `.` (`seuil_mm`).
La dernière ligne donne l'état d'évitement que le robot enverrait à l'ESP. Pour vérifier le montage, poser
un objet devant le robot : il doit apparaître au-dessus de `R`. `main_robot.py` ne doit pas tourner en
même temps (`./stop.sh`).

## Flasher un ESP32 depuis la carte

`tools/deploy.sh` copie aussi le firmware (`ESP/`) dans `~/Rasp/ESP`. Sur la carte :

```bash
cd ~/Rasp
./flash_esp.sh --ports                  # ports série, identifiants USB, liens /dev/lidar, /dev/esp
./flash_esp.sh moteurs --compiler       # compile seulement (sans ESP branché)
./flash_esp.sh moteurs                  # compile et flashe sur /dev/esp
./flash_esp.sh moteurs /dev/ttyUSB0     # sur un autre port
./flash_esp.sh actionneurs              # firmware actionneurs sur /dev/esp
```

La première fois, le script installe PlatformIO dans `~/.pio-venv` (il faut `python3-venv`) et le
premier build télécharge la chaîne de compilation ESP32 (quelques centaines de Mo, plusieurs minutes).
`main_robot.py` doit être arrêté (`./stop.sh`). Si l'ESP et le LiDAR ont la même puce USB (CP210x
10c4:ea60), la règle udev doit les distinguer par numéro de série (`ATTRS{serial}`, donné par
`./flash_esp.sh --ports`).

## Lancer

```bash
cd ~/Rasp && .venv/bin/python main_robot.py     # ou ./exec.sh ; arrêt : Ctrl+C ou ./stop.sh
```

Avant le match : choix de l'équipe et de la stratégie aux boutons (ou dans `config.json` : `team`,
`static_strat`), tirette insérée puis retirée pour lancer le match (100 s). La console affiche la
check-list (`[CHECK]`) et l'évitement (`[🛑 OBSTACLE]` / `[✅ LIBRE]`). Chaque match est enregistré dans
`logs/telemetry/match_*.jsonl`.

## Tester sur PC

```bash
uv run python -m unittest discover -s test     # ou tools/run_tests.sh
python3 tools/robot_sim.py --strat 3b_pouss_plus --equipe JAUNE
python3 tools/check_strategy.py <stratégie>
```

## À adapter au robot

- `launcher.py` et `buttons_thread.py` utilisent `RPi.GPIO` (numéros de broches Raspberry). Sur une
  autre carte, il faut une autre bibliothèque GPIO et d'autres numéros (`gpio.pins` dans `config.json`).
- `utils/led_service.py` (bandeau LED) et `utils/sensors/ina226_reader.py` (batterie, I2C) : selon le
  matériel présent.
- `lidar.montage` (position et orientation du LiDAR), `lidar.seuil_mm` / `proche_mm` et l'encombrement dans
  `core/frames.py` : dimensions du robot.
- Les stratégies de `Rasp/strat/strategies/` sont des exemples écrits pour le robot d'Eirbot.
