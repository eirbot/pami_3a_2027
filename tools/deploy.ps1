# Version Windows de deploy.sh : copie le code Rasp/ (et le firmware ESP/ dans Rasp/ESP) sur la carte
# du robot, en gardant sa config.json et ses journaux.
# Usage : powershell -ExecutionPolicy Bypass -File tools\deploy.ps1 utilisateur@ip [dossier distant, défaut : ~/Rasp]
# Nécessite ssh et tar, fournis avec Windows 10/11 (pas besoin de rsync) : un seul envoi par ssh
# (une seule demande de mot de passe). Contrairement à rsync, tous les fichiers sont renvoyés à chaque fois.
# Le driver LiDAR (LiDAR/lidar_udp) est recompilé sur la carte par main_robot.py au démarrage.
param(
    [Parameter(Mandatory = $true, Position = 0)][string]$Cible,
    [Parameter(Position = 1)][string]$Dossier = "Rasp"
)
$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$Archive = Join-Path ([System.IO.Path]::GetTempPath()) "deploy_robot_$PID.tar"

try {
    # Une seule archive : Rasp/ à la racine, ESP/ dans ESP/. La config.json du dépôt est renommée
    # config.json.depot ; la carte ne la garde que si elle n'a pas déjà sa config.json.
    & tar -cf $Archive `
        --exclude=.venv --exclude=__pycache__ --exclude=*.pyc --exclude=*.log `
        --exclude=./logs --exclude=./LiDAR/lidar_udp --exclude=.pio --exclude=.vscode `
        -s ",^\./config\.json$,./config.json.depot," `
        -C (Join-Path $Root "Rasp") . `
        -C $Root ESP
    if ($LASTEXITCODE -ne 0) { throw "Échec de la création de l'archive (tar)" }

    # Côté carte : extraction, config.json conservée, retrait des fins de ligne Windows (CRLF) des
    # scripts au cas où git les aurait converties au clonage (sinon bash/python refusent de les lancer),
    # et droit d'exécution des .sh (perdu en passant par Windows).
    $Distant = "mkdir -p '$Dossier' && cd '$Dossier' && tar -xvf - --no-same-owner " +
        "&& { [ -e config.json ] || mv config.json.depot config.json; rm -f config.json.depot; } " +
        "&& find . -path ./.venv -prune -o \( -name '*.sh' -o -name '*.py' \) -type f -exec sed -i 's/\r`$//' {} + " +
        "&& find . -path ./.venv -prune -o -name '*.sh' -type f -exec chmod +x {} +"

    # Redirection par cmd : le pipeline de PowerShell 5 abîme les données binaires.
    & cmd /c "ssh $Cible `"$Distant`" < `"$Archive`""
    if ($LASTEXITCODE -ne 0) { throw "Échec de l'envoi sur $Cible" }

    Write-Host "Code copié dans ${Cible}:$Dossier (config.json de la carte conservée si elle existait)"
}
finally {
    Remove-Item -ErrorAction SilentlyContinue $Archive
}
