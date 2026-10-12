param([string]$pick = "p50")
$root = Split-Path $PSScriptRoot -Parent
$p = (wsl wslpath -a ($root.Replace('\','/'))).Trim()
wsl bash "$p/scripts/pen.sh" $pick
