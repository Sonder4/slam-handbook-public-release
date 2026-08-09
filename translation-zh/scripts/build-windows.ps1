[CmdletBinding()]
param(
    [switch]$SkipSite
)

$ErrorActionPreference = 'Stop'
$translationRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = Split-Path -Parent $translationRoot
$python = Get-Command python -ErrorAction SilentlyContinue
if (-not $python) {
    throw 'Python was not found. Install Python 3.10+ and ensure python.exe is on PATH.'
}

$initexmf = Get-Command initexmf -ErrorAction SilentlyContinue
if ($initexmf) {
    & $initexmf.Source --set-config-value '[MPM]AutoInstall=1' 2>$null
}

$inputFiles = @(
    (Join-Path $translationRoot 'content\chapter-1.md'),
    (Join-Path $translationRoot 'content\chapter-2.md')
)
$outputFiles = @(
    (Join-Path $translationRoot 'latex\chapters\chapter-1.tex'),
    (Join-Path $translationRoot 'latex\chapters\chapter-2.tex')
)
$converter = Join-Path $PSScriptRoot 'md_to_latex.py'
& $python.Source $converter --input $inputFiles[0] $inputFiles[1] --output $outputFiles[0] $outputFiles[1] --check
if ($LASTEXITCODE -ne 0) { throw 'Markdown validation failed.' }
& $python.Source $converter --input $inputFiles[0] $inputFiles[1] --output $outputFiles[0] $outputFiles[1]
if ($LASTEXITCODE -ne 0) { throw 'Markdown to LaTeX conversion failed.' }

$xelatex = Get-Command xelatex -ErrorAction SilentlyContinue
if (-not $xelatex) {
    throw 'XeLaTeX was not found. Install MiKTeX (winget install MiKTeX.MiKTeX) or TeX Live, then restart PowerShell.'
}

$latexRoot = Join-Path $translationRoot 'latex'
Push-Location $latexRoot
try {
    for ($pass = 1; $pass -le 2; $pass++) {
        & $xelatex.Source -interaction=nonstopmode -halt-on-error -file-line-error 'main.tex'
        if ($LASTEXITCODE -ne 0) { throw "XeLaTeX compilation failed on pass $pass." }
    }
} finally {
    Pop-Location
}

$pdf = Join-Path $latexRoot 'main.pdf'
if (-not (Test-Path -LiteralPath $pdf)) { throw 'Expected PDF was not generated.' }
$dist = Join-Path $translationRoot 'dist'
$assets = Join-Path $translationRoot 'content\assets'
New-Item -ItemType Directory -Force -Path $dist,$assets | Out-Null
Copy-Item -LiteralPath $pdf -Destination (Join-Path $dist 'slam-handbook-zh-ch1-2.pdf') -Force
Copy-Item -LiteralPath $pdf -Destination (Join-Path $assets 'slam-handbook-zh-ch1-2.pdf') -Force

if (-not $SkipSite) {
    $mkdocs = Get-Command mkdocs -ErrorAction SilentlyContinue
    if (-not $mkdocs) {
        throw 'mkdocs was not found. Install it with: python -m pip install mkdocs-material pymdown-extensions'
    }
    $webPreprocessor = Join-Path $PSScriptRoot 'prepare_web_markdown.py'
    & $python.Source $webPreprocessor --source (Join-Path $translationRoot 'content') --output (Join-Path $translationRoot 'site-docs')
    if ($LASTEXITCODE -ne 0) { throw 'Website Markdown preparation failed.' }

    Push-Location $translationRoot
    try {
        & $mkdocs.Source build --strict --config-file 'mkdocs.yml'
        if ($LASTEXITCODE -ne 0) { throw 'MkDocs build failed.' }
    } finally {
        Pop-Location
    }
}

Write-Host "PDF: $dist\slam-handbook-zh-ch1-2.pdf"
if (-not $SkipSite) { Write-Host "Site: $translationRoot\site" }
