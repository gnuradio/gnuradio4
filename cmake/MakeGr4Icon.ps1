<#
    .SYNOPSIS
        Builds the installer icon from the GNU Radio logo.

    .DESCRIPTION
        Written for the unified installer, which needs a .ico and has only a
        512x466 PNG to work with.  The logo is padded to a square rather than
        stretched, and every size is stored as a PNG payload, which .ico has
        allowed since Vista.  Uses System.Drawing so the build needs no image
        tooling beyond Windows itself.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string] $Logo,
    [Parameter(Mandatory = $true)][string] $OutFile
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

Add-Type -AssemblyName System.Drawing

# 256 is what Explorer shows in its largest view; the rest are the sizes
# Windows actually picks from, so leaving them out forces it to downscale
$sizes = @(16, 24, 32, 48, 64, 128, 256)

$source = [System.Drawing.Image]::FromFile((Resolve-Path -LiteralPath $Logo).Path)
try {
    $payloads = New-Object System.Collections.Generic.List[byte[]]
    foreach ($size in $sizes) {
        $bitmap = New-Object System.Drawing.Bitmap($size, $size, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
        try {
            $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
            try {
                $graphics.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
                $graphics.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::HighQuality
                $graphics.Clear([System.Drawing.Color]::Transparent)

                # fit inside the square, centred, aspect ratio preserved
                $scale = [Math]::Min($size / $source.Width, $size / $source.Height)
                $width = [int][Math]::Round($source.Width * $scale)
                $height = [int][Math]::Round($source.Height * $scale)
                $graphics.DrawImage($source, [int](($size - $width) / 2), [int](($size - $height) / 2), $width, $height)
            } finally {
                $graphics.Dispose()
            }

            $stream = New-Object System.IO.MemoryStream
            try {
                $bitmap.Save($stream, [System.Drawing.Imaging.ImageFormat]::Png)
                $payloads.Add($stream.ToArray())
            } finally {
                $stream.Dispose()
            }
        } finally {
            $bitmap.Dispose()
        }
    }
} finally {
    $source.Dispose()
}

$directory = Split-Path -Parent $OutFile
if ($directory -and -not (Test-Path -LiteralPath $directory)) {
    New-Item -ItemType Directory -Path $directory -Force | Out-Null
}

# ICONDIR, then one 16-byte ICONDIRENTRY each, then the PNG payloads
$output = New-Object System.IO.MemoryStream
$writer = New-Object System.IO.BinaryWriter($output)
try {
    $writer.Write([UInt16] 0)
    $writer.Write([UInt16] 1)
    $writer.Write([UInt16] $sizes.Count)

    $offset = 6 + 16 * $sizes.Count
    for ($index = 0; $index -lt $sizes.Count; $index++) {
        $size = $sizes[$index]
        # 256 is stored as 0: the field is one byte wide
        $dimension = if ($size -ge 256) { 0 } else { $size }
        $writer.Write([byte] $dimension)
        $writer.Write([byte] $dimension)
        $writer.Write([byte] 0)
        $writer.Write([byte] 0)
        $writer.Write([UInt16] 1)
        $writer.Write([UInt16] 32)
        $writer.Write([UInt32] $payloads[$index].Length)
        $writer.Write([UInt32] $offset)
        $offset += $payloads[$index].Length
    }
    foreach ($payload in $payloads) {
        $writer.Write($payload)
    }
    $writer.Flush()
    [System.IO.File]::WriteAllBytes($OutFile, $output.ToArray())
} finally {
    $writer.Dispose()
    $output.Dispose()
}

Write-Output "Wrote $OutFile ($($sizes -join ', '))"
