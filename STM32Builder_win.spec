# -*- mode: python ; coding: utf-8 -*-

block_cipher = None

datas = [
    ('toolchain/mac', 'toolchain/mac'),
    ('toolchain/win', 'toolchain/win'),
    ('firmware', 'Accoustic-Controller'),
    ('icons', 'icons'),
]

a = Analysis(
    ['gui.py'],
    pathex=['.'],
    binaries=[],
    datas=datas,
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='STM32Builder',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon='icons/app.ico',
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='STM32Builder',
)

app = BUNDLE(
    coll,
    name='STM32Builder.app',
    icon='icons/app.ico',
    bundle_identifier=None,
)
