# Shuffleboard icon and startup screen

`icon.ico` is the unmodified original WPILib 2020 icon. `icon.png` is its PNG
conversion; `build_mac.sh` produces `icon.icns`. No assets are downloaded by
the build scripts or application at startup.

Source: [WPILibInstaller, files/wpilib-256.ico](https://github.com/wpilibsuite/WPILibInstaller/blob/b471f1540a2652a7f8450dde8484ab64812520b7/files/wpilib-256.ico).
The installer snapshot is from February 24, 2020. Its
[`CreateToolShortcuts` method](https://github.com/wpilibsuite/WPILibInstaller/blob/b471f1540a2652a7f8450dde8484ab64812520b7/DotNet/WPILibInstaller/MainForm.cs#L279-L320)
assigns this icon to the FRC Shuffleboard desktop and Start Menu shortcuts.
The artwork belongs to WPILib and its respective owners.

`splash.html` adapts the 480 × 240 startup screen from
[Shuffleboard v2020.3.2](https://github.com/wpilibsuite/shuffleboard/tree/v2020.3.2/app/src/main):
`Preloader.fxml`, `preloader.css`, `PreloaderController.java`, `HexagonGrid.java`,
and the header styles from `api/base.css`. The blue hexagons, typography and
progress bar follow these originals; the purple `powered by Mapperx` line is
an addition. The version on the splash identifies the source design.
The upstream license is included in `WPILIB_LICENSE.txt` and in built apps.

This application packages MapperX; it is not the official WPILib Java application.
