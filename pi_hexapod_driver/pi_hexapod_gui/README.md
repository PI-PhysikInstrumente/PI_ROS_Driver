## pi_hexapod_gui

This package contains a the PI Hexapod Control GUI. Implemented as a rqt plugin it provides easy access to various control modes of hexapods with ROS controllers.

A brief description of the relevant directories and their content:
- **resource**: Here is the ui-file located. It has been created with QT-Creator and defines the layout, the components and the general appearance of the plugin.
- **rqt**: This folder contains a pre-configured rqt perspective (gui_mainboard.perspective) known as "PI Hexapod Mainboard".
- **src**: The implementation of the functionality connected to the rqt plugin called PI Hexapod Control GUI is located in this folder.

##### First use:
- After building and sourcing your workspace the plugin (PiGuiPlugin) should be listed in `$rqt --list-plugins`. If not, try `$rqt --force-discover`.
- Now either start `$rqt` and add the plugin via the "Plugins" menu on the top. Or start the PI Hexapod Control GUI plugin as a standalone via `$rqt --standalone pi_hexapod_gui`. With the second option you do not have the possibility to add other plugins to your perspective.

##### Using the PI Hexapod Mainboard:
- This preconfigured rqt-perspective can be started via the "Perspectives" menu on the top of rqt. Select "Import" and navigate to the corresponding file.

---

### License

Please consider the license information provided with this repository.
