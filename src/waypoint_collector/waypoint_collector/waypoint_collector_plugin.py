from rqt_gui_py.plugin import Plugin
from .waypoint_collector_widget import WaypointCollectorWidget


class WaypointCollectorPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('WaypointCollectorPlugin')

        self._widget = WaypointCollectorWidget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f'{self._widget.windowTitle()} ({context.serial_number()})'
            )
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('data_dir', self._widget.data_dir)

    def restore_settings(self, plugin_settings, instance_settings):
        data_dir = instance_settings.value('data_dir', self._widget.data_dir)
        self._widget.set_data_dir(data_dir)
