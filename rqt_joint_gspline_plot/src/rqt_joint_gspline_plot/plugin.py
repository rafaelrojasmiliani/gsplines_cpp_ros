#!/usr/bin/env python
from rqt_gui_py.plugin import Plugin
from .main_widget import MainWidget


class JointGSplinePlot(Plugin):
    def __init__(self, context):
        super(JointGSplinePlot, self).__init__(context)
        self.context = context
        self.setObjectName('JointGSplinePlot')
        # Create a MainWidget
        self.main_widget = MainWidget()
        context.add_widget(self.main_widget)

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def trigger_configuration(self):
        pass

    def shutdown_plugin(self):
        self.main_widget.close()
