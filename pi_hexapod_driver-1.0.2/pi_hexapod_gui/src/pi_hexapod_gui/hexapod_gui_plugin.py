#!/usr/bin/env python2.7

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This file is part of the PI-Hexapod Driver.
#
#
# © Copyright 2020 Physik Instrumente (PI) GmbH & Co. KG, Karlsruhe, Germany
# © Copyright 2020 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------

# This file contains the implementation of the corresponing RQT plugin functionality.

import os
import time
import math
import threading

from random import randint
import rospy
import rospkg

import controller_manager_msgs.srv

import actionlib
import pi_hexapod_msgs.msg

import control_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import std_srvs.srv

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class PiGuiPlugin(Plugin):
    """This class corresponds to the RQT plugin (see plugin.xml)"""

    def __init__(self, context):
        super(PiGuiPlugin, self).__init__(context)
        print(__name__)

        self.setObjectName("PiGuiPlugin")

        # Create empty QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(
            rospkg.RosPack().get_path("pi_hexapod_gui"), "resource", "pi_hexapod_gui.ui"
        )
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName("PiGuiPluginUi")
        # Show an enumerated WindowTitle on left-top of each plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                ("PI Hexapod Control GUI (%d)" % context.serial_number())
            )
        else:
            self._widget.setWindowTitle(("PI Hexapod Control GUI"))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Create ROS context, required to access ROS functionalities (e.g. topics and services)
        try:
            rospy.init_node("pi_hexapod_gui_node")
        except rospy.ROSException:
            pass

        self._config_highlight_style_sheet = "background-color: rgba(46, 255, 112, 40);"

        # Controller related configurations
        self._name_service_list_controllers = "/controller_manager/list_controllers"
        self._name_service_switch_controller = "/controller_manager/switch_controller"
        self._name_no_controller = "- none -"
        self._list_blocked_controllers = [
            "joint_state_controller",
            "cart_x_controller",
            "cart_y_controller",
            "cart_z_controller",
            "ang_u_controller",
            "ang_v_controller",
            "ang_w_controller",
        ]
        rospy.wait_for_service(self._name_service_list_controllers, 30)
        self._srv_list_controllers = rospy.ServiceProxy(
            self._name_service_list_controllers,
            controller_manager_msgs.srv.ListControllers,
        )
        self._srv_switch_controller = rospy.ServiceProxy(
            self._name_service_switch_controller,
            controller_manager_msgs.srv.SwitchController,
        )
        initialized_controllers = self._srv_list_controllers.call().controller
        items = []
        for c in initialized_controllers:
            if c.name not in self._list_blocked_controllers:
                items.append(c.name)
        self._widget.comboBox_controller.addItems([self._name_no_controller])
        self._widget.comboBox_controller.addItems(items)
        self._currentController = self._name_no_controller
        self._widget.comboBox_controller.currentIndexChanged.connect(
            self.change_controller
        )

        # Driver Status Visualization
        self._name_topic_error_id = "/pi_hardware_interface/error_id"
        self._name_topic_error_msg = "/pi_hardware_interface/error_msg"
        self._name_no_error = "No error"
        self._widget.lcdNumber_error_id.setDigitCount(5)
        self._widget.label_error_msg.setText(self._name_no_error)
        self._widget.label_error_msg.setWordWrap(True)
        self._sub_error_id = rospy.Subscriber(
            self._name_topic_error_id, std_msgs.msg.Int32, self.error_id_callback
        )
        self._sub_error_msg = rospy.Subscriber(
            self._name_topic_error_msg, std_msgs.msg.String, self.error_msg_callback
        )

        # Referencing
        self._action_client_referencing = actionlib.SimpleActionClient(
            "/pi_hardware_interface/referencing", pi_hexapod_msgs.msg.ReferencingAction,
        )
        self._widget.pushButton_reference.setEnabled(False)
        self._widget.pushButton_reference.clicked.connect(self.reference_clicked)
        self.reference_clicked()

        # Return to Home Position
        self._widget.pushButton_return_home.setEnabled(False)
        self._widget.pushButton_return_home.clicked.connect(self.return_home_clicked)

        # Test Motions (additional functionality is located in the controller-switch-callback)
        self._name_test_motion_jgp = "Test-Motion (Sine)"
        self._widget.pushButton_test_motion.setEnabled(False)
        self._widget.pushButton_test_motion.clicked.connect(self.test_motion_clicked)
        # This thread executes some of the test motions
        self._sine_thread = None
        self._trajectory_thread = None

        # Resume Control
        self._name_service_enable_control = "/pi_hardware_interface/enable_control"
        rospy.wait_for_service(self._name_service_enable_control, 3)
        self._srv_enable_control = rospy.ServiceProxy(
            self._name_service_enable_control, std_srvs.srv.SetBool
        )
        req = std_srvs.srv.SetBoolRequest()
        req.data = True
        res = self._srv_enable_control.call(req)
        if res.success:
            self._widget.pushButton_enable.setEnabled(False)
        self._widget.pushButton_enable.clicked.connect(self.enable_clicked)
        self._widget.pushButton_enable.setText("Resume Control")

        # Halt Hexapod
        self._name_service_halt_hexapod = "/pi_hardware_interface/halt_hexapod"
        rospy.wait_for_service(self._name_service_halt_hexapod, 3)
        self._srv_halt_hexapod = rospy.ServiceProxy(
            self._name_service_halt_hexapod, std_srvs.srv.Trigger
        )
        self._widget.pushButton_halt.clicked.connect(self.halt_clicked)

        # Stop Hexapod
        self._name_service_stop_hexapod = "/pi_hardware_interface/stop_hexapod"
        rospy.wait_for_service(self._name_service_stop_hexapod, 3)
        self._srv_stop_hexapod = rospy.ServiceProxy(
            self._name_service_stop_hexapod, std_srvs.srv.Trigger
        )
        self._widget.pushButton_stop.clicked.connect(self.stop_clicked)

        # Slider Control (resolutions can be adjusted here if needed)

        self._topic_name_joint_traj_controller_command = (
            "/joint_trajectory_controller/command"
        )
        self._topic_name_joint_group_pos_controller_command = (
            "/joint_group_pos_controller/command"
        )
        self._resolutions_trans = [1.0, 0.1, 0.01, 0.001]
        self._resolutions_rot = [1.0, 0.1, 0.01, 0.001]
        # The sliders work on a integer scale, since the smallest resolution defines it
        self._factor_min_trans = 1.0 / min(self._resolutions_trans)
        self._factor_min_rot = 1.0 / min(self._resolutions_rot)

        # For the joint_group_position controller the release event is required
        self._widget.horizontalSlider_x.valueChanged.connect(self.manage_slider_changes)
        self._widget.horizontalSlider_y.valueChanged.connect(self.manage_slider_changes)
        self._widget.horizontalSlider_z.valueChanged.connect(self.manage_slider_changes)
        self._widget.horizontalSlider_u.valueChanged.connect(self.manage_slider_changes)
        self._widget.horizontalSlider_v.valueChanged.connect(self.manage_slider_changes)
        self._widget.horizontalSlider_w.valueChanged.connect(self.manage_slider_changes)
        # For the trajectory controller the release event is required
        self._widget.horizontalSlider_x.sliderReleased.connect(
            self.manage_slider_changes
        )
        self._widget.horizontalSlider_y.sliderReleased.connect(
            self.manage_slider_changes
        )
        self._widget.horizontalSlider_z.sliderReleased.connect(
            self.manage_slider_changes
        )
        self._widget.horizontalSlider_u.sliderReleased.connect(
            self.manage_slider_changes
        )
        self._widget.horizontalSlider_v.sliderReleased.connect(
            self.manage_slider_changes
        )
        self._widget.horizontalSlider_w.sliderReleased.connect(
            self.manage_slider_changes
        )
        # These parameters are important for the slider limits
        self._x_lower_lim = rospy.get_param("/pi_hardware_interface/x/lower_limit")
        self._y_lower_lim = rospy.get_param("/pi_hardware_interface/y/lower_limit")
        self._z_lower_lim = rospy.get_param("/pi_hardware_interface/z/lower_limit")
        self._u_lower_lim = rospy.get_param("/pi_hardware_interface/u/lower_limit")
        self._v_lower_lim = rospy.get_param("/pi_hardware_interface/v/lower_limit")
        self._w_lower_lim = rospy.get_param("/pi_hardware_interface/w/lower_limit")
        self._x_upper_lim = rospy.get_param("/pi_hardware_interface/x/upper_limit")
        self._y_upper_lim = rospy.get_param("/pi_hardware_interface/y/upper_limit")
        self._z_upper_lim = rospy.get_param("/pi_hardware_interface/z/upper_limit")
        self._u_upper_lim = rospy.get_param("/pi_hardware_interface/u/upper_limit")
        self._v_upper_lim = rospy.get_param("/pi_hardware_interface/v/upper_limit")
        self._w_upper_lim = rospy.get_param("/pi_hardware_interface/w/upper_limit")
        new_step = max(self._resolutions_trans) * self._factor_min_trans
        self._widget.horizontalSlider_x.setSingleStep(new_step)
        self._widget.horizontalSlider_y.setSingleStep(new_step)
        self._widget.horizontalSlider_z.setSingleStep(new_step)
        self._widget.horizontalSlider_x.setTickInterval(new_step)
        self._widget.horizontalSlider_y.setTickInterval(new_step)
        self._widget.horizontalSlider_z.setTickInterval(new_step)
        new_step = max(self._resolutions_rot) * self._factor_min_rot
        self._widget.horizontalSlider_u.setSingleStep(new_step)
        self._widget.horizontalSlider_v.setSingleStep(new_step)
        self._widget.horizontalSlider_w.setSingleStep(new_step)
        self._widget.horizontalSlider_u.setTickInterval(new_step)
        self._widget.horizontalSlider_v.setTickInterval(new_step)
        self._widget.horizontalSlider_w.setTickInterval(new_step)
        self._widget.horizontalSlider_x.setMinimum(
            self._x_lower_lim * 1000.0 * self._factor_min_trans
        )
        self._widget.horizontalSlider_y.setMinimum(
            self._y_lower_lim * 1000.0 * self._factor_min_trans
        )
        self._widget.horizontalSlider_z.setMinimum(
            self._z_lower_lim * 1000.0 * self._factor_min_trans
        )
        self._widget.horizontalSlider_u.setMinimum(
            math.degrees(self._u_lower_lim) * self._factor_min_rot
        )
        self._widget.horizontalSlider_v.setMinimum(
            math.degrees(self._v_lower_lim) * self._factor_min_rot
        )
        self._widget.horizontalSlider_w.setMinimum(
            math.degrees(self._w_lower_lim) * self._factor_min_rot
        )
        self._widget.horizontalSlider_x.setMaximum(
            self._x_upper_lim * 1000.0 * self._factor_min_trans
        )
        self._widget.horizontalSlider_y.setMaximum(
            self._y_upper_lim * 1000.0 * self._factor_min_trans
        )
        self._widget.horizontalSlider_z.setMaximum(
            self._z_upper_lim * 1000.0 * self._factor_min_trans
        )
        self._widget.horizontalSlider_u.setMaximum(
            math.degrees(self._u_upper_lim) * self._factor_min_rot
        )
        self._widget.horizontalSlider_v.setMaximum(
            math.degrees(self._v_upper_lim) * self._factor_min_rot
        )
        self._widget.horizontalSlider_w.setMaximum(
            math.degrees(self._w_upper_lim) * self._factor_min_rot
        )
        self._widget.comboBox_trans_step.addItems(map(str, self._resolutions_trans))
        self._widget.comboBox_trans_step.currentIndexChanged.connect(
            self.trans_step_changed
        )
        self._widget.comboBox_rot_step.addItems(map(str, self._resolutions_rot))
        self._widget.comboBox_rot_step.currentIndexChanged.connect(
            self.rot_step_changed
        )

        self._widget.lineEdit_x.setText(
            str(self._widget.horizontalSlider_x.value() * self._factor_min_trans)
        )
        self._widget.lineEdit_y.setText(
            str(self._widget.horizontalSlider_y.value() * self._factor_min_trans)
        )
        self._widget.lineEdit_z.setText(
            str(self._widget.horizontalSlider_z.value() * self._factor_min_trans)
        )
        self._widget.lineEdit_u.setText(
            str(self._widget.horizontalSlider_x.value() * self._factor_min_rot)
        )
        self._widget.lineEdit_v.setText(
            str(self._widget.horizontalSlider_y.value() * self._factor_min_rot)
        )
        self._widget.lineEdit_w.setText(
            str(self._widget.horizontalSlider_z.value() * self._factor_min_rot)
        )
        self._widget.lineEdit_x.returnPressed.connect(self.trans_x_changed)
        self._widget.lineEdit_y.returnPressed.connect(self.trans_y_changed)
        self._widget.lineEdit_z.returnPressed.connect(self.trans_z_changed)
        self._widget.lineEdit_u.returnPressed.connect(self.trans_u_changed)
        self._widget.lineEdit_v.returnPressed.connect(self.trans_v_changed)
        self._widget.lineEdit_w.returnPressed.connect(self.trans_w_changed)
        self._pub_traj_command = rospy.Publisher(
            self._topic_name_joint_traj_controller_command,
            trajectory_msgs.msg.JointTrajectory,
            queue_size=1,
        )
        self._pub_gpos_command = rospy.Publisher(
            self._topic_name_joint_group_pos_controller_command,
            std_msgs.msg.Float64MultiArray,
            queue_size=1,
        )
        self._pub_traj_command_seq = 0

        rospy.loginfo("Plugin successfully initialized.")

    def trans_x_changed(self):
        try:
            f = float(self._widget.lineEdit_x.text())
            self._widget.horizontalSlider_x.setValue(f * self._factor_min_trans)
        except Exception as e:
            pass

    def trans_y_changed(self):
        try:
            f = float(self._widget.lineEdit_y.text())
            self._widget.horizontalSlider_y.setValue(f * self._factor_min_trans)
        except Exception as e:
            pass

    def trans_z_changed(self):
        try:
            f = float(self._widget.lineEdit_z.text())
            self._widget.horizontalSlider_z.setValue(f * self._factor_min_trans)
        except Exception as e:
            pass

    def trans_u_changed(self):
        try:
            f = float(self._widget.lineEdit_u.text())
            self._widget.horizontalSlider_u.setValue(f * self._factor_min_rot)
        except Exception as e:
            pass

    def trans_v_changed(self):
        try:
            f = float(self._widget.lineEdit_v.text())
            self._widget.horizontalSlider_v.setValue(f * self._factor_min_rot)
        except Exception as e:
            pass

    def trans_w_changed(self):
        try:
            f = float(self._widget.lineEdit_w.text())
            self._widget.horizontalSlider_w.setValue(f * self._factor_min_rot)
        except Exception as e:
            pass

    def trans_step_changed(self):
        new_step = (
            float(self._widget.comboBox_trans_step.currentText())
            * self._factor_min_trans
        )
        self._widget.horizontalSlider_x.setSingleStep(new_step)
        self._widget.horizontalSlider_y.setSingleStep(new_step)
        self._widget.horizontalSlider_z.setSingleStep(new_step)

    def rot_step_changed(self):
        new_step = (
            float(self._widget.comboBox_rot_step.currentText()) * self._factor_min_rot
        )
        self._widget.horizontalSlider_u.setSingleStep(new_step)
        self._widget.horizontalSlider_v.setSingleStep(new_step)
        self._widget.horizontalSlider_w.setSingleStep(new_step)

    def error_id_callback(self, error_id_msg):
        """The PI Hardware publishes the error ID when it changes"""
        self._widget.lcdNumber_error_id.display(error_id_msg.data)

    def error_msg_callback(self, error_msg_msg):
        """The PI Hardware publishes the error message when the error ID changes"""
        self._widget.label_error_msg.setText(error_msg_msg.data)

    def enable_clicked(self):
        req = std_srvs.srv.SetBoolRequest()
        req.data = True
        res = self._srv_enable_control.call(req)
        if res.success:
            self._widget.pushButton_enable.setEnabled(False)
            self._widget.pushButton_enable.setStyleSheet("")

    def halt_clicked(self):
        self._srv_halt_hexapod.call()
        self._widget.pushButton_enable.setEnabled(True)
        self._widget.pushButton_enable.setStyleSheet(self._config_highlight_style_sheet)

    def stop_clicked(self):
        self._srv_stop_hexapod.call()
        self._widget.pushButton_enable.setEnabled(True)
        self._widget.pushButton_enable.setStyleSheet(self._config_highlight_style_sheet)

    def _isAnySliderPressed(self):
        x_p = self._widget.horizontalSlider_x.isSliderDown()
        y_p = self._widget.horizontalSlider_y.isSliderDown()
        z_p = self._widget.horizontalSlider_z.isSliderDown()
        u_p = self._widget.horizontalSlider_u.isSliderDown()
        v_p = self._widget.horizontalSlider_v.isSliderDown()
        w_p = self._widget.horizontalSlider_w.isSliderDown()
        return x_p or y_p or z_p or u_p or v_p or w_p

    def manage_slider_changes(self):
        # When the sliders change the line-edit-field has to be updated and a command
        # for the currently chosen controller is sent
        self._widget.lineEdit_x.setText(
            "{0}".format(
                self._widget.horizontalSlider_x.value() / self._factor_min_trans
            )
        )
        self._widget.lineEdit_y.setText(
            "{0}".format(
                self._widget.horizontalSlider_y.value() / self._factor_min_trans
            )
        )
        self._widget.lineEdit_z.setText(
            "{0}".format(
                self._widget.horizontalSlider_z.value() / self._factor_min_trans
            )
        )
        self._widget.lineEdit_u.setText(
            "{0}".format(self._widget.horizontalSlider_u.value() / self._factor_min_rot)
        )
        self._widget.lineEdit_v.setText(
            "{0}".format(self._widget.horizontalSlider_v.value() / self._factor_min_rot)
        )
        self._widget.lineEdit_w.setText(
            "{0}".format(self._widget.horizontalSlider_w.value() / self._factor_min_rot)
        )
        if self._currentController == "joint_trajectory_controller":
            # Sending too many trajectory command messages would cause errors.
            # Therefore, the command is sent when the sliders are released.
            if self._isAnySliderPressed():
                # Only log every second
                rospy.loginfo_once("Release slider for the command to be sent.")
                return
            trajectory_msg = trajectory_msgs.msg.JointTrajectory()
            trajectory_msg.header.seq = self._pub_traj_command_seq
            point_msg = trajectory_msgs.msg.JointTrajectoryPoint()
            point_msg.positions = [
                self._widget.horizontalSlider_x.value()
                / 1000.0
                / self._factor_min_trans,
                self._widget.horizontalSlider_y.value()
                / 1000.0
                / self._factor_min_trans,
                self._widget.horizontalSlider_z.value()
                / 1000.0
                / self._factor_min_trans,
                math.radians(
                    self._widget.horizontalSlider_u.value() / self._factor_min_rot
                ),
                math.radians(
                    self._widget.horizontalSlider_v.value() / self._factor_min_rot
                ),
                math.radians(
                    self._widget.horizontalSlider_w.value() / self._factor_min_rot
                ),
            ]
            point_msg.time_from_start = rospy.Duration(1.0)
            trajectory_msg.points = [point_msg]
            trajectory_msg.joint_names = [
                "cart_x",
                "cart_y",
                "cart_z",
                "ang_u",
                "ang_v",
                "ang_w",
            ]
            self._pub_traj_command.publish(trajectory_msg)
            self._pub_traj_command_seq += 1
        elif self._currentController == "joint_group_pos_controller":
            array_msg = std_msgs.msg.Float64MultiArray()
            array_msg.data = [
                self._widget.horizontalSlider_x.value()
                / 1000.0
                / self._factor_min_trans,
                self._widget.horizontalSlider_y.value()
                / 1000.0
                / self._factor_min_trans,
                self._widget.horizontalSlider_z.value()
                / 1000.0
                / self._factor_min_trans,
                math.radians(
                    self._widget.horizontalSlider_u.value() / self._factor_min_rot
                ),
                math.radians(
                    self._widget.horizontalSlider_v.value() / self._factor_min_rot
                ),
                math.radians(
                    self._widget.horizontalSlider_w.value() / self._factor_min_rot
                ),
            ]
            self._pub_gpos_command.publish(array_msg)

    def _update_position_to_real_position(self):
        joint_state_msg = rospy.wait_for_message(
            "/joint_states", sensor_msgs.msg.JointState, 3.0
        )
        idx_x = joint_state_msg.name.index("cart_x")
        idx_y = joint_state_msg.name.index("cart_y")
        idx_z = joint_state_msg.name.index("cart_z")
        idx_u = joint_state_msg.name.index("ang_u")
        idx_v = joint_state_msg.name.index("ang_v")
        idx_w = joint_state_msg.name.index("ang_w")
        self._widget.horizontalSlider_x.setValue(
            round(joint_state_msg.position[idx_x] * 1000.0, 3) * self._factor_min_trans
        )
        self._widget.horizontalSlider_y.setValue(
            round(joint_state_msg.position[idx_y] * 1000.0, 3) * self._factor_min_trans
        )
        self._widget.horizontalSlider_z.setValue(
            round(joint_state_msg.position[idx_z] * 1000.0, 3) * self._factor_min_trans
        )
        self._widget.horizontalSlider_u.setValue(
            round(math.degrees(joint_state_msg.position[idx_u]), 3)
            * self._factor_min_rot
        )
        self._widget.horizontalSlider_v.setValue(
            round(math.degrees(joint_state_msg.position[idx_v]), 3)
            * self._factor_min_rot
        )
        self._widget.horizontalSlider_w.setValue(
            round(math.degrees(joint_state_msg.position[idx_w]), 3)
            * self._factor_min_rot
        )

    def change_controller(self):
        """ This method is called when the user changed the controller type """
        switch_request = controller_manager_msgs.srv.SwitchControllerRequest()
        switch_request.start_controllers = [
            self._widget.comboBox_controller.currentText()
        ]
        switch_request.stop_controllers = [self._currentController]
        switch_request.strictness = 1
        response = self._srv_switch_controller.call(switch_request)
        if response:
            self._currentController = self._widget.comboBox_controller.currentText()
            if (
                self._widget.comboBox_controller.currentText()
                != self._name_no_controller
            ):
                if (
                    self._widget.comboBox_controller.currentText()
                    == "joint_trajectory_controller"
                ):
                    self._widget.pushButton_test_motion.setText(
                        "Test-Motion (Trajectory)"
                    )
                else:
                    self._widget.pushButton_test_motion.setText(
                        self._name_test_motion_jgp
                    )

                self._widget.pushButton_reference.setEnabled(False)
                self._widget.pushButton_return_home.setEnabled(True)
                self._widget.pushButton_test_motion.setEnabled(True)
            else:
                self._widget.pushButton_reference.setEnabled(True)
                self._widget.pushButton_return_home.setEnabled(False)
                self._widget.pushButton_test_motion.setEnabled(False)
        else:
            self._widget.comboBox_controller.SetCurrentText(self._name_no_controller)
            self._widget.pushButton_reference.setEnabled(True)
            self._widget.pushButton_return_home.setEnabled(False)
            self._widget.pushButton_test_motion.setEnabled(False)
        self._update_position_to_real_position()

    def reference_clicked(self):
        self._widget.pushButton_reference.setEnabled(False)
        self._action_client_referencing.wait_for_server(rospy.Duration(3.0))
        self._action_client_referencing.send_goal(
            pi_hexapod_msgs.msg.ReferencingActionGoal()
        )
        self._action_client_referencing.wait_for_result(rospy.Duration(7.0))
        result = self._action_client_referencing.get_result()
        self._widget.label_referenced.setText(str(result.success))
        self._widget.pushButton_reference.setEnabled(True)

    def return_home_clicked(self):
        self._widget.horizontalSlider_x.setValue(0)
        self._widget.horizontalSlider_y.setValue(0)
        self._widget.horizontalSlider_z.setValue(0)
        self._widget.horizontalSlider_u.setValue(0)
        self._widget.horizontalSlider_v.setValue(0)
        self._widget.horizontalSlider_w.setValue(0)
        self.manage_slider_changes()

    def _execute_sine_test(self, idx_var):
        # In the following a sine motion is sent out to a random joint/coordinate.
        array_msg = std_msgs.msg.Float64MultiArray()
        array_msg.data = [
            0.0 / 1000.0,
            0.0 / 1000.0,
            0.0 / 1000.0,
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
        ]
        max_vals = [
            self._x_upper_lim,  # [m]
            self._y_upper_lim,  # [m]
            self._z_upper_lim,  # [m]
            self._u_upper_lim,  # [rad]
            self._v_upper_lim,  # [rad]
            self._w_upper_lim,  # [rad]
        ]
        rospy.loginfo("Test-Motion with random-axis: %d", idx_var)
        # The command publishing rate in Hz
        command_rate = 20
        # Command rate times the duration in seconds determines the number of steps
        motion_steps = command_rate * 15
        r = rospy.Rate(command_rate)
        for i in range(motion_steps + 1):
            if idx_var < 3:
                # translational coordinates (reduced amplitude to not exceed speed limit)
                array_msg.data[idx_var] = (
                    math.sin(i * 1.0 / motion_steps * 2.0 * math.pi)
                    * max_vals[idx_var]
                    * 0.25
                )
            else:
                # rotational coordinates (reduced amplitude to not exceed speed limit)
                array_msg.data[idx_var] = (
                    math.sin(i * 1.0 / motion_steps * 2.0 * math.pi)
                    * max_vals[idx_var]
                    * 0.25
                )
            self._pub_gpos_command.publish(array_msg)
            r.sleep()
        # Make sure latest command has time to be accomplished
        time.sleep(1.0)
        self._widget.pushButton_test_motion.setText(self._name_test_motion_jgp)
        self._widget.pushButton_test_motion.setEnabled(True)
        self._widget.pushButton_return_home.setEnabled(True)
        self._widget.comboBox_controller.setEnabled(True)
        self._widget.pushButton_test_motion.setStyleSheet("")
        self._update_position_to_real_position()

    def _execute_trajectory_test(self):
        # In the following an example trajectory message is created.
        trajectory_msg = trajectory_msgs.msg.JointTrajectory()
        trajectory_msg.header.seq = self._pub_traj_command_seq
        point_msg_0 = trajectory_msgs.msg.JointTrajectoryPoint()
        t_0 = 5.0
        t_1 = 5.0
        t_2 = 5.0
        t_3 = 5.0
        t_4 = 5.0
        point_msg_0.positions = [
            0.0 / 1000.0,
            0.0 / 1000.0,
            0.0 / 1000.0,
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
        ]
        point_msg_0.time_from_start = rospy.Duration(t_0)
        point_msg_1 = trajectory_msgs.msg.JointTrajectoryPoint()
        point_msg_1.positions = [
            8.0 / 1000.0,
            0.0 / 1000.0,
            -1.4 / 1000.0,
            math.radians(0.0),
            math.radians(-2.0),
            math.radians(0.0),
        ]
        point_msg_1.time_from_start = rospy.Duration(t_0 + t_1)
        point_msg_2 = trajectory_msgs.msg.JointTrajectoryPoint()
        point_msg_2.positions = [
            0.0 / 1000.0,
            0.0 / 1000.0,
            0.0 / 1000.0,
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
        ]
        point_msg_2.time_from_start = rospy.Duration(t_0 + t_1 + t_2)
        point_msg_3 = trajectory_msgs.msg.JointTrajectoryPoint()
        point_msg_3.positions = [
            4.8 / 1000.0,
            -4.1 / 1000.0,
            -2.8 / 1000.0,
            math.radians(0.0),
            math.radians(0.0),
            math.radians(5.0),
        ]
        point_msg_3.time_from_start = rospy.Duration(t_0 + t_1 + t_2 + t_3)
        point_msg_4 = trajectory_msgs.msg.JointTrajectoryPoint()
        point_msg_4.positions = [
            0.0 / 1000.0,
            0.0 / 1000.0,
            0.0 / 1000.0,
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
        ]
        point_msg_4.time_from_start = rospy.Duration(t_0 + t_1 + t_2 + t_3 + t_4)
        trajectory_msg.points = [
            point_msg_0,
            point_msg_1,
            point_msg_2,
            point_msg_3,
            point_msg_4,
        ]
        trajectory_msg.joint_names = [
            "cart_x",
            "cart_y",
            "cart_z",
            "ang_u",
            "ang_v",
            "ang_w",
        ]
        rospy.loginfo("Following: ExampleTrajectory")
        client = actionlib.SimpleActionClient(
            "/joint_trajectory_controller/follow_joint_trajectory",
            control_msgs.msg.FollowJointTrajectoryAction,
        )
        client.wait_for_server(timeout=rospy.Duration(5.0))
        action_goal = control_msgs.msg.FollowJointTrajectoryActionGoal()
        action_goal.goal.trajectory = trajectory_msg
        client.send_goal(action_goal.goal)
        rospy.loginfo("Following: Trajectory has been sent.")
        result = client.wait_for_result()
        rospy.loginfo("Following: Result received.")
        # Make sure hexapod has time to accomplish the trajectory
        time.sleep(2.0)
        self._widget.pushButton_test_motion.setText("Test-Motion (Trajectory)")
        self._widget.pushButton_test_motion.setEnabled(True)
        self._widget.pushButton_return_home.setEnabled(True)
        self._widget.comboBox_controller.setEnabled(True)
        self._widget.pushButton_test_motion.setStyleSheet("")
        self._update_position_to_real_position()

    def test_motion_clicked(self):
        self._widget.comboBox_controller.setEnabled(False)
        if self._currentController == "joint_trajectory_controller":
            if (
                self._trajectory_thread is None
                or self._trajectory_thread.ident is not None
            ):
                # create new thread object if required
                self._trajectory_thread = threading.Thread(
                    target=self._execute_trajectory_test
                )
            elif self._trajectory_thread.isAlive():
                # Only allow one test-motion at a time
                return
            self._widget.pushButton_test_motion.setText("Running: ExampleTrajectory")
            self._widget.pushButton_test_motion.setStyleSheet(
                self._config_highlight_style_sheet
            )
            self._widget.pushButton_return_home.setEnabled(False)
            self._widget.pushButton_test_motion.setEnabled(False)
            self._trajectory_thread.start()
        elif self._currentController == "joint_group_pos_controller":
            # Choose random axis
            idx_var = randint(0, 5)
            if self._sine_thread is None or self._sine_thread.ident is not None:
                # create new thread object if required
                self._sine_thread = threading.Thread(
                    target=self._execute_sine_test, args=(idx_var,)
                )
            elif self._sine_thread.isAlive():
                # Only allow one test-motion at a time
                return
            self._widget.pushButton_test_motion.setText(
                "Running: Sine-Motion (Joint: {0})".format(idx_var)
            )
            self._widget.pushButton_test_motion.setStyleSheet(
                self._config_highlight_style_sheet
            )
            self._widget.pushButton_test_motion.setEnabled(False)
            self._widget.pushButton_return_home.setEnabled(False)
            self._sine_thread.start()
