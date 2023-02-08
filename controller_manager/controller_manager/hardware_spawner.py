#!/usr/bin/env python3
# Copyright 2023 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import sys
import time

from controller_manager import list_hardware_components, set_hardware_component_state

from lifecycle_msgs.msg import State
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# from https://stackoverflow.com/a/287944


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def first_match(iterable, predicate):
    return next((n for n in iterable if predicate(n)), None)


def wait_for_value_or(function, node, timeout, default, description):
    while node.get_clock().now() < timeout:
        if result := function():
            return result
        node.get_logger().info(
            f"Waiting for {description}", throttle_duration_sec=2, skip_first=True
        )
        time.sleep(0.2)
    return default


def combine_name_and_namespace(name_and_namespace):
    node_name, namespace = name_and_namespace
    return namespace + ("" if namespace.endswith("/") else "/") + node_name


def find_node_and_namespace(node, full_node_name):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return first_match(
        node_names_and_namespaces,
        lambda n: combine_name_and_namespace(n) == full_node_name,
    )


def has_service_names(node, node_name, node_namespace, service_names):
    client_names_and_types = node.get_service_names_and_types_by_node(
        node_name, node_namespace
    )
    if not client_names_and_types:
        return False
    client_names, _ = zip(*client_names_and_types)
    return all(service in client_names for service in service_names)


def wait_for_controller_manager(node, controller_manager, timeout_duration):
    # List of service names from controller_manager we wait for
    service_names = (
        f"{controller_manager}/configure_controller",
        f"{controller_manager}/list_controllers",
        f"{controller_manager}/list_controller_types",
        f"{controller_manager}/list_hardware_components",
        f"{controller_manager}/list_hardware_interfaces",
        f"{controller_manager}/load_controller",
        f"{controller_manager}/reload_controller_libraries",
        f"{controller_manager}/set_hardware_component_state",
        f"{controller_manager}/switch_controller",
        f"{controller_manager}/unload_controller",
    )

    # Wait for controller_manager
    timeout = node.get_clock().now() + Duration(seconds=timeout_duration)
    node_and_namespace = wait_for_value_or(
        lambda: find_node_and_namespace(node, controller_manager),
        node,
        timeout,
        None,
        f"'{controller_manager}' node to exist",
    )

    # Wait for the services if the node was found
    if node_and_namespace:
        node_name, namespace = node_and_namespace
        return wait_for_value_or(
            lambda: has_service_names(node, node_name, namespace, service_names),
            node,
            timeout,
            False,
            f"'{controller_manager}' services to be available",
        )

    return False


def handle_set_component_state_service_call(
    node, controller_manager_name, component, target_state, action
):
    response = set_hardware_component_state(
        node, controller_manager_name, component, target_state
    )
    if response.ok and response.state == target_state:
        node.get_logger().info(
            bcolors.OKGREEN
            + f"{action} component '{component}'. Hardware now in state: {response.state}."
        )
    elif response.ok and not response.state == target_state:
        node.get_logger().warn(
            bcolors.WARNING
            + f"Could not {action} component '{component}'. Service call returned ok=True, but state: {response.state} is not equal to target state '{target_state}'."
        )
    else:
        node.get_logger().warn(
            bcolors.WARNING
            + f"Could not {action} component '{component}'. Service call failed."
        )


def activate_components(node, controller_manager_name, components_to_activate):
    active_state = State()
    active_state.id = State.PRIMARY_STATE_ACTIVE
    active_state.label = "active"
    for component in components_to_activate:
        handle_set_component_state_service_call(
            node, controller_manager_name, component, active_state, "activated"
        )


def configure_components(node, controller_manager_name, components_to_configure):
    inactive_state = State()
    inactive_state.id = State.PRIMARY_STATE_INACTIVE
    inactive_state.label = "inactive"
    for component in components_to_configure:
        handle_set_component_state_service_call(
            node, controller_manager_name, component, inactive_state, "configured"
        )


def main(args=None):

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    parser = argparse.ArgumentParser()
    activate_group = parser.add_argument_group("activate")
    configure_group = parser.add_argument_group("configure")

    parser.add_argument(
        "-c",
        "--controller-manager",
        help="Name of the controller manager ROS node",
        default="controller_manager",
        required=False,
    )
    parser.add_argument(
        "--controller-manager-timeout",
        help="Time to wait for the controller manager",
        required=False,
        default=10,
        type=int,
    )

    activate_group.add_argument(
        "--activate",
        help="Activates the given components. Note: Components are by default configured before activated. ",
        nargs="+",
        default=[],
        required=False,
    )
    activate_group.add_argument(
        "--activate-all",
        help="Activates all present hardware components. If this flag is present the components presented by '--activate' are ignored.",
        required=False,
        default=False,
        action="store_true",
    )

    configure_group.add_argument(
        "--configure",
        help="Configures the given components.",
        nargs="+",
        default=[],
        required=False,
    )
    configure_group.add_argument(
        "--configure-all",
        help="Configures all present hardware components. If this flag is present the components presented by '--configure' are ignored.",
        required=False,
        default=False,
        action="store_true",
    )

    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(command_line_args)
    controller_manager_name = args.controller_manager
    controller_manager_timeout = args.controller_manager_timeout
    components_to_activate = args.activate
    activate_all = args.activate_all
    components_to_configure = args.configure
    configure_all = args.configure_all

    node = Node("hardware_spawner")
    if not controller_manager_name.startswith("/"):
        spawner_namespace = node.get_namespace()
        if spawner_namespace != "/":
            controller_manager_name = f"{spawner_namespace}/{controller_manager_name}"
        else:
            controller_manager_name = f"/{controller_manager_name}"
    try:
        if not wait_for_controller_manager(
            node, controller_manager_name, controller_manager_timeout
        ):
            node.get_logger().error("Controller manager not available")
            return 1

        if (
            not activate_all
            and not configure_all
            and components_to_activate == []
            and components_to_configure == []
        ):
            node.get_logger().warn(
                "You did not specify any components to configure or activate."
            )
            parser.print_help()
            return 0

        if activate_all:
            response = list_hardware_components(node, controller_manager_name)
            components_to_activate = list(map((lambda x: x.name), response.component))

        if configure_all:
            response = list_hardware_components(node, controller_manager_name)
            components_to_configure = list(map((lambda x: x.name), response.component))

        # get components that should only be configured and only configure them
        # activation does configuration anyways so would be unnecessary to keep
        # components in configure list if they get activate.
        configure_only_components = [
            component
            for component in components_to_configure
            if component not in components_to_activate
        ]
        configure_components(node, controller_manager_name, configure_only_components)
        activate_components(node, controller_manager_name, components_to_activate)

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    ret = main()
    sys.exit(ret)