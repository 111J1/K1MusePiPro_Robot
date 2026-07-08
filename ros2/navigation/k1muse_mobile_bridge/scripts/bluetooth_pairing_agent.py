#!/usr/bin/env python3
"""BlueZ pairing agent for phone-only pairing on K1.

The agent registers as NoInputNoOutput so Android can complete classic
Bluetooth pairing from the phone side without a K1 desktop confirmation.
"""

import argparse
import sys
import time

import dbus
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib


BLUEZ = "org.bluez"
AGENT_IFACE = "org.bluez.Agent1"
AGENT_MANAGER_IFACE = "org.bluez.AgentManager1"
ADAPTER_IFACE = "org.bluez.Adapter1"
DEVICE_IFACE = "org.bluez.Device1"
PROPS_IFACE = "org.freedesktop.DBus.Properties"
OBJECT_MANAGER_IFACE = "org.freedesktop.DBus.ObjectManager"
AGENT_PATH = "/com/embodiedai/robotcontroller/PairingAgent"


class PairingAgent(dbus.service.Object):
    def __init__(self, bus):
        super().__init__(bus, AGENT_PATH)
        self.bus = bus

    def _device_props(self, device):
        return dbus.Interface(self.bus.get_object(BLUEZ, device), PROPS_IFACE)

    def _device_address(self, device):
        try:
            return str(self._device_props(device).Get(DEVICE_IFACE, "Address"))
        except Exception:
            return str(device)

    def _trust_device(self, device):
        try:
            self._device_props(device).Set(DEVICE_IFACE, "Trusted", dbus.Boolean(True))
            print(f"Trusted device {self._device_address(device)}", flush=True)
        except Exception as exc:
            print(f"Warning: failed to trust {device}: {exc}", flush=True)

    @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
    def Release(self):
        print("BlueZ released pairing agent", flush=True)

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        self._trust_device(device)
        print(f"Auto pairing legacy PIN for {self._device_address(device)}", flush=True)
        return "0000"

    @dbus.service.method(AGENT_IFACE, in_signature="ou", out_signature="")
    def DisplayPinCode(self, device, pincode):
        print(f"DisplayPinCode {self._device_address(device)} {pincode}", flush=True)

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        self._trust_device(device)
        print(f"Auto pairing passkey for {self._device_address(device)}", flush=True)
        return dbus.UInt32(0)

    @dbus.service.method(AGENT_IFACE, in_signature="ouq", out_signature="")
    def DisplayPasskey(self, device, passkey, entered):
        print(f"DisplayPasskey {self._device_address(device)} {passkey} entered={entered}", flush=True)

    @dbus.service.method(AGENT_IFACE, in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        self._trust_device(device)
        print(f"Auto confirmed pairing for {self._device_address(device)} passkey={passkey}", flush=True)

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        self._trust_device(device)
        print(f"Auto authorized pairing for {self._device_address(device)}", flush=True)

    @dbus.service.method(AGENT_IFACE, in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        self._trust_device(device)
        print(f"Auto authorized service {uuid} for {self._device_address(device)}", flush=True)

    @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
    def Cancel(self):
        print("Pairing request canceled", flush=True)


def find_adapter(bus):
    manager = dbus.Interface(bus.get_object(BLUEZ, "/"), OBJECT_MANAGER_IFACE)
    for path, interfaces in manager.GetManagedObjects().items():
        if ADAPTER_IFACE in interfaces:
            return path
    return None


def configure_adapter(bus, adapter_path, discoverable):
    props = dbus.Interface(bus.get_object(BLUEZ, adapter_path), PROPS_IFACE)
    safe_set_adapter_property(props, "Powered", dbus.Boolean(True))
    safe_set_adapter_property(props, "Pairable", dbus.Boolean(True))
    safe_set_adapter_property(props, "PairableTimeout", dbus.UInt32(0))
    if discoverable:
        safe_set_adapter_property(props, "Discoverable", dbus.Boolean(True))
        safe_set_adapter_property(props, "DiscoverableTimeout", dbus.UInt32(0))


def safe_set_adapter_property(props, name, value):
    for attempt in range(3):
        try:
            props.Set(ADAPTER_IFACE, name, value)
            return
        except Exception as exc:
            if "org.bluez.Error.Busy" not in str(exc) or attempt == 2:
                print(f"Warning: failed to set adapter {name}: {exc}", flush=True)
                return
            time.sleep(0.5)


def trust_known_devices(bus):
    manager = dbus.Interface(bus.get_object(BLUEZ, "/"), OBJECT_MANAGER_IFACE)
    for path, interfaces in manager.GetManagedObjects().items():
        props = interfaces.get(DEVICE_IFACE)
        if not props:
            continue
        if not bool(props.get("Paired", False)):
            continue
        try:
            device_props = dbus.Interface(bus.get_object(BLUEZ, path), PROPS_IFACE)
            device_props.Set(DEVICE_IFACE, "Trusted", dbus.Boolean(True))
        except Exception as exc:
            print(f"Warning: failed to trust paired device {path}: {exc}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--discoverable",
        action="store_true",
        help="Keep the adapter discoverable as well as pairable.",
    )
    parser.add_argument(
        "--capability",
        default="NoInputNoOutput",
        help="BlueZ agent capability. Default: NoInputNoOutput.",
    )
    args = parser.parse_args()

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    adapter_path = None
    for _ in range(30):
        adapter_path = find_adapter(bus)
        if adapter_path:
            break
        time.sleep(1)
    if not adapter_path:
        print("No BlueZ adapter found", file=sys.stderr, flush=True)
        return 1

    configure_adapter(bus, adapter_path, args.discoverable)
    trust_known_devices(bus)

    agent = PairingAgent(bus)
    manager = dbus.Interface(bus.get_object(BLUEZ, "/org/bluez"), AGENT_MANAGER_IFACE)
    manager.RegisterAgent(AGENT_PATH, args.capability)
    manager.RequestDefaultAgent(AGENT_PATH)
    print(
        f"Bluetooth pairing agent ready: adapter={adapter_path} "
        f"capability={args.capability} discoverable={args.discoverable}",
        flush=True,
    )

    def keep_default_agent():
        try:
            configure_adapter(bus, adapter_path, args.discoverable)
            trust_known_devices(bus)
            manager.RequestDefaultAgent(AGENT_PATH)
        except Exception as exc:
            print(f"Warning: failed to refresh pairing agent: {exc}", flush=True)
        return True

    GLib.timeout_add_seconds(5, keep_default_agent)

    try:
        GLib.MainLoop().run()
    finally:
        try:
            manager.UnregisterAgent(AGENT_PATH)
        except Exception:
            pass
        agent.remove_from_connection()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
