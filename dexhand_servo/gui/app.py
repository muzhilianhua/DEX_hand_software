"""QApplication setup: theme, E-stop shortcuts, App Nap, signal/exit handling."""

from __future__ import annotations

import atexit
import ctypes
import ctypes.util
import signal
import sys
from typing import Any

from PySide6.QtCore import QEvent, QObject, Qt, QTimer
from PySide6.QtWidgets import QAbstractSpinBox, QApplication, QLineEdit, QPlainTextEdit

from ..control.worker import Controller
from .main_window import MainWindow
from .theme import apply_dark_theme


class EstopShortcuts(QObject):
    """Application-wide filter: Esc always triggers the E-stop, Space unless a text field has focus.

    Installed on the QApplication so a focused slider, button or dialog cannot swallow the key.
    """

    def __init__(self, controller: Controller) -> None:
        super().__init__()
        self.controller = controller

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        if event.type() != QEvent.Type.KeyPress:
            return False
        key = event.key()  # type: ignore[attr-defined]
        if key == Qt.Key.Key_Escape or (key == Qt.Key.Key_Space and not _typing_text()):
            self.controller.estop("keyboard")
            return True
        return False


def _typing_text() -> bool:
    focus = QApplication.focusWidget()
    return isinstance(focus, QLineEdit | QAbstractSpinBox | QPlainTextEdit) and not getattr(
        focus, "isReadOnly", lambda: False
    )()


def install_estop_shortcuts(app: QApplication, controller: Controller) -> EstopShortcuts:
    shortcuts = EstopShortcuts(controller)
    app.installEventFilter(shortcuts)
    return shortcuts


def keep_awake_macos() -> Any:
    """Opt out of App Nap so the control loop keeps its rate when the window is hidden.

    Calls -[NSProcessInfo beginActivityWithOptions:reason:] through the Objective-C
    runtime; returns the activity token (keep it alive) or None if unavailable.
    """
    if sys.platform != "darwin":
        return None
    try:
        objc = ctypes.cdll.LoadLibrary(ctypes.util.find_library("objc"))
        ctypes.cdll.LoadLibrary(ctypes.util.find_library("Foundation"))
        objc.objc_getClass.restype = ctypes.c_void_p
        objc.objc_getClass.argtypes = [ctypes.c_char_p]
        objc.sel_registerName.restype = ctypes.c_void_p
        objc.sel_registerName.argtypes = [ctypes.c_char_p]
        send = objc.objc_msgSend
        send.restype = ctypes.c_void_p

        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        info = send(objc.objc_getClass(b"NSProcessInfo"), objc.sel_registerName(b"processInfo"))
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p]
        reason = send(objc.objc_getClass(b"NSString"), objc.sel_registerName(b"stringWithUTF8String:"),
                      b"Dexhand servo control loop")
        user_initiated, latency_critical = 0x00FFFFFF, 0xFF00000000
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint64, ctypes.c_void_p]
        token = send(info, objc.sel_registerName(b"beginActivityWithOptions:reason:"),
                     user_initiated | latency_critical, reason)
        send.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        send(token, objc.sel_registerName(b"retain"))
        return token
    except Exception:  # noqa: BLE001 - best effort; the GUI works without it
        return None


def run_gui(controller: Controller, title: str) -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    assert isinstance(app, QApplication)
    apply_dark_theme(app)
    activity = keep_awake_macos()
    window = MainWindow(controller, title=title)
    shortcuts = install_estop_shortcuts(app, controller)

    # Ctrl-C / SIGTERM in the terminal: close the window, which stops the controller (torque off).
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: window.close())
    pump = QTimer()  # lets the Python interpreter run signal handlers while Qt's loop is busy
    pump.start(200)
    pump.timeout.connect(lambda: None)
    atexit.register(controller.stop)

    controller.start()
    window.show()
    code = app.exec()
    controller.stop()
    del shortcuts, activity
    return code
