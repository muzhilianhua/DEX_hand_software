"""Dark theme tokens shared by every widget (dataviz reference palette, dark surface)."""

from __future__ import annotations

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

PAGE = "#0d0d0d"
SURFACE = "#1a1a19"
SURFACE_RAISED = "#232322"
TEXT = "#ffffff"
TEXT_SECONDARY = "#c3c2b7"
TEXT_MUTED = "#898781"
GRID = "#2c2c2a"
AXIS = "#383835"
SERIES = "#3987e5"  # target / setpoint marks
ACTUAL = "#ffffff"

# Status colours always ship with an icon + label, never colour alone.
GOOD = "#0ca30c"
WARNING = "#fab219"
SERIOUS = "#ec835a"
CRITICAL = "#d03b3b"

STYLESHEET = f"""
QWidget {{ font-size: 13px; }}
QGroupBox {{
    background: {SURFACE}; border: 1px solid {GRID}; border-radius: 8px;
    margin-top: 14px; padding: 10px 8px 8px 8px;
}}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {TEXT}; font-weight: 600; }}
QLabel[role="muted"] {{ color: {TEXT_MUTED}; }}
QLabel[role="secondary"] {{ color: {TEXT_SECONDARY}; }}
QLabel[role="value"] {{ color: {TEXT}; font-weight: 600; }}
QPushButton {{
    background: {SURFACE_RAISED}; border: 1px solid {AXIS}; border-radius: 6px; padding: 5px 10px;
}}
QPushButton:hover {{ border-color: {TEXT_MUTED}; }}
QPushButton:checked {{ background: #12361a; border-color: {GOOD}; }}
QPushButton:disabled {{ color: {TEXT_MUTED}; }}
QPushButton#estop {{
    background: {CRITICAL}; color: white; font-size: 18px; font-weight: 800;
    border: 2px solid #ff8a8a; border-radius: 10px; padding: 10px 26px;
}}
QPushButton#estop:pressed {{ background: #a02020; }}
QLabel#banner {{ border-radius: 6px; padding: 8px 12px; font-weight: 700; }}
QPlainTextEdit {{ background: {SURFACE}; border: 1px solid {GRID}; border-radius: 6px; }}
QListWidget, QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
    background: {SURFACE_RAISED}; border: 1px solid {AXIS}; border-radius: 4px; padding: 3px;
}}
"""


def apply_dark_theme(app: QApplication) -> None:
    app.setStyle("Fusion")
    palette = QPalette()
    roles = {
        QPalette.ColorRole.Window: PAGE,
        QPalette.ColorRole.WindowText: TEXT,
        QPalette.ColorRole.Base: SURFACE,
        QPalette.ColorRole.AlternateBase: SURFACE_RAISED,
        QPalette.ColorRole.Text: TEXT,
        QPalette.ColorRole.Button: SURFACE_RAISED,
        QPalette.ColorRole.ButtonText: TEXT,
        QPalette.ColorRole.ToolTipBase: SURFACE_RAISED,
        QPalette.ColorRole.ToolTipText: TEXT,
        QPalette.ColorRole.Highlight: SERIES,
        QPalette.ColorRole.HighlightedText: TEXT,
        QPalette.ColorRole.PlaceholderText: TEXT_MUTED,
    }
    for role, color in roles.items():
        palette.setColor(role, QColor(color))
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(TEXT_MUTED))
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, QColor(TEXT_MUTED))
    app.setPalette(palette)
    app.setStyleSheet(STYLESHEET)
