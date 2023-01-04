# SPDX-License-Identifier: GPL-2.0-or-later

# Initialization script for SPA 2D Animation template

import bpy
from bpy.app.handlers import persistent


def apply_gpencil_brush_settings():
    """Load brush with custom settings."""
    pencil = bpy.data.brushes["Pencil"]
    pencil.gpencil_settings.pen_strength = 1
    pencil.gpencil_settings.use_strength_pressure = False
    bpy.data.scenes["Scene"].tool_settings.gpencil_paint.show_brush = False



@persistent
def load_handler(_):
    apply_gpencil_brush_settings()


def register():
    bpy.app.handlers.load_factory_startup_post.append(load_handler)


def unregister():
    bpy.app.handlers.load_factory_startup_post.remove(load_handler)
