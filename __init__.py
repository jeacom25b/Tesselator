'''
Copyright (C) 2018 Jean Da Costa machado.
Jean3dimensional@gmail.com

Created by Jean Da Costa machado

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

bl_info = {
    "name": "Tesselator",
    "description": "retopology tools",
    "author": "Jean Da Costa Machado",
    "version": (1, 0, 4),
    "blender": (2, 79, 0),
    "wiki_url": "",
    "category": "Sculpt",
    "location": "3D View > Tool shelf > Sculptkt \  Alt + W"}

import bpy

# load and reload submodules
##################################

modules = [
    "surface_particles",
    "grease_draw",
    "vector_fields",
]

import importlib

imported_modules = []

for module in modules:
    if module in locals():
        if hasattr(module, "unregister"):
            module.unregister()
        importlib.reload(locals()[module])
    else:
        exec("from . import %s" % module)
        imported_modules.append(locals()[module])

def register():
    bpy.utils.register_module(__name__)
    for module in imported_modules:
        if hasattr(module, "register"):
            module.register()

def unregister():
    bpy.utils.unregister_module(__name__)
    for module in imported_modules:
        if hasattr(module, "unregister"):
            module.unregister()
