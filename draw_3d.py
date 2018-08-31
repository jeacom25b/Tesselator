import bgl
import bpy

_callbacks = []
_handles = []


def get_callback():
    if _callbacks:
        return _callbacks[0]
    else:
        _callbacks.append(DrawCallback())
        return _callbacks[0]


def register():
    callback = get_callback()
    _handles.append(bpy.types.SpaceView3D.draw_handler_add(callback, (), "WINDOW", "POST_VIEW"))
    print("Registered Draw 3D module:", _handles)


def unregister():
    print("Unregistered Draw 3D module", _handles)
    bpy.types.SpaceView3D.draw_handler_remove(_handles[0], "WINDOW")
    _callbacks.clear()
    _handles.clear()


class DrawObject:
    def __init__(self):
        self.commands = []
        get_callback().add_object(self)

    def add_line(self, start, end, width=1.5, color=(1, 0, 0, 1)):
        self.commands.append((start, end, width, color))

    def start_drawing(self):
        bgl.glEnable(bgl.GL_BLEND)
        bgl.glBegin(bgl.GL_LINES)

    def stop_drawing(self):
        bgl.glDisable(bgl.GL_BLEND)
        bgl.glEnd()

    def line3d(self, start, end, width=1.5, color=(1, 0, 0, 1)):
        bgl.glColor4f(*color)
        bgl.glLineWidth(width)
        bgl.glVertex3f(*start)
        bgl.glVertex3f(*end)

    def draw(self):
        self.start_drawing()
        for c in self.commands:
            self.line3d(*c)
        self.stop_drawing()

    def remove(self):
        get_callback().remove_object(self)


class DrawCallback:

    def __call__(self, *args):
        if not self.enable:
            return
        for obj in self.objects:
            print(obj)
            obj.draw()

    def __init__(self):
        self.enable = True
        self.objects = []

    def add_object(self, obj):
        self.objects.append(obj)

    def remove_object(self, obj):
        self.objects.remove(obj)
