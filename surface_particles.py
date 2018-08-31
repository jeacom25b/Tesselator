import bpy
from mathutils import Matrix, Vector
from mathutils.kdtree import KDTree
from . import grease_draw


class ParticleManager:
    _particle_types = {}

    @classmethod
    def register_particle_type(cls, type, name):
        cls._particle_types[name] = type

    @classmethod
    def unregister_particle_type(cls, name):
        del cls._particle_types[name]

    def __init__(self, context):
        self.obj = context.active_object
        self.particles = []
        self.edges = []
        self.draw_layer = grease_draw.StrokeLayer(context)
        self.kd_tree = None

    def create_particle(self, type, location, radius=0.03):
        if type in self._particle_types:
            p = self._particle_types[type](radius, location, self)
            self.particles.append(p)
            return p
        else:
            raise KeyError("No such particle registered: %s" % type)

    def build_kd_tree(self):
        self.kd_tree = KDTree(len(self.particles))
        for index, particle in enumerate(self.particles):
            self.kd_tree.insert(particle.location, index)
        self.kd_tree.balance()

    def nearest_n_particles(self, location, n):
        for location, index, distance in self.kd_tree.find_n(location, n):
            particle = self.particles[index]
            yield particle, distance

    def nearest_n_tag_particles(self, location, n, tag):
        def tag_filter(i):
            return self.particles[i].tag is tag

        for location, index, distance in self.kd_tree.find_n(location, n, filter=tag_filter):
            particle = self.particles[index]
            yield particle, distance

    def step(self, speed):
        for particle in self.particles:
            particle.step(speed)

    def sample_obj(self, location):
        return self.obj.closest_point_on_mesh(location)


class BaseParticle:
    def __init__(self, radius, location, manager):
        self.radius = radius
        self._location = Vector(location)
        self._normal = Vector((0, 0, 1))
        self.manager = manager
        self.stroke_model = None
        self.create_stroke()
        self.index = 0
        self.tag = ""

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, value):
        if type(value) is not Vector:
            self._location = Vector(value)
        else:
            self._location = value

    @property
    def normal(self):
        return self._normal

    @normal.setter
    def normal(self, value):
        if type(value) is not Vector:
            self._normal = Vector(value)
        else:
            self._normal = value

    def step(self, speed):
        pass

    def create_stroke(self):
        stroke = self.manager.draw_layer.create_stroke()
        stroke.set_points([
            (-1, 0, 0),
            (1, 0, 0),
            (0, 0, 0),
            (0, 1, 0),
            (0, -1, 0)]
        )
        self.stroke_model = stroke

    def delete_stroke(self):
        self.manager.draw_layer.delete_stroke(self.stroke_model)

    def update_stroke(self):
        self.stroke_model.location = self.location
        z_component = self.normal
        x_component = z_component.orthogonal()
        y_component = x_component.cross(z_component)
        mat = Matrix([x_component, y_component, z_component]).transposed().to_4x4()
        self.stroke_model.rotation = mat
        self.stroke_model.scale = [self.radius] * 3
        pass


class TestParticle(BaseParticle):
    def step(self, speed):
        from random import random
        hit, location, normal, index = self.manager.sample_obj(
            self.location + Vector((0, 0, -speed)))
        self.location = location
        self.normal = normal
        self.update_stroke()


ParticleManager.register_particle_type(TestParticle, "test_particle")


class TestGrease(bpy.types.Operator):
    bl_idname = "test.grease"
    bl_label = "Test Grease"
    bl_description = ""
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return True

    def invoke(self, context, event):
        self.particles = ParticleManager(context)
        self.particles.create_particle("test_particle", (0, 0, 10), 0.2)

        context.window_manager.modal_handler_add(self)
        self._timer = context.window_manager.event_timer_add(0.01, context.window)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type == "TIMER":
            self.particles.step(0.01)

        if event.type in {"RIGHTMOUSE", "ESC"}:
            context.window_manager.event_timer_remove(self._timer)
            self.particles.draw_layer.remove_layer()
            return {"CANCELLED"}

        return {"PASS_THROUGH"}
