import bpy
import bmesh
from mathutils import Vector, bvhtree, Matrix, geometry
from math import sqrt, cos


class HitInfo:
    def __init__(self, location, normal, face_index, distance, field):
        self.co = location
        face = field.bm.faces[face_index]
        verts = [vert.co for vert in face.verts]
        normals = [vert.normal for vert in field.bm.faces[face_index].verts]
        curvature = [Vector((0, 0, field.mesh_curvature[vert.index])) for vert in field.bm.faces[face_index].verts]

        self.normal = geometry.barycentric_transform(location, *verts, *normals).normalized()
        self.curvature = geometry.barycentric_transform(location, *verts, *curvature).z

        co = face.verts[0]
        vec = field.vert_field[co.index].u
        vecs = [field.vert_field[vert.index].get_nearest_vec(vec) for vert in face.verts]
        vec = geometry.barycentric_transform(location, *verts, *vecs)
        self.frame = CrossFrame(vec, self.normal)

        self.face = face_index


class CrossFrame:
    def __init__(self, u, normal, strength=1.):
        if u.dot(normal) == 1 or u.length_squared == 0:
            u = normal.orthogonal()
        self.strength = strength
        self.normal = normal
        self.v = (u / u.length).cross(normal)
        self.u = self.v.cross(normal)

    def get_nearest_vec(self, vec):
        u_d = vec.dot(self.u)
        v_d = vec.dot(self.v)
        if u_d * u_d > v_d * v_d:
            return self.u if u_d > 0 else -self.u
        else:
            return self.v if v_d > 0 else -self.v

    def calc_alignment(self, vec):
        vec = vec.normalized()
        dv = self.v.dot(vec)
        du = self.u.dot(vec)
        return max(dv * dv, du * du)

    def select_4_edges(self, vert):
        edge_vecs = [(edge, edge.other_vert(vert).co - vert.co) for edge in vert.link_edges]
        u_edges = []
        v_edges = []
        edge_vecs = sorted(edge_vecs,
                           key=lambda v: self.u.dot(v[1].normalized()))
        u_edges.append(edge_vecs.pop(-1))
        u_edges.append(edge_vecs.pop(0))
        edge_vecs = sorted(edge_vecs,
                           key=lambda v: self.v.dot(v[1].normalized()), )
        v_edges.append(edge_vecs.pop(-1))
        v_edges.append(edge_vecs.pop(0))
        return u_edges, v_edges

    def get_matrix(self):
        return Matrix((self.u, self.v, self.normal)).transposed().to_4x4()


class FrameField:

    def __init__(self, obj):
        self.bm = bmesh.new()
        self.bm.from_mesh(obj.data)
        bmesh.ops.triangulate(self.bm, faces=self.bm.faces)
        self.bm.verts.ensure_lookup_table()
        self.bm.edges.ensure_lookup_table()
        self.bm.faces.ensure_lookup_table()
        self.tree = bvhtree.BVHTree.FromBMesh(self.bm)
        self.vert_field = {}
        self.face_field = {}
        self.mesh_curvature = {}

    def build_major_curvatures(self):
        for vert in self.bm.verts:
            best_normal = Vector()
            best_value = -10
            sum_curvature = 0
            for edge in vert.link_edges:
                other_vert = edge.other_vert(vert)
                value = other_vert.normal.angle(vert.normal)
                if value > best_value:
                    best_normal = other_vert.normal
                    best_value = value
                edge_vec = other_vert.co - vert.co
                le = edge_vec.length_squared
                if le > 0:
                    sum_curvature += edge_vec.dot(other_vert.normal - vert.normal) / le
            sum_curvature /= len(vert.link_edges)
            self.vert_field[vert.index] = CrossFrame(best_normal.cross(vert.normal), vert.normal, best_value)
            self.mesh_curvature[vert.index] = max(sum_curvature, -sum_curvature)
        self.ready = True

    def from_grease_pencil(self, gp_layer, mat, x_mirror=False):
        frame = gp_layer.active_frame
        for stroke in frame.strokes:
            for i in range(len(stroke.points) - 1):
                p0 = mat * stroke.points[i].co
                p1 = mat * stroke.points[i + 1].co
                p_avg = (p0 + p1) / 2
                d = (p0 - p1).normalized()
                location, normal, face_index, distance = self.tree.find_nearest(p_avg)
                for vert in self.bm.faces[face_index].verts:
                    i = vert.index
                    self.vert_field[i] = CrossFrame(d, vert.normal)

                if x_mirror:
                    p_avg.x = -p_avg.x
                    d.x = -d.x
                    location, normal, face_index, distance = self.tree.find_nearest(p_avg)
                    for vert in self.bm.faces[face_index].verts:
                        i = vert.index
                        self.vert_field[i] = CrossFrame(d, vert.normal)

    def erase_part(self, factor=3):
        target_size = 1 + len(self.vert_field) / factor
        for item in (sorted(self.vert_field.items(), key=lambda i: i[1].strength)):
            del self.vert_field[item[0]]
            if len(self.vert_field) <= target_size:
                break

    def marching_growth(self):
        seen_verts = set(self.bm.verts[i] for i in self.vert_field.keys())
        current_front = set()
        for vert in seen_verts:
            for edge in vert.link_edges:
                other_vert = edge.other_vert(vert)
                if other_vert not in seen_verts:
                    current_front.add(other_vert)

        while True:
            new_front = set()
            for vert in current_front:
                u = Vector()
                connected_frames = 0
                for edge in vert.link_edges:
                    other_vert = edge.other_vert(vert)
                    if other_vert in seen_verts:
                        if not connected_frames:
                            u += self.vert_field[other_vert.index].u
                            connected_frames += 1
                        else:
                            u += self.vert_field[other_vert.index].get_nearest_vec(u)
                            connected_frames += 1
                    else:
                        new_front.add(other_vert)
                if connected_frames:
                    self.vert_field[vert.index] = CrossFrame(u / connected_frames, vert.normal)
                    seen_verts.add(vert)
            current_front = new_front
            if len(new_front) == 0:
                break

    def smooth(self, iterations=2):
        for _ in range(iterations):
            new_vert_field = {}
            new_curvature = {}
            for index in self.vert_field.keys():
                vert = self.bm.verts[index]
                count = 1
                u = Vector()
                c = 0
                for edge in vert.link_edges:
                    i = edge.other_vert(vert).index
                    if i in self.vert_field:
                        u += self.vert_field[i].get_nearest_vec(u)
                        c += self.mesh_curvature[i]
                        count += 1
                u /= count
                c /= count
                new_vert_field[index] = CrossFrame(u, vert.normal)
                new_curvature[index] = c
            self.vert_field = new_vert_field

    def mirror_field(self):
        new_field = {}
        for vert in self.bm.verts:
            if vert.co.x < 0:
                new_field[vert.index] = self.vert_field[vert.index]
            else:
                vec = self.sample_point(vert.co).frame.u
                new_field[vert.index] = CrossFrame(vec, vert.normal)
        self.vert_field = new_field

    def sample_point(self, point):
        hit = self.tree.find_nearest(point)
        if None not in hit:
            return HitInfo(*hit, self)
        else:
            return None

    def preview_field(self, out):
        bm = bmesh.new()
        for index in self.vert_field.keys():
            vert = self.bm.verts[index]
            v0 = bm.verts.new(vert.co)
            for v in self.vert_field[vert.index].u, self.vert_field[vert.index].v:
                c = 0.5 / self.mesh_curvature[index]
                n_vert = bm.verts.new(v * c + vert.co)
                bm.edges.new((v0, n_vert))
                n_vert = bm.verts.new(-v * c + vert.co)
                bm.edges.new((v0, n_vert))
        bm.to_mesh(out.data)


class Remesher:
    def __init__(self, obj):
        self.obj = obj
        self.field = FrameField(obj)
        self.bm = bmesh.new()
        self.bm.from_mesh(obj.data)
        bmesh.ops.triangulate(self.bm, faces=self.bm.faces)
        self.frames_samples = {}

    def back_to_mesh(self):
        self.bm.to_mesh(self.obj.data)

    def topology_update(self, min_length, max_length, adaptive):
        collapse_edges = []
        split_edges = []
        seen_verts = set()
        for edge in self.bm.edges:
            v0 = edge.verts[0]
            v1 = edge.verts[1]
            curvature = self.frames_samples[v0].curvature + self.frames_samples[v1].curvature
            curvature /= 2
            curvature *= adaptive
            curvature += 1 - adaptive

            avg_neighbor_le = 0
            count = 0
            for n_edge in [n_edge for n_edge in v0.link_edges] + [n_edge for n_edge in v1.link_edges]:
                if n_edge is not edge:
                    avg_neighbor_le += n_edge.calc_length()
                    count += 1
            if count == 0:
                continue
            avg_neighbor_le /= count

            le = edge.calc_length()

            if le < min_length / curvature or le < avg_neighbor_le * 0.5:
                if edge.verts[0] not in seen_verts and edge.verts[1] not in seen_verts:
                    collapse_edges.append(edge)
                    seen_verts.add(edge.verts[0])
                    seen_verts.add(edge.verts[1])

            elif le > max_length / curvature:
                split_edges.append(edge)

        bmesh.ops.subdivide_edges(self.bm, edges=split_edges, cuts=1)
        bmesh.ops.collapse(self.bm, edges=collapse_edges)

        bmesh.ops.triangulate(self.bm, faces=self.bm.faces)

        dissolve_verts = []
        for vert in self.bm.verts:
            if len(vert.link_edges) < 5:
                dissolve_verts.append(vert)
        bmesh.ops.dissolve_verts(self.bm, verts=dissolve_verts)
        bmesh.ops.triangulate(self.bm, faces=self.bm.faces)

    def sample_field(self, snap=True):
        new_frame_samples = {}
        if snap:
            for vert in self.bm.verts:
                hit = self.field.sample_point(vert.co)
                new_frame_samples[vert] = hit
                if hit:
                    vert.co = hit.co
        else:
            for vert in self.bm.verts:
                hit = self.field.sample_point(vert.co)
                if hit:
                    new_frame_samples[vert] = hit
        self.frames_samples = new_frame_samples

    def align_verts(self, factor=0.5):
        for vert, hit in self.frames_samples.items():
            if len(vert.link_edges) < 4 or not hit:
                continue
            mat = hit.frame.get_matrix()
            frame = hit.frame
            inv = mat.inverted()
            u_edges, v_edges = hit.frame.select_4_edges(vert)

            x_p = inv * u_edges[0][1]
            x_n = inv * u_edges[1][1]
            y_p = inv * v_edges[0][1]
            y_n = inv * v_edges[1][1]

            vec = Vector()

            vec.x = sorted([y_p.x, y_n.x], key=lambda x: x * x)[0]
            vec.y = sorted([x_p.y, x_n.y], key=lambda y: y * y)[0]

            vec.x += (y_p + y_n / 2).x * 0.5
            vec.y += (x_p + x_n / 2).y * 0.5
            vec = mat * vec
            vert.co += vec * factor

    def relax_verts(self, factor=0.5, ):
        for vert, hit in self.frames_samples.items():
            if len(vert.link_edges) < 4 or not hit:
                continue
            mat = hit.frame.get_matrix()
            inv = mat.inverted()
            u_edges, v_edges = hit.frame.select_4_edges(vert)

            x_p = inv * u_edges[0][1]
            x_n = inv * u_edges[1][1]
            y_p = inv * v_edges[0][1]
            y_n = inv * v_edges[1][1]

            vec = x_p + x_n + y_p + y_n
            vec.z = 0
            vert.co += mat * vec * factor

    def to_quads(self):
        edge_scores = dict([(edge, 0) for edge in self.bm.edges])
        for vert in self.bm.verts:
            if len(vert.link_edges) < 4:
                for edge in vert.link_edges:
                    edge_scores[edge] += 1
                continue
            frame = self.frames_samples[vert].frame
            u_edges, v_edges = frame.select_4_edges(vert)
            for edge, vec in u_edges + v_edges:
                edge_scores[edge] += 1

        delete_edges = [edge for (edge, score) in edge_scores.items() if score == 0]
        bmesh.ops.dissolve_edges(self.bm, edges=delete_edges)

    def symmetrize(self, axis=0):
        items = list(self.bm.verts) + list(self.bm.edges) + list(self.bm.faces)
        bmesh.ops.symmetrize(self.bm, input=items, direction=axis)


def power_decay(i, n, p=1):
    v = (1 - i / n) ** p
    if v > 1:
        v = 1
    elif v < 0:
        v = 0
    return v


def exp_decay(i):
    return 1 / (1 + i)


def snap_verts(source_verts, target_ob):
    for vert in source_verts:
        hit1, loc1, normal1, index1 = target_ob.ray_cast(vert.co, vert.normal)
        hit2, loc2, normal2, index2 = target_ob.ray_cast(vert.co, -vert.normal)
        dist1 = (loc1 - vert.co).length_squared
        dist2 = (loc2 - vert.co).length_squared
        if hit1 and hit2:
            if dist1 < dist2:
                vert.co = loc1
            else:
                vert.co = loc2

        elif not hit2 and hit1:
            vert.co = loc1

        elif not hit1 and hit2:
            vert.co = loc2


class Test(bpy.types.Operator):
    bl_idname = "tesselator.tesselate"
    bl_label = "Tesselate"
    bl_description = ""
    bl_options = {"REGISTER", "UNDO"}
    _source_ob = None
    _timer = None
    _remesher = None
    _algorithm_stepper = None
    _min_scale = None
    _max_scale = None
    _min_curv = None
    _max_curve = None

    resolution = bpy.props.FloatProperty(
        name="Resolution",
        min=1,
        default=20.0
    )

    decimation = bpy.props.FloatProperty(
        name="Pre Decimation",
        min=0.00001,
        max=1,
        default=0.2
    )

    relax_steps = bpy.props.IntProperty(
        name="Relax Iterations",
        min=1,
        default=10
    )

    align_steps = bpy.props.IntProperty(
        name="Align Iterations",
        min=1,
        default=5
    )

    subdivisions = bpy.props.IntProperty(
        name="Subdivisions",
        min=0,
        max=6,
        default=1
    )

    adaptive = bpy.props.FloatProperty(
        name="Detail",
        min=0,
        max=0.999,
        default=0.3
    )

    bias = bpy.props.FloatProperty(
        name="Bias",
        min=0.001,
        max=0.999,
        default=0.5
    )

    use_gp = bpy.props.BoolProperty(
        name="Grease Pencil Guides",
        default=True,
    )

    x_mirror = bpy.props.BoolProperty(
        name="X Mirror",
        default=True,
    )

    feature_threshold = bpy.props.FloatProperty(
        name="Feature Threshold",
        min=0.0001,
        max=1,
        default=0.5,
    )

    field_smoothing = bpy.props.IntProperty(
        name="Field Smoothing",
        min=0,
        default=10
    )

    @classmethod
    def poll(cls, context):
        if len(context.selected_objects) > 1:
            return
        if context.active_object:
            if context.active_object.type == "MESH":
                return True

    def draw(self, context):
        layout = self.layout
        box = layout.box()
        box.label("Density")
        box.prop(self, "resolution")
        box.prop(self, "subdivisions")
        box.prop(self, "adaptive", slider=True)

        box = layout.box()
        box.label("Topology Shuffle")
        box.prop(self, "align_steps")
        box.prop(self, "relax_steps")
        box.prop(self, "bias", slider=True)

        box = layout.box()
        box.label("Guiding Frames")
        box.prop(self, "use_gp")
        box.prop(self, "x_mirror")
        box.prop(self, "decimation", slider=True)
        box.prop(self, "feature_threshold", slider=True)
        box.prop(self, "field_smoothing")

    def algorithm_stepper(self, context):
        for x in range(self.align_steps):
            self._remesher.align_verts(factor=exp_decay(x) / 2)
            self._remesher.topology_update(self._min_scale, self._max_scale, self.adaptive)
            self._remesher.sample_field()
            self._remesher.back_to_mesh()
            yield {"RUNNING_MODAL"}

        if self.x_mirror:
            self._remesher.symmetrize(0)
            self._remesher.sample_field()
            self._remesher.topology_update(self._min_scale, self._max_scale, self.adaptive)
            self._remesher.sample_field()

        for x in range(self.relax_steps):
            self._remesher.relax_verts(factor=exp_decay(x) / 2)
            self._remesher.topology_update(self._min_scale, self._max_scale, self.adaptive)
            self._remesher.sample_field()
            self._remesher.back_to_mesh()
            yield {"RUNNING_MODAL"}

        yield self.finish(context)

    def invoke(self, context, event):
        wm = context.window_manager
        return wm.invoke_props_dialog(self)

    def execute(self, context):
        self._source_ob = context.active_object
        bpy.ops.object.duplicate()
        bpy.ops.object.convert(target="MESH")
        context.active_object.show_wire = True
        context.active_object.show_all_edges = True
        self._obj = context.active_object
        if self.decimation < 1:
            md = context.active_object.modifiers.new(type="DECIMATE", name="Decimate")
            md.ratio = self.decimation
        bpy.ops.object.convert(target="MESH")

        self._remesher = Remesher(context.active_object)
        rem = self._remesher

        rem.field.build_major_curvatures()
        rem.field.erase_part(1 / self.feature_threshold)

        if self.use_gp:
            gp = context.scene.grease_pencil
            if gp:
                if gp.layers:
                    layer = gp.layers.active
                    rem.field.from_grease_pencil(layer, context.active_object.matrix_world.inverted(), self.x_mirror)

        rem.field.marching_growth()
        rem.field.smooth(self.field_smoothing)
        if self.x_mirror:
            rem.field.mirror_field()

        rem.sample_field()

        r = self.resolution
        b = 1 - self.bias
        self._min_scale = max(context.active_object.dimensions) / max(context.active_object.scale) / r * b
        self._max_scale = max(context.active_object.dimensions) / max(context.active_object.scale) / r / b

        context.window_manager.modal_handler_add(self)
        self._timer = context.window_manager.event_timer_add(0.01, context.window)
        self._algorithm_stepper = self.algorithm_stepper(context)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type == "TIMER":
            context.area.tag_redraw()
            return next(self._algorithm_stepper)

        elif event.type in {"ESC", "RET"}:
            return self.finish(context)

        return {"RUNNING_MODAL"}

    def finish(self, context):
        self._remesher.back_to_mesh()
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.tris_convert_to_quads(face_threshold=3.14, shape_threshold=3.14)
        bpy.ops.object.mode_set(mode="OBJECT")
        context.active_object.show_wire = False

        bm = bmesh.new()
        bm.from_mesh(context.active_object.data)
        merge_verts = {}
        seen_verts = set()
        for face in bm.faces:
            merge_face_verts = []

            if len(face.verts) == 4:
                for vert in face.verts:
                    if len(vert.link_edges) == 3:
                        merge_face_verts.append(vert)
                if len(merge_face_verts) == 2:
                    v0 = merge_face_verts[0]
                    v1 = merge_face_verts[1]
                    if v0 not in seen_verts and v1 not in seen_verts:
                        v0.co = face.calc_center_median()
                        v1.co = v0.co
                        merge_verts[v0] = v1
                        seen_verts.add(v0)
                        seen_verts.add(v1)
        bmesh.ops.weld_verts(bm, targetmap=merge_verts)

        dissolve_edges = []
        seen_verts = set()
        for edge in bm.edges:
            triangles = 0
            for face in edge.link_faces:
                if len(face.verts) == 3:
                    triangles += 1
            if triangles == 2:
                v0 = edge.verts[0]
                v1 = edge.verts[1]
                if v0 not in seen_verts and v1 not in seen_verts:
                    dissolve_edges.append(edge)
                    seen_verts.add(v0)
                    seen_verts.add(v1)
        bmesh.ops.dissolve_edges(bm, edges=dissolve_edges)

        bm.to_mesh(context.active_object.data)

        md = context.active_object.modifiers.new(type="SUBSURF", name="SubSurf")
        md.levels=1
        bpy.ops.object.modifier_apply(modifier=md.name)

        md = context.active_object.modifiers.new(type="SHRINKWRAP", name="Snap")
        md.target=self._source_ob
        md.wrap_method = "PROJECT"
        md.use_negative_direction = True
        md.use_positive_direction = True
        bpy.ops.object.modifier_apply(modifier=md.name)

        if self.subdivisions > 0:
            md_mr = context.active_object.modifiers.new(type="MULTIRES", name="MultiRes")
            for i in range(self.subdivisions):
                bpy.ops.object.multires_subdivide(modifier=md_mr.name)
                md = context.active_object.modifiers.new(type="SHRINKWRAP", name="Snap")
                md.target = self._source_ob
                md.wrap_method = "PROJECT"
                md.use_negative_direction = True
                md.use_positive_direction = True
                bpy.ops.object.modifier_apply(modifier=md.name)

        # for vert in context.active_object.data.vertices:
        #     hit = self._remesher.field.ray_cast_sample(vert.co, vert.normal)
        #     if hit:
        #         vert.co = hit.co
        snap_verts(context.active_object.data.vertices, self._source_ob)
        context.window_manager.event_timer_remove(self._timer)
        return {"FINISHED"}
