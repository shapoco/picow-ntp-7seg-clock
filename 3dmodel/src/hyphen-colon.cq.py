import cadquery as cq
import os

body = (
    cq.Workplane("XY")
    .box(4.5, 15, 3, centered=(True, True, False))
)

hyphen = body.cut(
    cq.Workplane("XY")
    .box(3, 1.5, 10, centered=(True, True, False))
)

colon = (
    body
    .faces(">Z")
    .workplane()
    .pushPoints([
        (0, 1.5 * 2.54),
        (0, -1.5 * 2.54),
    ])
    .hole(2.5, 10)
)

show_object(hyphen.translate((-3, 0, 0)), name="hyphen")
show_object(colon.translate((3, 0, 0)), name="colon")

OUT_DIR = "../step"
os.makedirs(OUT_DIR, exist_ok=True)
hyphen.export(f"{OUT_DIR}/hyphen.step")
hyphen.export(f"{OUT_DIR}/hyphen.stl")
colon.export(f"{OUT_DIR}/colon.step")
colon.export(f"{OUT_DIR}/colon.stl")
