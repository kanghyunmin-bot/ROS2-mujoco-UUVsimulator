"""Export the editable draw.io workflow as a 178 mm vector PDF and a PNG."""

from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

from pypdf import PdfReader, PdfWriter, Transformation


ROOT = Path(__file__).resolve().parent
STEM = "figure-03-calibration-workflow"


def draw():
    with TemporaryDirectory() as temp:
        source = Path(temp) / "drawio-export.pdf"
        subprocess.run(
            ["drawio", "--export", "--theme", "light", "--output", str(source), str(ROOT / f"{STEM}.drawio")],
            check=True,
        )
        bounds = subprocess.run(
            ["gs", "-q", "-dBATCH", "-dNOPAUSE", "-sDEVICE=bbox", str(source)],
            check=True, capture_output=True, text=True,
        )
        match = re.search(r"%%HiResBoundingBox: ([^\n]+)", bounds.stderr)
        if not match:
            raise RuntimeError("No visible diagram in the draw.io PDF export.")
        left, bottom, right, top = (float(value) for value in match.group(1).split())
        if right <= left or top <= bottom:
            raise RuntimeError("Empty draw.io PDF export.")

        width = 178 / 25.4 * 72
        margin = 5
        scale = (width - 2 * margin) / (right - left)
        height = scale * (top - bottom) + 2 * margin
        page = PdfReader(source).pages[0]
        writer = PdfWriter()
        canvas = writer.add_blank_page(width, height)
        transform = Transformation().scale(scale).translate(margin - scale * left, margin - scale * bottom)
        canvas.merge_transformed_page(page, transform)
        writer.write(ROOT / f"{STEM}.pdf")
    subprocess.run(
        ["pdftoppm", "-r", "450", "-singlefile", "-png", str(ROOT / f"{STEM}.pdf"), str(ROOT / STEM)],
        check=True,
    )


if __name__ == "__main__":
    draw()
