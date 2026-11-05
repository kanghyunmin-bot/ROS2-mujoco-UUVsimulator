# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Verify rendered UT27 artifacts without running new simulation experiments."""

import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
from zipfile import ZipFile

from lxml import etree
from pypdf import PdfReader

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
W = "http://purl.oclc.org/ooxml/wordprocessingml/main"
RENDER = Path(sys.argv[1])
words = json.loads((HERE / "word_count.json").read_text())
assert 500 <= words["main_text_words"] <= 1000
verification = json.loads((HERE / "figure_verification.json").read_text())
assert all(c["matches_recorded_summary"] for c in verification["checks"])
for name, digest in verification["source_files"].items():
    assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest
platform = json.loads((HERE / "platform_verification.json").read_text())
for name, digest in platform["source_sha256"].items():
    assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest

capture = platform["capture"]
for name, digest in capture["source_sha256"].items():
    assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest
for name, digest in capture["panels"].items():
    assert hashlib.sha256((HERE / "figures/platform_capture" / (name + ".png")).read_bytes()).hexdigest() == digest

response = json.loads((HERE / "response_verification.json").read_text())
assert all(c["matches_recorded_summary"] for c in response["checks"])
for name, digest in response["source_sha256"].items():
    assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest

report = {
    "format": "UT27 two-page extended abstract; single column is an editorial choice",
    "main_text_words": words["main_text_words"],
    "new_simulation_runs": 0,
    "verified_replay_cases": len(verification["checks"]),
    "verified_xy_yaw_values": 2 * len(verification["checks"]),
    "author_confirmation": "pending",
    "external_submission": False,
    "notion_modified": False,
    "topic": "UUV-SIM platform design and implementation",
    "platform_evidence": platform,
    "response_evidence": response,
    "artifacts": {},
    "metric_source_sha256": {},
}
for rel in [
    "docs/assets/real2sim-application-20260915/comparison.json",
    "docs/assets/yaw-timing-20260915/full_comparison.json",
    "docs/assets/yaw-timing-20260915/clock_regression.json",
    "docs/assets/yaw-timing-20260915/fcu_timing_summary.json",
    "docs/assets/yaw-timing-20260915/comparison.json",
]:
    report["metric_source_sha256"][rel] = hashlib.sha256(
        (ROOT / rel).read_bytes()
    ).hexdigest()


def embedded(font):
    font = font.get_object()
    if "/DescendantFonts" in font:
        return all(embedded(f) for f in font["/DescendantFonts"])
    descriptor = font.get("/FontDescriptor")
    return bool(
        descriptor
        and any(
            k in descriptor.get_object()
            for k in ["/FontFile", "/FontFile2", "/FontFile3"]
        )
    )


for lang in ["en", "ko"]:
    name = f"ut27_abstract_{lang}"
    pdf = ROOT / "output/pdf" / f"{name}.pdf"
    docx = ROOT / "output/doc" / f"{name}.docx"
    reader = PdfReader(pdf)
    assert len(reader.pages) == 2, (lang, len(reader.pages))
    text = subprocess.check_output(["pdftotext", "-layout", str(pdf), "-"], text=True)
    count = len(re.findall(r"\S+", text))
    if lang == "en":
        assert 500 <= count <= 1000, count
        assert "Extended Abstract" in text
        for required in [
            "UUV-SIM",
            "ellipsoid",
            "SITL",
            "LeRobot",
            "2.65",
            "0.47",
            "2.51",
            "0.505",
            "0.0298",
            "51",
            "36",
            "10.1126/scirobotics.abm6074",
        ]:
            assert required in text, required
        for forbidden in [
            "First Name",
            "xxx-x-xxxx",
            "I. INTRODUCTION",
            "III. RESULTS",
            "Abstract—",
        ]:
            assert forbidden not in text, forbidden
        with (
            ZipFile(docx) as z,
            ZipFile(HERE / "template/ieee-conference-a4.docx") as source,
        ):
            for part in [
                "word/styles.xml",
                "word/numbering.xml",
                "word/theme/theme1.xml",
            ]:
                assert z.read(part) == source.read(part), part
            tree = etree.fromstring(z.read("word/document.xml"))
            columns = tree.findall(f".//{{{W}}}cols")
            assert columns and all(c.get(f"{{{W}}}num") == "1" for c in columns)
    fonts = set()
    for page in reader.pages:
        assert abs(float(page.mediabox.width) - 595.3) < 1
        assert abs(float(page.mediabox.height) - 841.9) < 1
        for font in page["/Resources"].get("/Font", {}).values():
            assert embedded(font), font
            fonts.add(str(font.get_object().get("/BaseFont")))
    pngs = list((RENDER / lang).glob("page-*.png"))
    assert len(pngs) == len(reader.pages), (lang, pngs)
    report["artifacts"][lang] = {
        "pages": len(reader.pages),
        "page_size": "A4",
        "pdf_text_words_excluding_raster_labels": count,
        "embedded_fonts": sorted(fonts),
        "pdf_sha256": hashlib.sha256(pdf.read_bytes()).hexdigest(),
        "docx_sha256": hashlib.sha256(docx.read_bytes()).hexdigest(),
        "rendered_pages": [str(p) for p in sorted(pngs)],
        "visual_review": "pending",
    }
(HERE / "qa.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
print(json.dumps(report, ensure_ascii=False, indent=2))
