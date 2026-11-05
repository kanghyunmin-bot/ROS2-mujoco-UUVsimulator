# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Create source-derived IEEE DOCX and a Korean reading copy using existing evidence."""

import hashlib
import json
import re
from copy import deepcopy
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt
from lxml import etree as E

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / "output/doc"
OUT.mkdir(parents=True, exist_ok=True)
M = json.loads((HERE / "manuscript.json").read_text())
W = "http://purl.oclc.org/ooxml/wordprocessingml/main"
R = "http://purl.oclc.org/ooxml/officeDocument/relationships"
NS = {"w": W, "r": R}
REF = HERE / "template/ieee-conference-a4.docx"
with ZipFile(REF) as z:
    base = {i.filename: z.read(i.filename) for i in z.infolist()}

# Preserve the retained template package. python-docx constructs only new body
# components; all old style, numbering and theme bytes remain untouched.
d = Document()
for e in list(d._element.body):
    d._element.body.remove(e)


def style(p, name):
    pr = p._p.get_or_add_pPr()
    e = OxmlElement("w:pStyle")
    e.set(qn("w:val"), name)
    pr.insert(0, e)
    n = OxmlElement("w:numPr")
    ni = OxmlElement("w:numId")
    ni.set(qn("w:val"), "0")
    n.append(ni)
    pr.append(n)


def para(
    text="",
    role="BodyText",
    size=11,
    align=WD_ALIGN_PARAGRAPH.JUSTIFY,
    bold=False,
    after=3,
    before=0,
):
    p = d.add_paragraph()
    style(p, role)
    f = p.paragraph_format
    f.alignment = align
    f.space_after = Pt(after)
    f.space_before = Pt(before)
    f.line_spacing = 1
    f.first_line_indent = Pt(10) if role == "BodyText" else Pt(0)
    f.widow_control = True
    r = p.add_run(text)
    r.font.name = "Times New Roman"
    r.font.size = Pt(size)
    r.bold = bold
    return p


def head(text):
    p = para(text, "Heading5", 10, WD_ALIGN_PARAGRAPH.CENTER, after=4, before=8)
    p.paragraph_format.keep_with_next = True
    return p


def picture(file, width=7):
    p = para("", align=WD_ALIGN_PARAGRAPH.CENTER, after=0)
    p.paragraph_format.keep_with_next = True
    r = p.add_run()
    r.add_picture(str(HERE / "figures" / file), width=Inches(width))
    for node in p._p.iter(qn("wp:docPr")):
        node.set("descr", M["figures"][0 if file.startswith("fig1") else 1]["caption"])


def section(cols, start="continuous", top=54):
    # Use the reference's A4 body page geometry, with a single-author title block.
    sp = OxmlElement("w:sectPr")
    ty = OxmlElement("w:type")
    ty.set(qn("w:val"), start)
    sp.append(ty)
    sz = OxmlElement("w:pgSz")
    sz.set(qn("w:w"), "11906")
    sz.set(qn("w:h"), "16838")
    sp.append(sz)
    mar = OxmlElement("w:pgMar")
    for k, v in {
        "top": top * 20,
        "bottom": 1440,
        "left": 907,
        "right": 907,
        "header": 720,
        "footer": 720,
        "gutter": 0,
    }.items():
        mar.set(qn("w:" + k), str(v))
    sp.append(mar)
    c = OxmlElement("w:cols")
    c.set(qn("w:num"), str(cols))
    c.set(qn("w:space"), "360")
    sp.append(c)
    return sp


def end_section(cols, start="continuous", top=54):
    p = para("", size=1, after=0)
    p.paragraph_format.line_spacing = Pt(1)
    p._p.get_or_add_pPr().append(section(cols, start, top))


para(M["title"], "papertitle", 20, WD_ALIGN_PARAGRAPH.CENTER, after=8)
a = M["author"]
para(a["name"], "Author", 11, WD_ALIGN_PARAGRAPH.CENTER, after=2)
para(
    a["department"] + ", " + a["institution"],
    "Affiliation",
    10,
    WD_ALIGN_PARAGRAPH.CENTER,
    after=1,
)
para(
    a["location"] + " | " + a["email"],
    "Affiliation",
    10,
    WD_ALIGN_PARAGRAPH.CENTER,
    after=7,
)
para("Extended Abstract", "BodyText", 10, WD_ALIGN_PARAGRAPH.CENTER, after=7)
para("Keywords—" + M["keywords"], "Keywords", 9, after=5)
for t in M["paragraphs"][: M["page_one_paragraphs"]]:
    para(t)
picture(M["figures"][0]["file"])
para(M["figures"][0]["caption"], "figurecaption", 8, after=8)
end_section(1, "nextPage", 54)
picture(M["figures"][1]["file"])
para(M["figures"][1]["caption"], "figurecaption", 8, after=8)
for t in M["paragraphs"][M["page_one_paragraphs"] :]:
    para(t)
picture(M["figures"][2]["file"])
para(M["figures"][2]["caption"], "figurecaption", 8, after=5)
head("References")
for t in M["references"]:
    p = para(t, "references", 8, after=3)
    p.paragraph_format.left_indent = Pt(12)
    p.paragraph_format.first_line_indent = Pt(-12)
d._element.body.append(section(1, "nextPage", 54))
tmp = HERE / "body_components.docx"
d.save(tmp)
with ZipFile(tmp) as z:
    generated = {i.filename: z.read(i.filename) for i in z.infolist()}
groot = E.fromstring(generated["word/document.xml"])
gbody = groot.find("{http://schemas.openxmlformats.org/wordprocessingml/2006/main}body")
bodyxml = E.tostring(gbody)
replacements = {
    b"http://schemas.openxmlformats.org/wordprocessingml/2006/main": W.encode(),
    b"http://schemas.openxmlformats.org/officeDocument/2006/relationships": R.encode(),
    b"http://schemas.openxmlformats.org/drawingml/2006/main": b"http://purl.oclc.org/ooxml/drawingml/main",
    b"http://schemas.openxmlformats.org/drawingml/2006/picture": b"http://purl.oclc.org/ooxml/drawingml/picture",
    b"http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing": b"http://purl.oclc.org/ooxml/drawingml/wordprocessingDrawing",
}
for old, new in replacements.items():
    bodyxml = bodyxml.replace(old, new)
relroot = E.fromstring(base["word/_rels/document.xml.rels"])
grels = E.fromstring(generated["word/_rels/document.xml.rels"])
for r in grels:
    if r.get("Type").endswith("/image"):
        oldid = r.get("Id")
        newid = "rIdUT" + oldid
        bodyxml = bodyxml.replace(
            ('"' + oldid + '"').encode(), ('"' + newid + '"').encode()
        )
        nr = deepcopy(r)
        nr.set("Id", newid)
        nr.set("Type", R + "/image")
        nr.set("Target", "media/ut27_" + Path(r.get("Target")).name)
        relroot.append(nr)
        base["word/" + nr.get("Target")] = generated["word/" + r.get("Target")]
orig = E.fromstring(base["word/document.xml"])
oldbody = orig.find("w:body", NS)
orig.replace(oldbody, E.fromstring(bodyxml))
base["word/document.xml"] = E.tostring(
    orig, xml_declaration=True, encoding="UTF-8", standalone=True
)
base["word/_rels/document.xml.rels"] = E.tostring(
    relroot, xml_declaration=True, encoding="UTF-8", standalone=True
)
ct = E.fromstring(base["[Content_Types].xml"])
ctns = "http://schemas.openxmlformats.org/package/2006/content-types"
if not any(e.get("Extension") == "png" for e in ct):
    E.SubElement(ct, "{" + ctns + "}Default", Extension="png", ContentType="image/png")
base["[Content_Types].xml"] = E.tostring(
    ct, xml_declaration=True, encoding="UTF-8", standalone=True
)
# Remove unused template example text in footer and footnote parts as well.
for name in list(base):
    if re.match(r"word/(footer\d+|header\d+|footnotes|endnotes)\.xml$", name):
        rr = E.fromstring(base[name])
        for e in rr.findall(".//w:t", NS):
            e.text = ""
        base[name] = E.tostring(
            rr, xml_declaration=True, encoding="UTF-8", standalone=True
        )
core = E.fromstring(base["docProps/core.xml"])
for name, value in [("title", M["title"]), ("creator", a["name"])]:
    e = core.find("{http://purl.org/dc/elements/1.1/}" + name)
    if e is not None:
        e.text = value
e = core.find(
    "{http://schemas.openxmlformats.org/package/2006/metadata/core-properties}lastModifiedBy"
)
if e is not None:
    e.text = a["name"]
base["docProps/core.xml"] = E.tostring(
    core, xml_declaration=True, encoding="UTF-8", standalone=True
)
final = OUT / "ut27_abstract_en.docx"
with ZipFile(final, "w", ZIP_DEFLATED) as z:
    for name, data in base.items():
        z.writestr(name, data)
tmp.unlink()

# A Korean reading copy, preserving paragraph order of the Korean draft. It is
# not the upload file and is not represented as an official Korean template.
ko = Document()
sec = ko.sections[0]
sec.page_width = Inches(8.2677)
sec.page_height = Inches(11.6929)
sec.top_margin = sec.bottom_margin = Inches(0.7)
sec.left_margin = sec.right_margin = Inches(0.7)
for sn in ["Normal", "Title", "Heading 1", "Heading 2"]:
    s = ko.styles[sn]
    s.font.name = "NanumMyeongjo"
    s.font.color.rgb = __import__("docx").shared.RGBColor(0, 0, 0)
    s.element.get_or_add_rPr().append(
        E.fromstring(
            '<w:rFonts xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" w:eastAsia="NanumMyeongjo"/>'
        )
    )
ko.styles["Normal"].font.size = Pt(10)
ko.styles["Normal"].paragraph_format.line_spacing = 1.15
ko.styles["Normal"].paragraph_format.space_after = Pt(5)
ko.styles["Title"].font.size = Pt(18)
for line in (HERE / "abstract_ko.md").read_text().splitlines():
    if not line.strip():
        continue
    if line.startswith("# "):
        ko.add_paragraph(line[2:].replace("플랫폼의 설계", "플랫폼의\n설계"), "Title")
    elif line.startswith("## "):
        if line == "## 참고문헌":
            ko.add_page_break()
            for filename, caption in [
                (
                    "fig1_platform_architecture.png",
                    "그림 1. UUV-SIM의 SITL·물리·관측·Real2Sim·VLA 연결 구조.",
                ),
                (
                    "fig2_platform_views.png",
                    "그림 2. 같은 모델·재질·상태에서 렌더한 (a) 수면 조명과 수조 전경, (b) 수중 로봇 근접 장면, (c) 부표·흰 밧줄·연결부의 근접 장면.",
                ),
                ("fig3_response_comparison.png", "그림 3. (a,b) 실물·명목·최신 보정 모델의 전진 속도. (c) 시간 보정 모델 대비 응답 보정 후 회전 해제 각속도 RMSE. 선택된 모든 2.5초 해제 구간을 평가했다."),
            ]:
                ko.add_picture(str(HERE / "figures" / filename), width=Inches(6.75))
                p = ko.add_paragraph(caption)
                for run in p.runs:
                    run.font.size = Pt(8)
        ko.add_paragraph(line[3:], "Heading 2")
    else:
        ko.add_paragraph(line)
ko.save(OUT / "ut27_abstract_ko.docx")


def words(text: str) -> int:
    return len(re.findall(r"\S+", text))


main = " ".join(M["paragraphs"])
full = " ".join(
    [main, M["title"], M["keywords"]]
    + M["references"]
    + [f["caption"] for f in M["figures"]]
)
(HERE / "word_count.json").write_text(
    json.dumps(
        {
            "main_text_words": words(main),
            "text_with_title_keywords_references_captions": words(full),
            "count_method": "whitespace-delimited tokens",
            "requirement": "500-1000 words, 2 pages",
        },
        indent=2,
    )
    + "\n"
)
with ZipFile(REF) as z:
    preserved = {
        n: hashlib.sha256(base[n]).hexdigest()
        for n in ["word/styles.xml", "word/numbering.xml", "word/theme/theme1.xml"]
    }
    assert all(base[n] == z.read(n) for n in preserved)
(HERE / "template_preservation.json").write_text(
    json.dumps(
        {
            "template_sha256": hashlib.sha256(REF.read_bytes()).hexdigest(),
            "preserved_parts": preserved,
            "geometry": "A4, margins 54/72/45.35/45.35 pt, single-column extended abstract, 11 pt body",
            "intentional_adaptations": [
                "single author",
                "three full-width figures",
                "single-column extended abstract rather than full-paper sections",
                "20 pt title and 11 pt body for abstract readability",
                "removed example and copyright placeholders",
            ],
        },
        indent=2,
    )
    + "\n"
)
print(final)

md = [
    "# " + M["title"],
    a["name"] + " | " + a["department"] + ", " + a["institution"],
    a["location"] + " | " + a["email"],
    "Extended Abstract",
    "Keywords: " + M["keywords"],
]
md.extend(M["paragraphs"])
md.extend(["## References"] + M["references"])
md.extend(["## Figure captions"] + [f["caption"] for f in M["figures"]])
(HERE / "abstract_en.md").write_text(
    "\n\n".join(md).replace("N s2/m2", "N·s²/m²") + "\n"
)
