/** Build the BlueROV2 framework figure as an editable PPTX and print SVG. */
import fs from "node:fs/promises";
import { execFileSync } from "node:child_process";
import os from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import { createRequire } from "node:module";

const here = dirname(fileURLToPath(import.meta.url));
const runtimeModules = process.env.ARTIFACT_NODE_MODULES ??
  join(os.homedir(), ".cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules");
const require = createRequire(join(runtimeModules, "figure-loader.cjs"));
const { Presentation, PresentationFile } = await import(pathToFileURL(require.resolve("@oai/artifact-tool")));
const stem = "figure-04-mujoco-framework";
const width = 850;
const height = 498;
const FONT = "DejaVu Sans";
const ink = "#182434", gray = "#495A6B", rule = "#CAD5DF";
const navy = "#143F78", teal = "#146458", plum = "#654280";
const white = "#FFFFFF", light = "#F4F7FA", blueBg = "#EFF5FC", tealBg = "#EAF5F2", purpleBg = "#F4EFF8";
const presentation = Presentation.create({ slideSize: { width, height } });
const page = presentation.slides.add();
page.background.fill = white;
const svg = [];
const invisible = { fill: "none", width: 0 };
const pos = (left, top, width, height) => ({ left, top, width, height });
const esc = text => text.replaceAll("&", "&amp;").replaceAll("<", "&lt;").replaceAll(">", "&gt;")
  .replaceAll('"', "&quot;");

function text(name, value, x, y, w, h, { size = 14, color = ink, bold = false, align = "left" } = {}) {
  const shape = page.shapes.add({ name, geometry: "textbox", position: pos(x, y, w, h), line: invisible, fill: "none" });
  shape.text = value;
  shape.text.style = { typeface: FONT, fontSize: size, color, bold, alignment: align,
    verticalAlignment: "middle", wrap: "none", insets: 0 };
  const dx = align === "center" ? w/2 : align === "right" ? w : 0;
  const anchor = align === "center" ? "middle" : align === "right" ? "end" : "start";
  svg.push(`<text x="${x+dx}" y="${y+h/2}" fill="${color}" font-size="${size}" `+
    `font-weight="${bold ? 700 : 400}" text-anchor="${anchor}" dominant-baseline="central">${esc(value)}</text>`);
}

function rect(name, x, y, w, h, fill, border = "none", radius = 0) {
  page.shapes.add({ name, geometry: radius ? "roundRect" : "rect", position: pos(x, y, w, h),
    fill, line: { fill: border, width: border === "none" ? 0 : 1 },
    ...(radius ? { borderRadius: radius } : {}) });
  svg.push(`<rect x="${x}" y="${y}" width="${w}" height="${h}" rx="${radius}" fill="${fill}" stroke="${border}"/>`);
}

function path(name, points, stroke, fill = "none", closed = false, dashed = false, thickness = 1.6) {
  const left = Math.min(...points.map(p=>p[0])), top = Math.min(...points.map(p=>p[1]));
  const w = Math.max(1, ...points.map(p=>p[0]-left)), h = Math.max(1, ...points.map(p=>p[1]-top));
  const commands = points.map(([x,y],i)=>({[i === 0 ? "moveTo" : "lineTo"]: { x:x-left, y:y-top }}));
  if (closed) commands.push({ close: {} });
  page.shapes.add({ name, geometry: "custom", position: pos(left,top,w,h), fill,
    line: { fill: stroke, width: stroke === "none" ? 0 : thickness, style: dashed ? "dashed" : "solid" },
    customPaths: [{ width: w, height: h, commands }] });
  const d = points.map(([x,y],i)=>`${i === 0 ? "M" : "L"}${x} ${y}`).join(" ") + (closed ? " Z" : "");
  svg.push(`<path d="${d}" fill="${fill}" stroke="${stroke}" stroke-width="${thickness}" stroke-linejoin="round"`+
    (dashed ? ' stroke-dasharray="5 4"' : "") + "/>");
}

function arrow(name, points, color, dashed = false) {
  const [x,y] = points.at(-1), [px,py] = points.at(-2);
  const mag = Math.hypot(x-px,y-py), ux = (x-px)/mag, uy = (y-py)/mag;
  const bx = x-7*ux, by = y-7*uy;
  path(name, [...points.slice(0,-1),[x-3*ux,y-3*uy]], color, "none", false, dashed);
  path(name+" arrowhead", [[x,y],[bx-3.4*uy,by+3.4*ux],[bx+3.4*uy,by-3.4*ux]], "none", color, true);
}

async function logo(name, file, x,y,w,h) {
  const bytes = await fs.readFile(join(here,"data","framework-logos",file));
  const contentType = file.endsWith(".svg") ? "image/svg+xml" : "image/png";
  page.images.add({ blob: new Uint8Array(bytes), contentType, alt: name, fit: "contain", position: pos(x,y,w,h) });
  svg.push(`<image x="${x}" y="${y}" width="${w}" height="${h}" href="data:${contentType};base64,${bytes.toString("base64")}"/>`);
}

function node(name,title,sub,x,y,w,h,{color=navy,fill=blueBg,detailSize=10.9,titleSize=14}={}) {
  rect(name,x,y,w,h,fill,color,6);
  text(name+" title",title,x+5,y+5,w-10,22,{size:titleSize,color,bold:true,align:"center"});
  text(name+" detail",sub,x+4,y+27,w-8,15,{size:detailSize,color:gray,align:"center"});
}

const engineFill = "#EAF3FF", engineEdge = "#6597D0";
const pythonFill = "#FFF4CD", pythonEdge = "#C69B34";
const interfaceFill = "#FFF0E4", interfaceEdge = "#CD804A";
const coreFill = "#FBE8A8", coreEdge = "#B28222";
const engineNode = {color:"#204A78",fill:"#F7FAFF",titleSize:13};
const coreNode = {color:"#6F5012",fill:"#FFFAE9",titleSize:13};
const interfaceNode = {color:"#8B4B20",fill:"#FFFAF5",titleSize:13};
const runtimeNode = {color:"#6F5012",fill:"#FFFBED",titleSize:12.5};

rect("MuJoCo engine lane",28,83,229,394,engineFill,engineEdge,26);
rect("Python runtime lane",302,83,253,394,pythonFill,pythonEdge,26);
rect("External interfaces lane",600,83,225,394,interfaceFill,interfaceEdge,26);
rect("MuJoCo logo field",80,19,125,31,"#172D47","none",4);
await logo("MuJoCo wordmark","mujoco.svg",90,23,105,22);
text("Engine heading","Physics engine",28,55,229,19,{size:14,bold:true,align:"center"});
await logo("Python logo","python.png",357,22,30,34);
text("Runtime heading","Python runtime",397,23,141,31,{size:15,bold:true});
await logo("ROS 2 logo","ros2.svg",659,21,108,27);
text("Interface heading","ROS 2 / operator tools",600,55,225,19,{size:14,bold:true,align:"center"});

rect("Vehicle model boundary",320,170,222,152,coreFill,coreEdge,19);

// Engine processing and Python force application.
arrow("Scene to physics",[[142,156],[142,195]],navy);
arrow("Physics to updated state",[[142,240],[142,263]],navy);
arrow("State to readout",[[142,307],[142,342]],navy);
arrow("State to renderer",[[142,388],[142,422]],navy);
arrow("T200 thrust to physics",[[338,210],[233,210]],navy);
arrow("Fluid wrench to physics",[[338,271],[278,271],[278,228],[233,228]],navy);
arrow("Body state to fluid model",[[233,289],[338,289]],navy);
text("Thrust label","Thrust",265,188,55,16,{size:10.6,color:navy,align:"center"});
text("Wrench label","Fluid loads",283,248,52,15,{size:8.8,color:navy,align:"center"});
text("Body state label","Pose + velocity",245,293,81,17,{size:9.2,color:navy,align:"center"});

// Load the fixed vehicle profile before starting the live PWM loop.
arrow("Load vehicle profile",[[368,149],[368,170]],"#8A6E2B",true);
path("SITL command lane",[[491,149],[491,162],[428,162]],navy);
arrow("Final servo PWM",[[428,162],[428,188]],navy);
arrow("Live pilot commands",[[623,120],[546,120]],navy);
arrow("ROS 2 RC commands",[[623,194],[592,194],[592,132],[546,132]],navy);
text("Live commands label","Pilot input",553,103,64,14,{size:9.5,color:navy,align:"center"});

// Publish live sensor data and return IMU/pressure to SITL.
arrow("State to sensor interface",[[233,365],[338,365]],navy);
arrow("Publish live sensors",[[522,364],[623,364]],navy);
arrow("Camera and telemetry output",[[586,364],[586,278],[623,278]],navy);
arrow("SITL JSON feedback",[[574,364],[574,144],[546,144]],navy,true);
text("SITL feedback label","SITL JSON",583,219,76,15,{size:9.5,color:navy});
text("SITL feedback detail","IMU / pressure",583,233,92,15,{size:8.8,color:gray});

node("Scene","MJCF vehicle + scene","UT27 / BlueROV2 · pool / tank",52,110,181,46,engineNode);
node("Physics","Physics step · mj_step","Rigid-body motion and contact",52,195,181,45,engineNode);
node("Updated state","Updated body state","Pose · linear / angular velocity",52,263,181,44,engineNode);
node("State readout","State / sensor readout","Body state · IMU · pressure",52,342,181,46,engineNode);
node("Rendering","MuJoCo rendering","Viewer · camera frames",52,422,181,46,engineNode);

node("Vehicle profile","Vehicle profile","Load before run",312,103,112,46,runtimeNode);
node("Flight controller","ArduSub SITL","6 / 8 motor mixer",435,103,111,46,runtimeNode);
node("Propulsion","T200 actuator model","PWM + voltage → RPM → thrust",338,188,184,46,coreNode);
node("Fluid model","Hydrodynamic model","Linear / quadratic drag · buoyancy",338,258,184,47,
     {...coreNode,detailSize:9.5});
text("Vehicle-specific fluid","BlueROV2: whole-body  ·  UT27: distributed option",323,304,216,17,
  {size:8.0,color:navy,align:"center"});
node("Sensor interface","Sensor / video bridge","IMU · Bar30 · camera frames",338,341,184,46,
  {...runtimeNode,titleSize:12.5});
text("Sensor models","Sensor timing and noise models",338,393,184,17,{size:9.0,color:gray,align:"center"});

node("Pilot","Live command input","Web GUI / QGroundControl",623,103,181,46,interfaceNode);
node("ROS commands","ROS 2 commands","RC override · arm / mode",623,171,181,46,interfaceNode);
node("Live monitoring","Visualization / video","Web dashboard · QGC video",623,255,181,46,interfaceNode);
node("ROS output","ROS 2 topics","IMU · depth · TF · diagnostics",623,341,181,46,interfaceNode);
text("Optional vehicle sensors","UT27 options: DVL · sonar · hydrophone",609,397,205,16,
  {size:9.1,color:"#8B4B20",align:"center"});
page.speakerNotes.textFrame.setText("Technical basis: UUV-Sim runtime architecture; BlueROV2 model contract; simulation_step_physics.py; underwater_wrench_runtime.py; ros2_sitl_sensor_transport.py. Logo sources: python.org/community/logos/; mujoco.org; ros.org/blog/media/.");
const drawing = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${width} ${height}" `+
  `font-family="${FONT}" width="${width}" height="${height}"><rect width="100%" height="100%" fill="white"/>`+
  svg.join("\n")+"</svg>\n";
await fs.writeFile(join(here,stem+".svg"), drawing);
const deck = await PresentationFile.exportPptx(presentation);
await deck.save(join(here,stem+".pptx"));
const hMm = (178*height/width).toFixed(6);
const html = `<!doctype html><meta charset="utf-8"><style>@page{size:178mm ${hMm}mm;margin:0}`+
  `body{margin:0}svg{display:block;width:178mm;height:${hMm}mm}</style>`+drawing;
const scratch = await fs.mkdtemp(join(os.tmpdir(), "uuv-framework-export-"));
try {
  const htmlPath = join(scratch, "print.html");
  await fs.writeFile(htmlPath, html);
  const pdfPath = join(here, stem + ".pdf");
  execFileSync(process.env.CHROME ?? "google-chrome", [
    "--headless", "--disable-gpu", "--no-pdf-header-footer",
    "--user-data-dir=" + join(scratch, "chrome"), "--print-to-pdf=" + pdfPath, pathToFileURL(htmlPath).href,
  ], { stdio: "pipe" });
  execFileSync("pdftoppm", ["-r", "450", "-singlefile", "-png", pdfPath, join(here, stem)]);
} finally {
  await fs.rm(scratch, { recursive: true, force: true });
}
