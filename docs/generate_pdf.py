#!/usr/bin/env python3
"""
generate_pdf.py — HAB Payload Stabilization System design document generator.

Produces docs/HAB_Stabilization_v2.pdf.

This script is the source of truth for the PDF. All hardware values below are
kept in sync with src/config.h. When config.h changes, update the CONSTANTS
block and regenerate:

    python3 docs/generate_pdf.py

Rev 1.1 (June 2026): Corrected encoder interface to reflect AS-BUILT state
(AS5048A on PWM, not SPI), corrected SimpleFOC motor pin map to match
config.h (IN1->D9, IN2->D10, IN3->D5, EN->D6), added a "Known Issue / Next
Upgrade" note documenting the PWM->SPI plan after the June 2026 launch.
"""

from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.lib import colors
from reportlab.platypus import (
    BaseDocTemplate, PageTemplate, Frame, Paragraph, Spacer, Table,
    TableStyle, KeepTogether,
)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT

# ── Values mirrored from src/config.h (keep in sync) ─────────────────────────
CFG = {
    "IN1": "D9 (PB8, TIM4_CH3)",
    "IN2": "D10 (PB9, TIM4_CH4)",
    "IN3": "D5 (PC7, TIM3_CH2)",
    "EN":  "D6 (PC6)",
    "ENCODER": "D11 (PC3) — PWM single-wire",
    "BNO085_ADDR": "0x4A",
    "SUPPLY_V": "9.0 V",
    "VOLT_LIMIT": "7.0 V",
    "TELEM": "USB CDC serial, 115200 baud, every 30 s",
    "PID": "Kp=0.50  Ki=0.02  Kd=0.08",
    "PID_RATE": "100 Hz",
    "FOC_RATE": "1000 Hz",
    "DEADBAND": "1.0\u00b0",
    "MAG_CUTOFF": "15 000 m",
    "BATT_CUTOFF": "7.2 V",
    "BATT_WARN": "7.8 V",
}

# ── Palette (matched to the original document) ───────────────────────────────
NAVY   = colors.HexColor("#1f2d4d")
BLUE   = colors.HexColor("#2f6fdb")
LIGHT  = colors.HexColor("#f5f7fb")
ROW_A  = colors.HexColor("#ffffff")
ROW_B  = colors.HexColor("#f3f6fb")
GREY   = colors.HexColor("#5a6478")
AMBER  = colors.HexColor("#b9770a")
AMBERBG= colors.HexColor("#fdf4e3")
CALLBG = colors.HexColor("#eaf1fc")
CALLBD = colors.HexColor("#bcd2f2")
LINE   = colors.HexColor("#d4dae6")

PAGE_W, PAGE_H = letter
MARGIN = 0.9 * inch

# ── Styles ───────────────────────────────────────────────────────────────────
ss = getSampleStyleSheet()

def style(name, **kw):
    return ParagraphStyle(name, parent=ss["Normal"], **kw)

S_TITLE   = style("title", fontName="Helvetica-Bold", fontSize=22, leading=26,
                  alignment=TA_CENTER, textColor=NAVY, spaceAfter=4)
S_SUB     = style("sub", fontName="Helvetica", fontSize=11, leading=14,
                  alignment=TA_CENTER, textColor=GREY)
S_SUB2    = style("sub2", fontName="Helvetica-Oblique", fontSize=9.5, leading=12,
                  alignment=TA_CENTER, textColor=GREY, spaceAfter=10)
S_H1      = style("h1", fontName="Helvetica-Bold", fontSize=14, leading=18,
                  textColor=NAVY, spaceBefore=14, spaceAfter=6)
S_H2      = style("h2", fontName="Helvetica-Bold", fontSize=11.5, leading=15,
                  textColor=NAVY, spaceBefore=8, spaceAfter=3)
S_BODY    = style("body", fontName="Helvetica", fontSize=9.8, leading=13.5,
                  textColor=colors.HexColor("#1c2230"), spaceAfter=6)
S_CAP     = style("cap", fontName="Helvetica-Oblique", fontSize=8.3, leading=11,
                  alignment=TA_CENTER, textColor=GREY, spaceAfter=8)
S_CALL    = style("call", fontName="Helvetica-Bold", fontSize=9.8, leading=14,
                  textColor=NAVY)
S_NOTE    = style("note", fontName="Helvetica", fontSize=9.5, leading=13.5,
                  textColor=colors.HexColor("#3a2c08"))
S_NOTEH   = style("noteh", fontName="Helvetica-Bold", fontSize=10.5, leading=14,
                  textColor=AMBER, spaceAfter=3)
S_CELL    = style("cell", fontName="Helvetica", fontSize=8.6, leading=11,
                  textColor=colors.HexColor("#1c2230"))
S_CELLB   = style("cellb", fontName="Helvetica-Bold", fontSize=8.6, leading=11,
                  textColor=NAVY)
S_TH      = style("th", fontName="Helvetica-Bold", fontSize=8.6, leading=11,
                  textColor=colors.white)


def header_footer(canvas, doc):
    canvas.saveState()
    # Header band
    canvas.setFillColor(NAVY)
    canvas.rect(0, PAGE_H - 0.75 * inch, PAGE_W, 0.75 * inch, fill=1, stroke=0)
    canvas.setFillColor(BLUE)
    canvas.rect(0, PAGE_H - 0.80 * inch, PAGE_W, 0.05 * inch, fill=1, stroke=0)
    # Footer band
    canvas.setFillColor(NAVY)
    canvas.rect(0, 0, PAGE_W, 0.5 * inch, fill=1, stroke=0)
    canvas.setFillColor(BLUE)
    canvas.rect(0, 0.5 * inch, PAGE_W, 0.04 * inch, fill=1, stroke=0)
    canvas.setFillColor(colors.HexColor("#aeb8cc"))
    canvas.setFont("Helvetica", 8)
    canvas.drawCentredString(
        PAGE_W / 2, 0.22 * inch,
        f"HAB Payload Stabilization System  |  K6ATV  |  As-Built + Roadmap  |  "
        f"June 2026  |  Page {doc.page}")
    canvas.restoreState()


def make_table(data, col_widths, header=True, font=S_CELL):
    """Alternating-row table styled like the original."""
    body = []
    for r, row in enumerate(data):
        cells = []
        for c, val in enumerate(row):
            if header and r == 0:
                cells.append(Paragraph(str(val), S_TH))
            elif c == 0:
                cells.append(Paragraph(str(val), S_CELLB))
            else:
                cells.append(Paragraph(str(val), font))
        body.append(cells)
    t = Table(body, colWidths=col_widths, hAlign="LEFT")
    sty = [
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
        ("LINEBELOW", (0, 0), (-1, -1), 0.4, LINE),
        ("BOX", (0, 0), (-1, -1), 0.4, LINE),
    ]
    if header:
        sty += [("BACKGROUND", (0, 0), (-1, 0), NAVY)]
        for r in range(1, len(data)):
            sty.append(("BACKGROUND", (0, r), (-1, r), ROW_A if r % 2 else ROW_B))
    else:
        for r in range(len(data)):
            sty.append(("BACKGROUND", (0, r), (-1, r), ROW_A if r % 2 else ROW_B))
    t.setStyle(TableStyle(sty))
    return t


def callout(text):
    p = Paragraph(text, S_CALL)
    t = Table([[p]], colWidths=[PAGE_W - 2 * MARGIN])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), CALLBG),
        ("BOX", (0, 0), (-1, -1), 0.8, CALLBD),
        ("LEFTPADDING", (0, 0), (-1, -1), 10),
        ("RIGHTPADDING", (0, 0), (-1, -1), 10),
        ("TOPPADDING", (0, 0), (-1, -1), 8),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 8),
    ]))
    return t


def note_box(title, body_lines):
    rows = [[Paragraph(title, S_NOTEH)]]
    for ln in body_lines:
        rows.append([Paragraph(ln, S_NOTE)])
    t = Table(rows, colWidths=[PAGE_W - 2 * MARGIN])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), AMBERBG),
        ("BOX", (0, 0), (-1, -1), 0.8, AMBER),
        ("LEFTPADDING", (0, 0), (-1, -1), 10),
        ("RIGHTPADDING", (0, 0), (-1, -1), 10),
        ("TOPPADDING", (0, 0), (-1, -1), 7),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
    ]))
    return t


def build(path):
    doc = BaseDocTemplate(
        path, pagesize=letter,
        leftMargin=MARGIN, rightMargin=MARGIN,
        topMargin=0.95 * inch, bottomMargin=0.7 * inch,
        title="HAB Payload Stabilization System",
        author="K6ATV",
    )
    frame = Frame(MARGIN, 0.7 * inch, PAGE_W - 2 * MARGIN,
                  PAGE_H - 1.65 * inch, id="main")
    doc.addPageTemplates([PageTemplate(id="all", frames=[frame],
                                       onPage=header_footer)])

    e = []  # story

    # ── Title block ──────────────────────────────────────────────────────────
    e.append(Paragraph("HAB PAYLOAD STABILIZATION SYSTEM", S_TITLE))
    e.append(Paragraph("Inverted Inertia Platform &mdash; As-Built State &amp; Upgrade Roadmap", S_SUB))
    e.append(Paragraph("K6ATV | June 2026 | Revision 1.1", S_SUB2))

    e.append(callout(
        "Design Summary: A HAB payload pointing system using a gimbal BLDC motor "
        "and four 500&nbsp;mm carbon-fibre arms to orient a passive 800&nbsp;g camera "
        "payload. No slip rings, no swashplate, no saturation condition, no active "
        "electronics on the payload. Payload heading is derived from an IMU on the "
        "arm plus a magnetic encoder reading a passive magnet &mdash; nothing electrical "
        "crosses the rotating interface."))

    # ── Status note up top (most important for readers after the launch) ─────
    e.append(Spacer(1, 8))
    e.append(note_box(
        "Status \u2014 June 2026 launch &amp; encoder upgrade in progress",
        [
            "This revision documents the system <b>as actually built and flown</b>. "
            "The AS5048A magnetic encoder is presently read in <b>PWM single-wire "
            "mode</b> (the cable that shipped pre-installed with the motor). The "
            "encoder hardware also supports SPI, and the <b>next planned upgrade is "
            "to move the encoder to SPI</b> to make rotation smoother &mdash; see "
            "&ldquo;Known Issue / Next Upgrade&rdquo; in Section&nbsp;7.",
        ]))

    e.append(Paragraph("1. How It Works", S_H1))
    e.append(Paragraph("1.1 Inverted Architecture", S_H2))
    e.append(Paragraph(
        "Rather than mounting a reaction wheel on the payload, this design inverts "
        "the concept. The arm assembly is the active control platform &mdash; it drives "
        "the payload to the correct heading. The GBM2804H gimbal outrunner motor "
        "connects the two assemblies: stator fixed to the arm, rotor bell fixed to "
        "the payload top plate. A 35&nbsp;mm bearing carries all structural loads "
        "independently of the motor.", S_BODY))
    e.append(Paragraph(
        "The GBM2804H is a <b>gimbal-wound BLDC</b>: a low-Kv (154 Kv), high "
        "pole-count machine (12N14P, 7 pole pairs) with relatively high phase "
        "resistance (~10&nbsp;\u03a9). Unlike a high-Kv drone motor built to spin fast, "
        "a gimbal motor is designed for smooth, high-torque positioning at very low "
        "RPM &mdash; which is exactly the regime this platform operates in, holding and "
        "nudging payload heading rather than rotating continuously. That low-RPM "
        "positioning duty is also what makes precise encoder feedback critical "
        "(see Section&nbsp;7).", S_BODY))

    e.append(Paragraph("1.2 Heading Sensing \u2014 Zero Payload Electronics", S_H2))
    e.append(Paragraph(
        "Payload absolute heading is computed entirely from sensors on the arm. The "
        "BNO085 provides the arm absolute world-frame yaw. The AS5048A (integrated "
        "into the motor) reads a magnet glued to the payload top plate. Summing them "
        "gives payload absolute heading across unlimited rotations &mdash; firmware tracks "
        "full turns to prevent wrap-around. The payload contributes only a passive "
        "N52 magnet (~1&nbsp;g).", S_BODY))

    e.append(Paragraph("1.3 No Saturation \u2014 Positioning Actuator", S_H2))
    e.append(Paragraph(
        "A conventional reaction wheel saturates by accumulating angular momentum "
        "until it hits maximum RPM. This design has no saturation: the motor is a "
        "positioning actuator. When the payload drifts, the motor drives it back. "
        "Nothing accumulates. Unlimited corrections over the full flight at no cost "
        "to authority.", S_BODY))

    e.append(Paragraph("1.4 The r\u00b2 Inertia Advantage", S_H2))
    e.append(Paragraph(
        "Placing electronics at 300&ndash;500&nbsp;mm radius instead of a compact central "
        "flywheel gives a large inertia advantage. At 800&nbsp;g payload the inertia "
        "ratio is about 25&times; &mdash; the arm resists corrections ~25 times more than the "
        "payload rotates, so motor corrections barely disturb the arm in normal "
        "operation.", S_BODY))

    inertia = [
        ["Configuration", "Mass", "Radius", "Inertia", "Ratio"],
        ["Compact 100 mm flywheel", "120 g", "50 mm", "1.5e-4 kg\u00b7m\u00b2", "1\u00d7"],
        ["Arm electronics (avg 400 mm)", "300 g", "~400 mm", "0.048 kg\u00b7m\u00b2", "320\u00d7"],
        ["Full arm assembly", "323 g", "various", "~0.040 kg\u00b7m\u00b2", "~267\u00d7"],
    ]
    e.append(make_table(inertia, [1.9*inch, 0.7*inch, 0.8*inch, 1.2*inch, 0.6*inch]))

    # ── Section 3 firmware ───────────────────────────────────────────────────
    e.append(Paragraph("2. Firmware \u2014 Control Loop", S_H1))
    e.append(Paragraph(
        f"Heading is computed at {CFG['PID_RATE']} on the Feather STM32F405. "
        f"payload_hdg = arm_yaw (BNO085) + motor_angle (AS5048A). A PID loop "
        f"({CFG['PID']}) drives the shortest-path angular error to zero, with a "
        f"{CFG['DEADBAND']} dead-band to avoid correcting trivial errors. The "
        f"SimpleFOC inner current loop runs at {CFG['FOC_RATE']}. Above "
        f"{CFG['MAG_CUTOFF']} the IMU switches to gyro-dominant heading "
        f"(magnetometer unreliable at altitude).", S_BODY))

    # ── Component spec ───────────────────────────────────────────────────────
    e.append(Paragraph("3. Component Specification (Arm Assembly)", S_H1))
    comp = [
        ["Component", "Part / Specification", "Mass", "Notes"],
        ["Gimbal BLDC Motor + Encoder", "iPower GBM2804H-100T (gimbal-wound) w/ integrated AS5048A; 12N14P, 10\u2009\u03a9, 154 Kv", "51 g", "Low-RPM positioning; encoder in PWM mode (see \u00a77)"],
        ["Motor Driver", "SimpleFOC Mini v1.1; DRV8313, 3-PWM, 5A max", "8 g", "Onboard 3.3V regulator"],
        ["Microcontroller", "Feather STM32F405 Express; 168 MHz M4F", "5 g", "Logic from Mini 3.3V; never 9V to BAT"],
        ["IMU", "Adafruit BNO085 #4754; onboard fusion", "3 g", f"I2C addr {CFG['BNO085_ADDR']}; gyro-only >15 km"],
        ["Bearing", "35 mm ID deep groove, stainless", "20 g", "Carries all structural loads"],
        ["Arms \u00d74", "6 mm OD 1 mm wall CF tube, 500 mm, 90\u00b0", "128 g", "32 g each; cross-pattern"],
        ["Battery", "6\u00d7 Energizer L91 AA, series, 9V, 3000 mAh", "87 g", "Rated -40C; ~4.3 h cold-derated"],
        ["Frame / Wiring", "PCB mounts, connectors, harness", "30 g", "Estimate"],
    ]
    e.append(make_table(comp, [1.25*inch, 2.5*inch, 0.55*inch, 1.9*inch]))
    e.append(Spacer(1, 3))
    e.append(Paragraph("Arm assembly total: ~332 g.", S_CAP))

    # ── Electrical pin map — CORRECTED ───────────────────────────────────────
    e.append(Paragraph("4. Electrical \u2014 Feather STM32F405 Pin Mapping", S_H1))
    e.append(Paragraph(
        "Pin assignments below match <b>src/config.h</b> as-built. The three motor "
        "IN pins are all on hardware timer channels; EN is any digital pin. The "
        "AS5048A is currently a single PWM line (not SPI &mdash; see Section&nbsp;7).", S_BODY))
    pins = [
        ["Signal", "Feather Pin", "Bus / Mode", "Notes"],
        ["SimpleFOC IN1", CFG["IN1"], "PWM", "Motor phase A"],
        ["SimpleFOC IN2", CFG["IN2"], "PWM", "Motor phase B"],
        ["SimpleFOC IN3", CFG["IN3"], "PWM", "Motor phase C"],
        ["SimpleFOC EN", CFG["EN"], "Digital", "Driver enable"],
        ["AS5048A angle", CFG["ENCODER"], "PWM in", "MagneticSensorPWM; SPI is the planned upgrade"],
        ["BNO085 SDA/SCL", "PB7 / PB6 (STEMMA QT)", "I2C1", f"addr {CFG['BNO085_ADDR']}"],
        ["Battery sense", "A0 (PA4)", "ADC", "10k/3.3k divider, ratio 4.03"],
        ["Motor + logic supply", "Mini VIN \u2190 9V; Feather 3V \u2190 Mini 3.3V", "Power", "Never connect 9V to Feather BAT/USB"],
    ]
    e.append(make_table(pins, [1.4*inch, 2.0*inch, 0.8*inch, 2.0*inch]))
    e.append(Spacer(1, 4))
    e.append(Paragraph(
        f"Telemetry: {CFG['TELEM']}. Battery warn {CFG['BATT_WARN']}, cutoff "
        f"{CFG['BATT_CUTOFF']}; motor voltage limit {CFG['VOLT_LIMIT']} from a "
        f"{CFG['SUPPLY_V']} nominal supply.", S_BODY))

    # ── Performance summary ──────────────────────────────────────────────────
    e.append(Paragraph("5. Performance Summary", S_H1))
    perf = [
        ["Parameter", "Value", "Notes"],
        ["Designed payload mass", "800 g", "Camera + enclosure + tracker"],
        ["Maximum payload mass", "1.2 kg", "10\u00d7 inertia margin maintained"],
        ["Inertia ratio at 800 g", "25\u00d7", "Arm 25\u00d7 harder to spin than payload"],
        ["Encoder resolution", "0.022\u00b0", "AS5048A 14-bit (sensor limit)"],
        ["Motor max torque", "~56 mN\u00b7m", "GBM2804H at 9V (peak)"],
        ["Saturation risk", "None", "Positioning actuator"],
        ["Flight duration covered", "2.8 h to 30 km", "Battery gives 4.3 h cold-derated"],
        ["Battery chemistry", "Li-FeS2 (L91)", "Rated -40C, functional to ~-55C"],
        ["Rotating electrical conn.", "Zero", "No slip rings anywhere"],
        ["Payload active electronics", "Zero", "Passive magnet only"],
    ]
    e.append(make_table(perf, [1.9*inch, 1.3*inch, 3.0*inch]))

    # ── Key decisions ────────────────────────────────────────────────────────
    e.append(Paragraph("6. Key Design Decisions", S_H1))
    for h, b in [
        ("Finless design",
         "At 800 g payload the arm inertia ratio is 25\u00d7 without fins. Ascending "
         "airflow creates velocity-dependent drag on bare CF arms opposing rotation. "
         "Fins can be retrofitted if testing shows excessive arm spin."),
        ("Motor with integrated AS5048A encoder",
         "The GBM2804H-100T with integrated AS5048A eliminates manual encoder "
         "alignment &mdash; the factory centres the magnet on the shaft axis. 14-bit "
         "resolution improves FOC smoothness at the near-zero RPM this system runs at."),
        ("6\u00d7 AA Energizer L91 in series (9V)",
         "Rated -40C, functional to ~-55C float altitude. 3000 mAh gives ~4.3 h "
         "cold-derated runtime vs ~2.8 h flight. LiPo cells degrade severely at "
         "float altitude; L91 cost ~$10 per flight, 87 g total."),
        ("Bearing separate from motor load path",
         "The 35 mm bearing carries all payload weight and pendulum swing forces. "
         "The motor handles only rotational torque, keeping it in its designed "
         "torque-only regime."),
    ]:
        e.append(Paragraph(h, S_H2))
        e.append(Paragraph(b, S_BODY))

    # ── Known issue / next upgrade — the new content ─────────────────────────
    e.append(Paragraph("7. Known Issue / Next Upgrade \u2014 Encoder PWM \u2192 SPI", S_H1))
    e.append(note_box(
        "Lesson learned \u2014 June 2026 launch",
        [
            "The AS5048A shipped with a pre-installed <b>PWM output cable</b>, so the "
            "encoder was wired and flown in PWM mode to save bring-up time. Bench "
            "testing of the PWM readings looked accurate enough at a glance, so it "
            "appeared safe to fly that way.",
            "<b>In flight / testing this proved insufficient.</b> A gimbal motor "
            "run as a fine-positioning actuator at near-zero RPM needs very exact, "
            "low-latency angle data to commutate smoothly. The PWM single-wire "
            "decode adds enough timing jitter and latency that the control loop "
            "exhibited twitching and related instability &mdash; static readings can look "
            "fine while the loop still lacks the precision it needs.",
            "<b>Next upgrade:</b> move the AS5048A to its native <b>SPI</b> interface "
            "(MISO/SCK/MOSI/CSN on SPI2), replacing MagneticSensorPWM with "
            "MagneticSensorSPI in firmware. SPI gives the deterministic, "
            "high-resolution angle reads the FOC loop needs and is expected to "
            "substantially smooth rotation. This is the primary planned change "
            "before the next flight.",
        ]))

    # ── Remaining work ───────────────────────────────────────────────────────
    e.append(Paragraph("8. Remaining Work Before Next Flight", S_H1))
    work = [
        ["Area", "Task", "Priority"],
        ["Firmware", "Convert encoder PWM \u2192 SPI (MagneticSensorSPI, SPI2 + CSN)", "High"],
        ["Firmware", "Re-verify encoder sign convention on bench after SPI swap", "High"],
        ["Firmware", "Re-tune PID with SPI encoder (start Kp=0.5 Ki=0.02 Kd=0.08)", "High"],
        ["Mechanical", "Arm-to-rotor-bell interface plate + AS5048A magnet mount", "High"],
        ["Mechanical", "Verify arm mass balance (asymmetry causes pendulum oscillation)", "High"],
        ["Firmware", "Add UART altitude input from tracker for mag cutoff", "Medium"],
        ["Thermal", "Freezer test BNO085 and AS5048A at -20C; characterise drift", "High"],
        ["Testing", "Bench: hold payload heading while manually rotating arm rig", "High"],
    ]
    e.append(make_table(work, [1.0*inch, 4.2*inch, 0.9*inch]))

    e.append(Spacer(1, 10))
    e.append(Paragraph(
        "As-built + roadmap &mdash; K6ATV &mdash; June 2026. github.com/radiohound &mdash; "
        "Specifications subject to revision following the encoder SPI conversion "
        "and prototype testing.", S_CAP))

    doc.build(e)
    print(f"wrote {path}")


if __name__ == "__main__":
    import os
    out = os.path.join(os.path.dirname(__file__), "HAB_Stabilization_v2.pdf")
    build(out)
