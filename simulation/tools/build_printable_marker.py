"""Genera en A4 una tarjeta AprilTag de 65 mm, incluido su margen blanco.

Ejemplo desde el repositorio:
    .venv/bin/python -m simulation.tools.build_printable_marker

No ejecuta ni modifica el generador histórico del patrón de 40 mm. El dibujo
es vectorial: ocho celdas de 6,5 mm forman el patrón de 52 mm y el margen
añade 6,5 mm por cada lado hasta completar la tarjeta exterior de 65 mm.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import shutil

import cv2


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = ROOT.parent / "output/pdf/RobotRacers_AprilTag_65mm_A4.pdf"
DEFAULT_ASSET = ROOT / "simulation/webcam/assets/apriltag-0-65mm-a4.pdf"
CARD_MM = 65.0
PATTERN_MM = 52.0
WHITE_MARGIN_MM = 6.5
CELL_MM = 6.5
MARKER_ID = 0


def build(output: Path = DEFAULT_OUTPUT, asset: Path = DEFAULT_ASSET) -> Path:
    """Dibuja una sola hoja A4 y guarda una copia idéntica como recurso web."""
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfgen.canvas import Canvas

    output, asset = Path(output), Path(asset)
    output.parent.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    cells = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, 8)
    if cells.shape != (8, 8) or CELL_MM * 8 != PATTERN_MM:
        raise RuntimeError("La geometría del patrón AprilTag no es la esperada.")
    if PATTERN_MM + 2 * WHITE_MARGIN_MM != CARD_MM:
        raise RuntimeError("El patrón y su margen deben completar exactamente 65 mm.")

    pdf = Canvas(str(output), pagesize=A4, pageCompression=1, invariant=1)
    pdf.setTitle("RobotRacers - AprilTag 36h11 ID 0 - tarjeta exterior de 65 mm")
    pdf.setAuthor("RobotRacers")
    pdf.setSubject("Imprimir al 100%: tarjeta 65 x 65 mm, patrón 52 x 52 mm")
    page_width, page_height = A4
    card_side = CARD_MM * mm
    card_x = (page_width - card_side) / 2
    card_y = (page_height - card_side) / 2
    pattern_x = card_x + WHITE_MARGIN_MM * mm
    pattern_y = card_y + WHITE_MARGIN_MM * mm

    ink = (.075, .18, .22)
    muted = (.29, .38, .4)
    pdf.setFillColorRGB(*ink)
    pdf.setFont("Helvetica-Bold", 11)
    pdf.drawString(25 * mm, page_height - 25 * mm, "ROBOTRACERS")
    pdf.setFont("Helvetica-Bold", 23)
    pdf.drawString(25 * mm, page_height - 38 * mm, "AprilTag - tarjeta de 65 mm")
    pdf.setFont("Helvetica", 11)
    pdf.setFillColorRGB(*muted)
    for text, top_mm in [
        ("Familia 36h11 - identificador 0", 49),
        ("Imprimir al 100% o Tamaño real. Desactivar Ajustar a página.", 61),
        ("Tarjeta exterior: 65 x 65 mm, incluido el blanco.", 70),
        ("Patrón negro: 52 x 52 mm. Margen blanco: 6,5 mm por lado.", 79),
    ]:
        pdf.drawString(25 * mm, page_height - top_mm * mm, text)

    # La tarjeta completa es blanca. No hay rótulos ni flechas dentro de ella.
    pdf.setFillColorRGB(1, 1, 1)
    pdf.rect(card_x, card_y, card_side, card_side, fill=1, stroke=0)
    pdf.setFillColorRGB(0, 0, 0)
    for row, values in enumerate(cells):
        for column, value in enumerate(values):
            if value == 0:
                pdf.rect(pattern_x + column * CELL_MM * mm,
                         pattern_y + (7 - row) * CELL_MM * mm,
                         CELL_MM * mm, CELL_MM * mm, fill=1, stroke=0)

    # El borde interior del trazo coincide con los 65 mm: toda la tinta de
    # recorte queda fuera de la zona blanca, sin reducir el margen del tag.
    cut_stroke = .25
    pdf.setStrokeColorRGB(.72, .76, .77)
    pdf.setLineWidth(cut_stroke)
    pdf.setDash(2, 2)
    pdf.rect(card_x - cut_stroke / 2, card_y - cut_stroke / 2,
             card_side + cut_stroke, card_side + cut_stroke, fill=0, stroke=1)
    pdf.setDash()

    # El detector usa el borde derecho del patrón canónico como frente.
    arrow_start = card_x + card_side + 8 * mm
    arrow_end = arrow_start + 22 * mm
    arrow_y = page_height / 2
    pdf.setFillColorRGB(*ink)
    pdf.setFont("Helvetica-Bold", 10)
    pdf.drawCentredString((arrow_start + arrow_end) / 2,
                         arrow_y + 7 * mm, "FRENTE")
    pdf.setStrokeColorRGB(*ink)
    pdf.setLineWidth(1.2)
    pdf.line(arrow_start, arrow_y, arrow_end, arrow_y)
    pdf.line(arrow_end - 4 * mm, arrow_y + 2.2 * mm, arrow_end, arrow_y)
    pdf.line(arrow_end - 4 * mm, arrow_y - 2.2 * mm, arrow_end, arrow_y)

    pdf.setFillColorRGB(*muted)
    pdf.setFont("Helvetica", 10)
    pdf.drawCentredString(page_width / 2, card_y - 10 * mm,
                         "Recorta por el borde interior de la línea gris.")
    pdf.drawCentredString(page_width / 2, card_y - 17 * mm,
                         "Conserva completos los cuatro márgenes blancos.")

    rule_x, rule_y = (page_width - 50 * mm) / 2, 75 * mm
    pdf.setFillColorRGB(*ink)
    pdf.setFont("Helvetica-Bold", 11)
    pdf.drawCentredString(page_width / 2, rule_y + 11 * mm, "Comprueba la escala con una regla")
    pdf.setStrokeColorRGB(*ink)
    pdf.setLineWidth(.8)
    pdf.line(rule_x, rule_y, rule_x + 50 * mm, rule_y)
    for x in (rule_x, rule_x + 50 * mm):
        pdf.line(x, rule_y - 2 * mm, x, rule_y + 2 * mm)
    pdf.setFont("Helvetica", 10)
    pdf.drawCentredString(page_width / 2, rule_y - 7 * mm, "50 mm")
    pdf.setFillColorRGB(*muted)
    for text, bottom_mm in [
        ("Si la barra no mide 50 mm, corrige la escala de impresión.", 53),
        ("Usa papel mate sobre una base rígida, horizontal y aislante.", 43),
        ("Orienta el borde derecho del patrón hacia el frente del robot.", 34),
        ("La flecha de esta hoja es una referencia; queda fuera de la tarjeta.", 25),
    ]:
        pdf.drawCentredString(page_width / 2, bottom_mm * mm, text)
    pdf.showPage()
    pdf.save()

    if asset.resolve() != output.resolve():
        asset.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(output, asset)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help="PDF de entrega A4, una página")
    parser.add_argument("--asset", type=Path, default=DEFAULT_ASSET,
                        help="Copia idéntica usada como recurso de la aplicación")
    args = parser.parse_args()
    final = build(args.output, args.asset)
    print(final)
    if args.asset.resolve() != final.resolve():
        print(args.asset)


if __name__ == "__main__":
    main()
