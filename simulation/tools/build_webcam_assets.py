"""Genera el marcador de prueba y un PDF vectorial a escala real."""
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "simulation/webcam/assets"


def build(output: Path = OUTPUT) -> None:
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfgen.canvas import Canvas

    output.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    cells = cv2.aruco.generateImageMarker(dictionary, 0, 8)
    # 20 píxeles/mm: patrón 800 px, margen 100 px, sin interpolación.
    marker = cv2.resize(cells, (800, 800), interpolation=cv2.INTER_NEAREST)
    printable = cv2.copyMakeBorder(marker, 100, 100, 100, 100,
                                  cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(str(output / "apriltag-0-margen.png"), printable)
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(printable)
    if ids is None or ids.flatten().tolist() != [0]:
        raise RuntimeError("El patrón generado no se reconoce como AprilTag 0.")

    # PNG sin rótulos: exclusivamente marcador y margen blanco.
    # PDF: tarjeta 50 x 62 mm, con rótulo separado de la zona blanca de 50 x 50.
    pdf = Canvas(str(output / "apriltag-0-40mm.pdf"), pagesize=A4)
    width, height = A4
    pdf.setTitle("AprilTag 0 - patrón de 40 mm - RobotRacers")
    pdf.setAuthor("RobotRacers")
    pdf.setFillColorRGB(.06, .1, .14)
    pdf.setFont("Helvetica-Bold", 22)
    pdf.drawString(25 * mm, height - 30 * mm, "Marcador para la webcam")
    pdf.setFont("Helvetica", 11)
    for line, y in [
        ("AprilTag 36h11 - identificador 0", height - 41 * mm),
        ("Imprimir al 100% / Tamaño real. Desactivar ajustar a página.", height - 51 * mm),
        ("Patrón negro: 40 x 40 mm. Margen blanco: 5 mm por lado.", height - 58 * mm),
    ]:
        pdf.drawString(25 * mm, y, line)

    x = (width - 50 * mm) / 2
    y = height - 140 * mm
    pdf.setFillColorRGB(1, 1, 1)
    pdf.rect(x, y - 12 * mm, 50 * mm, 62 * mm, fill=1, stroke=0)
    pdf.setFillColorRGB(0, 0, 0)
    for row, values in enumerate(cells):
        for column, value in enumerate(values):
            if value == 0:
                pdf.rect(x + (5 + column * 5) * mm,
                         y + (5 + (7 - row) * 5) * mm,
                         5 * mm, 5 * mm, fill=1, stroke=0)
    pdf.setStrokeColorRGB(.65, .68, .71)
    pdf.setLineWidth(.35)
    pdf.setDash(2, 2)
    pdf.rect(x, y - 12 * mm, 50 * mm, 62 * mm, fill=0, stroke=1)
    pdf.setDash()
    pdf.setFillColorRGB(.05, .12, .16)
    pdf.setFont("Helvetica-Bold", 9)
    pdf.drawString(x + 5 * mm, y - 7 * mm, "FRENTE")
    pdf.setStrokeColorRGB(.05, .12, .16)
    pdf.setLineWidth(1.2)
    pdf.line(x + 28 * mm, y - 6 * mm, x + 43 * mm, y - 6 * mm)
    pdf.line(x + 39 * mm, y - 8 * mm, x + 43 * mm, y - 6 * mm)
    pdf.line(x + 39 * mm, y - 4 * mm, x + 43 * mm, y - 6 * mm)
    pdf.setFont("Helvetica", 10)
    pdf.drawCentredString(width / 2, y - 23 * mm, "Recortar por el borde exterior de puntos.")
    pdf.drawCentredString(width / 2, y - 30 * mm, "Tarjeta 50 x 62 mm, incluido el rótulo de orientación.")

    rule_x, rule_y = 25 * mm, 70 * mm
    pdf.setFont("Helvetica-Bold", 11)
    pdf.drawString(rule_x, rule_y + 10 * mm, "Comprueba la escala antes de recortar")
    pdf.setLineWidth(.8)
    pdf.line(rule_x, rule_y, rule_x + 40 * mm, rule_y)
    for endpoint in (rule_x, rule_x + 40 * mm):
        pdf.line(endpoint, rule_y - 2 * mm, endpoint, rule_y + 2 * mm)
    pdf.setFont("Helvetica", 10)
    pdf.drawCentredString(rule_x + 20 * mm, rule_y - 7 * mm, "40 mm")
    for line, offset in [
        ("Usar papel mate sobre una base rígida, horizontal y aislante.", 22),
        ("Mantener el margen blanco limpio y la flecha hacia el frente del robot.", 29),
        ("La cámara verá el centro del marcador; no es el centro del robot.", 36),
    ]:
        pdf.drawString(rule_x, rule_y - offset * mm, line)
    pdf.showPage()
    pdf.save()
    print(output / "apriltag-0-40mm.pdf")


if __name__ == "__main__":
    build()
