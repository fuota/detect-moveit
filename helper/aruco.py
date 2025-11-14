import cv2
import numpy as np
from reportlab.pdfgen import canvas
from reportlab.lib.units import cm
from reportlab.lib.utils import ImageReader
from PIL import Image
import io

#version check
subset = 1
# ArUco dictionary for 4x4 markers (supports up to 50 markers)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Marker size in pixels (approximately 2.5 cm at 300 DPI)
marker_pixels = 250
start_marker_id = 25
end_marker_id = 50

# Generate 50 marker images
marker_images = []
for i in range(start_marker_id, end_marker_id):
    marker = cv2.aruco.generateImageMarker(dictionary, i, marker_pixels)
    # Convert OpenCV image to PIL Image
    pil_img = Image.fromarray(marker)
    marker_images.append(pil_img)

# PDF page dimensions (A4-like but taller to fit all markers)
page_width = 21 * cm
page_height = 31 * cm

# Create PDF canvas
c = canvas.Canvas(f"aruco_markers_{subset}.pdf", pagesize=(page_width, page_height))

# Set font for marker IDs
c.setFont("Helvetica", 8)

# Layout parameters
markers_per_row = 6
marker_size = 2.5 * cm
spacing = 0.5 * cm

# Calculate left margin to center the rows
left_margin = (page_width - (markers_per_row * marker_size + (markers_per_row - 1) * spacing)) / 2

# Top margin
top_margin = 2 * cm
y = page_height - top_margin - marker_size

# Place markers
col = 0
for idx, i in enumerate(range(start_marker_id, end_marker_id)):
    x = left_margin + col * (marker_size + spacing)
    
    # Draw the ID text above the marker
    y_text = y + marker_size + 0.2 * cm
    c.drawCentredString(x + marker_size / 2, y_text, f"ID: {i}")
    
    # Draw the marker on the PDF
    c.drawImage(ImageReader(marker_images[idx]), x, y, width=marker_size, height=marker_size)
    
    col += 1
    if col == markers_per_row:
        col = 0
        y -= (marker_size + spacing + 0.5 * cm)  # Move to next row

# Save the PDF
c.save()

print(f"PDF generated: aruco_markers_{subset}.pdf")
