# PGM Editor

A standalone Python application for creating and editing PGM (Portable Gray Map) image files.

## Features

- Create new PGM images with custom dimensions (default: 128x128)
- Open existing PGM files (P2/ASCII and P5/binary formats)
- Drawing tools with adjustable properties:
  - Pencil, eraser, line, rectangle, circle tools
  - Adjustable brush size, grayscale value, shape, and hardness
- Save images in PGM format
- Undo/redo functionality
- Zoom and pan capabilities

## Installation

1. Clone this repository
2. Install the required dependencies:

```bash
pip install -r requirements.txt
```

## Usage

Run the application:

```bash
python main.py
```

### Creating a New Image

1. Select `File > New` from the menu
2. Enter the desired width and height
3. Click `Create`

### Opening an Existing Image

1. Select `File > Open` from the menu
2. Navigate to and select a PGM file
3. Click `Open`

### Drawing Tools

- **Pencil**: Freehand drawing
- **Eraser**: Erase parts of the image
- **Line**: Draw straight lines
- **Rectangle**: Draw rectangles
- **Circle**: Draw circles
- **Fill**: Fill an area with a specific grayscale value

### Adjusting Brush Properties

- **Size**: Change the brush size
- **Value**: Set the grayscale value (0-255)
- **Shape**: Choose between circle, square, or custom shapes
- **Hardness**: Adjust the edge softness of the brush

### Saving Your Work

1. Select `File > Save` or `File > Save As` from the menu
2. Choose a location and filename
3. Select the PGM format type (P2/ASCII or P5/binary)
4. Click `Save`

## Development

### Project Structure

```
pgm_editor/
├── main.py                  # Entry point
├── requirements.txt         # Dependencies
├── README.md                # Documentation
├── model/                   # Data and business logic
├── view/                    # GUI components
├── controller/              # Application control logic
└── tests/                   # Test suite
```

### Running Tests

```bash
pytest
```

For test coverage report:

```bash
pytest --cov=pgm_editor
```

## License

MIT
